# Copyright 2026 DroneSwarming maintainers.
"""Own the PX4 SITL process group for each flight case."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import TextIO

CASE_PROFILES: dict[str, tuple[str, ...]] = {
    'smoke': ('FT-00', 'FT-01', 'FT-06'),
    'stress': ('FT-02', 'FT-03', 'FT-04', 'FT-05'),
}
ROS_DOMAIN_ID: str = '77'
PX4_ROOTFS_DIRECTORY: Path = Path('/opt/PX4-Autopilot/build/px4_sitl_default/rootfs')
RUN_LOG_ROOT: Path = Path('/workspace/run-logs')
PROCESS_STOP_TIMEOUT_SEC: float = 10.0
CASE_TIMEOUT_SEC: float = 360.0


class FlightTestError(RuntimeError):
    """Represent a flight suite setup, process, or report failure."""


def parse_arguments(arguments: list[str] | None = None) -> argparse.Namespace:
    """Parse one profile or one explicitly selected flight case."""
    parser = argparse.ArgumentParser(
        description='Run isolated PX4 SITL flight tests and save evidence.'
    )
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--profile', choices=tuple(CASE_PROFILES))
    selection.add_argument(
        '--case',
        choices=tuple(case_id for case_ids in CASE_PROFILES.values() for case_id in case_ids),
    )
    parser.add_argument('--commit', required=True)
    parser.add_argument('--dirty', choices=('true', 'false'), required=True)
    return parser.parse_args(arguments)


def resolve_run_directory() -> Path:
    """Validate the host-persisted artifact directory passed by sim-shell."""
    directory_value: str | None = os.environ.get('DRONE_RUN_DIR')
    if not directory_value:
        raise FlightTestError('DRONE_RUN_DIR is required; start through scripts/test-flights')
    directory: Path = Path(directory_value).resolve()
    if not directory.is_dir() or not os.access(directory, os.W_OK):
        raise FlightTestError(f'flight artifact directory is not writable: {directory}')
    return directory


def selected_cases(options: argparse.Namespace) -> tuple[str, ...]:
    """Resolve the selected profile or explicit case into ordered IDs."""
    if options.case is not None:
        return (str(options.case),)
    if options.profile is None:
        raise FlightTestError('select either --profile or --case')
    return CASE_PROFILES[str(options.profile)]


def check_udp_port(port_number: int) -> None:
    """Reject a run when another process already owns a PX4 network port."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port_probe:
        try:
            port_probe.bind(('0.0.0.0', port_number))
        except OSError as exception:
            raise FlightTestError(
                f'UDP port {port_number} is in use; refusing to start SITL'
            ) from exception


def start_process(
    command: list[str], environment: dict[str, str], log_path: Path
) -> tuple[subprocess.Popen[bytes], TextIO]:
    """Start an owned process group and capture its output to a log file."""
    log_stream: TextIO = log_path.open('w', encoding='utf-8')
    try:
        process: subprocess.Popen[bytes] = subprocess.Popen(
            command,
            cwd='/workspace',
            env=environment,
            stdin=subprocess.PIPE,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError:
        log_stream.close()
        raise
    return process, log_stream


def prepare_px4_log_directory(run_directory: Path) -> None:
    """Route PX4 ULog output into the host-persisted run directory."""
    px4_rootfs: Path = PX4_ROOTFS_DIRECTORY
    run_log_directory: Path = run_directory / 'px4/log'
    run_log_directory.mkdir(parents=True, exist_ok=True)
    permitted_log_root: Path = RUN_LOG_ROOT.resolve()
    for px4_log_directory in (px4_rootfs / 'log', px4_rootfs / 'fs/microsd/log'):
        px4_log_directory.parent.mkdir(parents=True, exist_ok=True)
        if px4_log_directory.is_symlink():
            existing_target: Path = px4_log_directory.resolve()
            if existing_target == run_log_directory.resolve():
                continue
            if not existing_target.is_relative_to(permitted_log_root):
                raise FlightTestError(
                    f'PX4 log link points outside the run archive: {existing_target}'
                )
            px4_log_directory.unlink()
        if px4_log_directory.exists():
            if not px4_log_directory.is_dir():
                raise FlightTestError('PX4 log path exists and is not a directory')
            if any(px4_log_directory.iterdir()):
                relative_directory: str = px4_log_directory.relative_to(px4_rootfs).as_posix()
                log_path_name: str = relative_directory.replace('/', '-')
                if relative_directory == 'log':
                    log_path_name = f'rootfs-{log_path_name}'
                archive_name: str = f'preexisting-{log_path_name}'
                archived_directory: Path = run_directory / 'px4' / archive_name
                suffix: int = 0
                while archived_directory.exists() or archived_directory.is_symlink():
                    suffix += 1
                    archived_directory = run_directory / 'px4' / f'{archive_name}-{suffix}'
                try:
                    shutil.move(px4_log_directory, archived_directory)
                except OSError as exception:
                    raise FlightTestError(
                        f'could not archive existing PX4 log path {px4_log_directory}'
                    ) from exception
                px4_log_directory.symlink_to(run_log_directory, target_is_directory=True)
                continue
            px4_log_directory.rmdir()
        px4_log_directory.symlink_to(run_log_directory, target_is_directory=True)


def stop_process(process: subprocess.Popen[bytes], log_stream: TextIO) -> int:
    """Stop an owned process group and return its normalized exit status."""
    was_running: bool = process.poll() is None
    if was_running:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=PROCESS_STOP_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=PROCESS_STOP_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
    log_stream.flush()
    log_stream.close()
    if process.stdin is not None:
        process.stdin.close()
    status: int = process.returncode if process.returncode is not None else 1
    if was_running and status in (-signal.SIGINT, -signal.SIGTERM):
        return 0
    return 128 + abs(status) if status < 0 else status


def run_case(case_id: str, artifact_root: Path, commit: str, dirty_tree: bool) -> dict[str, object]:
    """Start SITL, run one case, and save its terminal status."""
    case_directory: Path = artifact_root / 'flight-tests' / case_id
    case_directory.mkdir(parents=True, exist_ok=False)
    environment: dict[str, str] = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID
    environment['HEADLESS'] = '1'
    environment.pop('PX4_GZ_WORLD', None)
    environment['PX4_PARAM_SDLOG_MODE'] = '2'

    process_logs: dict[str, TextIO] = {}
    processes: dict[str, subprocess.Popen[bytes]] = {}
    lock_stream: TextIO | None = None
    case_result: dict[str, object] = {
        'case_id': case_id,
        'commit': commit,
        'dirty_tree': dirty_tree,
        'started_at_utc': datetime.now(UTC).isoformat(),
        'status': 'failed',
        'message': 'case did not complete',
        'process_exit_codes': {},
        'simulation': {
            'ros_domain_id': int(ROS_DOMAIN_ID),
            'px4_topic_prefix': '/fmu',
            'px4_system_id': 1,
            'px4_model': 'gz_x500',
            'headless': True,
        },
    }
    try:
        check_udp_port(8888)
        check_udp_port(14550)
        px4_lock_path: Path = Path(
            '/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/.drone-sitl.lock'
        )
        lock_stream = px4_lock_path.open('a', encoding='utf-8')
        try:
            fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exception:
            raise FlightTestError('another PX4 SITL process owns the rootfs lock') from exception
        prepare_px4_log_directory(artifact_root)

        agent, agent_log = start_process(
            ['MicroXRCEAgent', 'udp4', '-p', '8888'],
            environment,
            case_directory / 'xrce-agent.log',
        )
        processes['xrce-agent'] = agent
        process_logs['xrce-agent'] = agent_log
        time.sleep(1.0)
        if agent.poll() is not None:
            raise FlightTestError('MicroXRCEAgent exited before PX4 startup')

        px4, px4_log = start_process(
            ['make', '-C', '/opt/PX4-Autopilot', 'px4_sitl', 'gz_x500'],
            environment,
            case_directory / 'px4-sitl.log',
        )
        processes['px4-sitl'] = px4
        process_logs['px4-sitl'] = px4_log
        time.sleep(3.0)
        if px4.poll() is not None:
            raise FlightTestError('PX4 SITL exited during startup')

        application, application_log = start_process(
            [
                'ros2',
                'launch',
                'drone_bringup',
                'single_vehicle.launch.py',
                'enable_flight_commands:=true',
                'px4_topic_prefix:=/fmu',
                'px4_system_id:=1',
            ],
            environment,
            case_directory / 'application.log',
        )
        processes['application'] = application
        process_logs['application'] = application_log
        runner, runner_stream = start_process(
            [
                'ros2',
                'run',
                'flight_test',
                'flight_test_case',
                '--case',
                case_id,
                '--artifact-dir',
                str(case_directory),
            ],
            environment,
            case_directory / 'runner.log',
        )
        processes['runner'] = runner
        process_logs['runner'] = runner_stream
        runner_deadline: float = time.monotonic() + CASE_TIMEOUT_SEC
        while runner.poll() is None:
            failed_processes: list[str] = [
                process_name
                for process_name in ('xrce-agent', 'px4-sitl', 'application')
                if processes[process_name].poll() is not None
            ]
            if failed_processes:
                raise FlightTestError(
                    'owned process exited during the flight: ' + ', '.join(failed_processes)
                )
            if time.monotonic() >= runner_deadline:
                raise FlightTestError(f'case exceeded {CASE_TIMEOUT_SEC:.0f} second deadline')
            time.sleep(0.2)
        runner_result_path: Path = case_directory / 'case-result.json'
        if not runner_result_path.is_file():
            raise FlightTestError(
                f'case runner exited with status {runner.returncode} without writing '
                'case-result.json'
            )
        try:
            runner_result: dict[str, object] = json.loads(
                runner_result_path.read_text(encoding='utf-8')
            )
        except (json.JSONDecodeError, OSError) as exception:
            raise FlightTestError(f'could not read case-result.json: {exception}') from exception
        if runner.returncode != 0 or runner_result.get('status') != 'passed':
            failure: str = str(runner_result.get('failure', 'case assertions failed'))
            if runner.returncode != 0:
                failure = f'{failure} (case runner exit status {runner.returncode})'
            raise FlightTestError(failure)
        case_result['status'] = 'passed'
        case_result['message'] = 'all case assertions passed'
    except (FlightTestError, OSError, subprocess.TimeoutExpired) as exception:
        case_result['message'] = str(exception)
    finally:
        exit_codes: dict[str, int] = {}
        for process_name in ('runner', 'application', 'px4-sitl', 'xrce-agent'):
            process: subprocess.Popen[bytes] | None = processes.get(process_name)
            log_stream: TextIO | None = process_logs.get(process_name)
            if process is not None and log_stream is not None:
                exit_codes[process_name] = stop_process(process, log_stream)
        failed_cleanup_processes: dict[str, int] = {
            process_name: exit_code
            for process_name, exit_code in exit_codes.items()
            if process_name != 'runner' and exit_code != 0
        }
        if failed_cleanup_processes:
            case_result['status'] = 'failed'
            cleanup_message: str = 'owned process cleanup failed: ' + ', '.join(
                f'{process_name}={exit_code}'
                for process_name, exit_code in failed_cleanup_processes.items()
            )
            existing_message: str = str(case_result['message'])
            case_result['message'] = (
                f'{existing_message}; {cleanup_message}'
                if existing_message != 'case did not complete'
                else cleanup_message
            )
        case_result['process_exit_codes'] = exit_codes
        case_result['ended_at_utc'] = datetime.now(UTC).isoformat()
        result_path: Path = case_directory / 'process-result.json'
        result_path.write_text(json.dumps(case_result, indent=2) + '\n', encoding='utf-8')
        if lock_stream is not None:
            fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)
            lock_stream.close()
    return case_result


def write_profile_report(
    report_path: Path,
    profile_name: str,
    commit: str,
    dirty_tree: bool,
    case_results: list[dict[str, object]],
) -> None:
    """Write a Markdown summary linking the selected case results."""
    lines: list[str] = [
        f'# Flight test profile: {profile_name}',
        '',
        f'- Commit: `{commit}`',
        f'- Dirty tree: `{str(dirty_tree).lower()}`',
        f'- Written: `{datetime.now(UTC).isoformat()}`',
        '',
        '| Case | Status | Result |',
        '| --- | --- | --- |',
    ]
    for case_result in case_results:
        case_id: str = str(case_result['case_id'])
        status: str = str(case_result['status'])
        message: str = str(case_result['message']).replace('|', '\\|')
        case_report: str = f'{case_id}/case-report.md'
        process_report: str = f'{case_id}/process-result.json'
        lines.append(
            f'| [{case_id}]({case_report}) | {status} | {message}; '
            f'[process evidence]({process_report}) |'
        )
    report_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(arguments: list[str] | None = None) -> int:
    """Run selected SITL cases and return nonzero when a case fails."""
    options: argparse.Namespace = parse_arguments(arguments)
    try:
        artifact_root: Path = resolve_run_directory()
        case_ids: tuple[str, ...] = selected_cases(options)
    except FlightTestError as exception:
        print(f'flight-test setup failed: {exception}', file=sys.stderr)
        return 2

    commit: str = str(options.commit)
    dirty_tree: bool = options.dirty == 'true'
    profile_name: str = str(options.profile or options.case)
    profile_directory: Path = artifact_root / 'flight-tests'
    profile_directory.mkdir(parents=True, exist_ok=True)
    print(f'commit={commit} dirty_tree={str(dirty_tree).lower()}')
    print(f'cases={", ".join(case_ids)}')
    print(f'artifacts={profile_directory}')

    case_results: list[dict[str, object]] = []
    interrupted: bool = False
    for case_id in case_ids:
        print(f'starting {case_id}')
        try:
            case_result: dict[str, object] = run_case(case_id, artifact_root, commit, dirty_tree)
        except KeyboardInterrupt:
            interrupted = True
            case_result = {
                'case_id': case_id,
                'commit': commit,
                'dirty_tree': dirty_tree,
                'status': 'failed',
                'message': 'flight-test suite interrupted by the operator',
                'process_exit_codes': {},
            }
        except Exception as exception:
            case_result = {
                'case_id': case_id,
                'commit': commit,
                'dirty_tree': dirty_tree,
                'status': 'failed',
                'message': str(exception),
                'process_exit_codes': {},
            }
        case_results.append(case_result)
        print(f'{case_id}: {case_result["status"]} ({case_result["message"]})')
        if interrupted or case_result['status'] != 'passed':
            break

    report_path: Path = profile_directory / f'{profile_name.lower()}-profile.md'
    write_profile_report(report_path, profile_name, commit, dirty_tree, case_results)
    all_passed: bool = len(case_results) == len(case_ids) and all(
        str(result['status']) == 'passed' for result in case_results
    )
    print('flight-test profile: passed' if all_passed else 'flight-test profile: failed')
    return 130 if interrupted else (0 if all_passed else 1)


if __name__ == '__main__':
    raise SystemExit(main())
