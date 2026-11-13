"""Supervise isolated fault cases and retain native file logs and machine-readable results."""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import os
import platform
import signal
import subprocess
import sys
import time
import unittest
import uuid
import xml.etree.ElementTree as ElementTree
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path
from typing import cast

type JsonValue = None | bool | int | float | str | list[JsonValue] | dict[str, JsonValue]

CASE_WALL_LIMIT_SEC: float = 35.0
DEFAULT_SEED: int = 1731
FIRST_ROS_DOMAIN: int = 180
ROS_DOMAIN_COUNT: int = 20
SOURCE_DIRECTORY: Path = Path(__file__).resolve().parent
REPOSITORY_ROOT: Path = next(
    (path for path in SOURCE_DIRECTORY.parents if (path / '.git').exists()), SOURCE_DIRECTORY
)


def write_json(path: Path, value: JsonValue) -> None:
    """Replace complete JSON records atomically, including after a failing case."""
    temporary: Path = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def configure_logging(directory: Path) -> None:
    """Write runner diagnostics directly to a file without redirecting process streams."""
    logging.basicConfig(
        filename=directory / 'runner.log',
        level=logging.DEBUG,
        format='%(asctime)s %(levelname)s %(message)s',
        force=True,
    )


def child_case(directory: Path, case: str, seed: int) -> int:
    """Run one unittest with a direct file reporter and retain full failure tracebacks."""
    from suite import TortureTests

    TortureTests.artifact_directory = directory
    TortureTests.seed = seed
    test = TortureTests(case)
    started: float = time.monotonic()
    with (directory / 'case.log').open('w', encoding='utf-8') as report:
        result: unittest.TestResult = unittest.TextTestRunner(stream=report, verbosity=2).run(
            unittest.TestSuite([test])
        )
    errors: list[str] = [detail for _, detail in result.errors]
    failures: list[str] = [detail for _, detail in result.failures]
    status: str = 'error' if errors else ('failed' if failures else 'passed')
    peer = getattr(test, 'peer', None)
    details: list[str] = failures or errors
    record: dict[str, JsonValue] = {
        'case': case,
        'seed': seed,
        'status': status,
        'elapsed_sec': time.monotonic() - started,
        'errors': errors,
        'failures': failures,
        'cleanup_errors': list(peer.cleanup_errors) if peer is not None else [],
        'reason': details[0].splitlines()[-1] if details else '',
        'recent_events': list(peer.recent) if peer is not None else [],
        'event_counts': dict(peer.counts) if peer is not None else {},
        'peak_rss_kib': dict(peer.peak_rss_kib) if peer is not None else {},
        'maximum_heartbeat_gap_sec': max(
            (later - earlier for earlier, later in pairwise(peer.heartbeat_times)), default=None
        )
        if peer is not None
        else None,
        'log_files': [
            str(path.relative_to(directory))
            for path in directory.rglob('*')
            if path.is_file()
            and (path.suffix == '.log' or path.name.startswith(('asan.', 'ubsan.')))
        ],
    }
    write_json(directory / 'result.json', record)
    return 0 if status == 'passed' else 1


def recent_events(directory: Path) -> list[JsonValue]:
    """Recover bounded trailing evidence even when a child was killed before its report."""
    path: Path = directory / 'events.jsonl'
    if not path.exists():
        return []
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size - 65_536))
        lines: list[bytes] = stream.read().splitlines()[-40:]
    events: list[JsonValue] = []
    for line in lines:
        try:
            # The logger owns this JSON boundary; discard a partial first/last line after a crash.
            events.append(cast(JsonValue, json.loads(line)))
        except json.JSONDecodeError:
            continue
    return events


def supervise(directory: Path, case: str, seed: int, domain: int) -> dict[str, JsonValue]:
    """Use an independent wall-clock watchdog and kill the entire owned process group."""
    directory.mkdir()
    environment: dict[str, str] = dict(os.environ)
    for name in (
        'ROS_DISCOVERY_SERVER',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE',
        'CYCLONEDDS_URI',
    ):
        environment.pop(name, None)
    environment.update(
        {
            'ROS_DOMAIN_ID': str(domain),
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
            'ROS_LOCALHOST_ONLY': '1',
            'ROS_LOG_DIR': str(directory / 'observer'),
        }
    )
    arguments: list[str] = [
        sys.executable,
        str(directory.parent / 'test-source' / 'run.py'),
        '--child',
        '--case',
        case,
        '--seed',
        str(seed),
        '--artifact-dir',
        str(directory),
    ]
    started: float = time.monotonic()
    process = subprocess.Popen(
        arguments,
        env=environment,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    timeout: bool = False
    try:
        process.wait(timeout=CASE_WALL_LIMIT_SEC)
    except subprocess.TimeoutExpired:
        timeout = True
        logging.error('%s exceeded watchdog (%s seconds)', case, CASE_WALL_LIMIT_SEC)
    finally:
        # Children inherit this dedicated group. Cleanup also removes orphans after a normal exit.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3.0)
    path: Path = directory / 'result.json'
    if path.is_file() and not timeout:
        record: dict[str, JsonValue] = cast(dict[str, JsonValue], json.loads(path.read_text()))
        if process.returncode != 0 and record.get('status') == 'passed':
            record['status'] = 'error'
            record['errors'] = ['Child reported success but exited unsuccessfully']
    else:
        record = {
            'case': case,
            'seed': seed,
            'status': 'timeout' if timeout else 'error',
            'errors': [
                f'Case watchdog expired after {CASE_WALL_LIMIT_SEC}s'
                if timeout
                else 'Child exited before reporting; inspect runner.log and native ROS logs'
            ],
            'failures': [],
            'recent_events': recent_events(directory),
        }
        record['reason'] = 'Watchdog timeout' if timeout else 'Child exited without a result'
    record.update(
        {
            'elapsed_sec': time.monotonic() - started,
            'exit_code': process.returncode,
            'ros_domain_id': domain,
            'artifact_directory': directory.name,
        }
    )
    write_json(path, record)
    return record


def git_output(*arguments: str) -> str:
    result = subprocess.run(
        ['git', '-c', 'safe.directory=' + str(REPOSITORY_ROOT), *arguments],
        cwd=REPOSITORY_ROOT,
        check=True,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    return result.stdout.strip()


def reports(directory: Path, results: list[dict[str, JsonValue]]) -> None:
    """Persist partial progress, readable failure locations, and JUnit failure classification."""
    counts: dict[str, JsonValue] = {
        status: sum(result['status'] == status for result in results)
        for status in ('passed', 'failed', 'error', 'timeout')
    }
    write_json(directory / 'summary.json', {'counts': counts, 'cases': results})
    root: ElementTree.Element = ElementTree.Element(
        'testsuite',
        name='flight_torture',
        tests=str(len(results)),
        failures=str(counts['failed']),
        errors=str(int(cast(int, counts['error'])) + int(cast(int, counts['timeout']))),
    )
    lines: list[str] = [
        '# Flight torture results',
        '',
        'Synthetic PX4 protocol tests. These results do not establish vehicle dynamics or hardware readiness.',
        '',
        '| Case | Seed | Result | Seconds | Failure | Evidence |',
        '| --- | ---: | --- | ---: | --- | --- |',
    ]
    for result in results:
        name: str = str(result['case'])
        path: str = str(result['artifact_directory'])
        elapsed: float = cast(float, result['elapsed_sec'])
        case_element: ElementTree.Element = ElementTree.SubElement(
            root, 'testcase', name=name, classname='flight_torture', time=f'{elapsed:.3f}'
        )
        if result['status'] != 'passed':
            detail: str = json.dumps(
                {'errors': result['errors'], 'failures': result['failures']}, indent=2
            )
            failure: ElementTree.Element = ElementTree.SubElement(
                case_element,
                'failure' if result['status'] == 'failed' else 'error',
                type=str(result['status']),
                message=str(result.get('reason', '')) + f'; inspect {path}/result.json',
            )
            failure.text = detail
        reason: str = str(result.get('reason', ''))[:240].replace('|', '\\|').replace('\n', ' ')
        lines.append(
            f'| {name} | {result["seed"]} | {result["status"]} | {elapsed:.2f} | {reason} | [result]({path}/result.json), [timeline]({path}/events.jsonl), [assertions]({path}/case.log) |'
        )
    ElementTree.ElementTree(root).write(
        directory / 'junit.xml', encoding='utf-8', xml_declaration=True
    )
    (directory / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    """Run selected seeded cases without altering the application or hiding contract failures."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', action='append', help='exact test name; may be repeated')
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED)
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--artifact-dir', type=Path)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--child', action='store_true', help=argparse.SUPPRESS)
    options: argparse.Namespace = parser.parse_args()
    if not 1 <= options.repeat <= 1000 or not 0 <= options.seed <= 2**31 - options.repeat:
        parser.error('--repeat must be 1..1000 and --seed must be 0..2147483647')
    if options.child:
        if options.artifact_dir is None or options.case is None or len(options.case) != 1:
            parser.error('child requires one case and an existing artifact directory')
        directory: Path = options.artifact_dir.resolve()
        configure_logging(directory)
        try:
            return child_case(directory, options.case[0], options.seed)
        except Exception:
            logging.exception('Case bootstrap failed')
            return 2
    from suite import TortureTests

    available: list[str] = unittest.TestLoader().getTestCaseNames(TortureTests)
    if options.list:
        for name in available:
            print(name + ': ' + str(getattr(TortureTests, name).__doc__))
        return 0
    selected: list[str] = options.case or available
    if any(name not in available for name in selected):
        parser.error('unknown case; use --list')
    timestamp: str = datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ')
    directory = (
        options.artifact_dir
        or REPOSITORY_ROOT / 'run-logs' / 'torture' / (timestamp + '-' + uuid.uuid4().hex[:8])
    ).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    configure_logging(directory)
    source_archive: Path = directory / 'test-source'
    source_archive.mkdir()
    for path in SOURCE_DIRECTORY.glob('*.py'):
        (source_archive / path.name).write_bytes(path.read_bytes())
    metadata: dict[str, JsonValue] = {
        'started_at_utc': datetime.now(UTC).isoformat(),
        'git_commit': git_output('rev-parse', 'HEAD'),
        'git_status': git_output('status', '--porcelain'),
        'platform': platform.platform(),
        'python': sys.version,
        'seed': options.seed,
        'repeat': options.repeat,
        'case_wall_limit_sec': CASE_WALL_LIMIT_SEC,
        'cases': selected,
        'environment': {
            name: os.environ.get(name)
            for name in ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'TORTURE_IMAGE_ID')
        },
        'test_source_sha256': {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in source_archive.glob('*.py')
        },
    }
    write_json(directory / 'run.json', metadata)
    (directory / 'source.patch').write_text(
        git_output('diff', '--binary', 'HEAD') + '\n', encoding='utf-8'
    )
    results: list[dict[str, JsonValue]] = []
    print('Artifacts: ' + str(directory), flush=True)
    try:
        for repetition in range(options.repeat):
            for index, name in enumerate(selected):
                seed: int = options.seed + repetition
                case_directory: Path = directory / f'{len(results):03d}-{name}-s{seed}'
                result = supervise(
                    case_directory, name, seed, FIRST_ROS_DOMAIN + index % ROS_DOMAIN_COUNT
                )
                result['contract'] = getattr(TortureTests, name).__doc__
                results.append(result)
                reports(directory, results)
                print(
                    f'{str(result["status"]).upper():7} {name} ({float(cast(float, result["elapsed_sec"])):.2f}s)',
                    flush=True,
                )
    except KeyboardInterrupt:
        logging.exception('Suite interrupted; completed cases remain in summary.json')
        return 130
    except Exception:
        logging.exception('Suite infrastructure failed')
        return 2
    finally:
        reports(directory, results)
    print('Report: ' + str(directory / 'report.md'), flush=True)
    return 0 if all(result['status'] == 'passed' for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
