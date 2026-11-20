"""Run each blind wire test in a bounded process group and write replay artifacts."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

SUITE_DIRECTORY = Path(__file__).resolve().parent
DEFAULT_TIMEOUT_SEC = 90.0
MEMORY_LIMIT_BYTES = 1536 * 1024 * 1024
LOG_LIMIT_BYTES = 64 * 1024 * 1024
THREAD_LIMIT = 512
POLL_INTERVAL_SEC = 0.1
MAXIMUM_FUZZ_COUNT = 4096
MAXIMUM_TIMEOUT_SEC = 600.0


def discover() -> list[str]:
    """List this suite's test definitions without importing ROS or application modules."""
    names: list[str] = []
    for path in sorted(SUITE_DIRECTORY.glob('test_*.py')):
        module = ast.parse(path.read_text(encoding='utf-8'))
        for definition in module.body:
            if isinstance(definition, ast.ClassDef):
                names.extend(
                    f'{path.stem}.{definition.name}.{method.name}'
                    for method in definition.body
                    if isinstance(method, ast.FunctionDef) and method.name.startswith('test_')
                )
    return names


def usage(process_group: int) -> tuple[int, int]:
    """Read aggregate resident memory and thread counts for one worker process group."""
    resident_bytes = 0
    thread_count = 0
    page_bytes = os.sysconf('SC_PAGE_SIZE')
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            stat = (path / 'stat').read_text(encoding='utf-8').rsplit(')', 1)[1].split()
            if int(stat[2]) != process_group:
                continue
            resident_bytes += int(stat[21]) * page_bytes
            thread_count += int(stat[17])
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return resident_bytes, thread_count


def terminate_group(process: subprocess.Popen) -> None:
    """Reap the test and its node descendants, including descendants of exited workers."""
    for signum in [signal.SIGTERM, signal.SIGKILL]:
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            break
        if signum == signal.SIGTERM:
            deadline_sec = time.monotonic() + 2.0
            while time.monotonic() < deadline_sec:
                if usage(process.pid)[1] == 0:
                    break
                time.sleep(POLL_INTERVAL_SEC)
    process.wait(timeout=3.0)


def run_case(
    name: str, directory: Path, seed: int, fuzz_count: int, timeout_sec: float
) -> dict[str, str | int | float]:
    """Execute one replayable scenario with wall-time, memory, thread, and log budgets."""
    directory.mkdir(parents=True)
    environment = os.environ.copy()
    environment.update(
        {
            'ROS_DOMAIN_ID': '213',
            'ROS_LOCALHOST_ONLY': '1',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
            'ROS_LOG_DIR': str(directory / 'ros'),
            'BLIND_ARTIFACT_DIRECTORY': str(directory),
            'BLIND_SEED': str(seed),
            'BLIND_FUZZ_COUNT': str(fuzz_count),
            'PYTHONUNBUFFERED': '1',
        }
    )
    start_sec = time.monotonic()
    peak_bytes = 0
    peak_threads = 0
    failure = ''
    with (directory / 'worker.log').open('w', encoding='utf-8') as log_file:
        # Case names come from the suite's AST inventory; no shell interprets arguments.
        process = subprocess.Popen(  # noqa: S603
            [sys.executable, '-m', 'unittest', '-v', name],
            cwd=SUITE_DIRECTORY,
            env=environment,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            while process.poll() is None:
                resident_bytes, thread_count = usage(process.pid)
                peak_bytes = max(peak_bytes, resident_bytes)
                peak_threads = max(peak_threads, thread_count)
                log_bytes = sum(
                    path.stat().st_size for path in directory.rglob('*') if path.is_file()
                )
                if time.monotonic() - start_sec > timeout_sec:
                    failure = f'Watchdog exceeded {timeout_sec} seconds'
                elif resident_bytes > MEMORY_LIMIT_BYTES:
                    failure = f'Resident memory exceeded {MEMORY_LIMIT_BYTES} bytes'
                elif thread_count > THREAD_LIMIT:
                    failure = f'Thread count exceeded {THREAD_LIMIT}'
                elif log_bytes > LOG_LIMIT_BYTES:
                    failure = f'Artifacts exceeded {LOG_LIMIT_BYTES} bytes'
                if failure:
                    break
                time.sleep(POLL_INTERVAL_SEC)
        finally:
            terminate_group(process)
    if not failure and process.returncode != 0:
        failure = f'Worker exited {process.returncode}; see worker.log and wire.jsonl'
    result: dict[str, str | int | float] = {
        'case': name,
        'seed': seed,
        'fuzz_count': fuzz_count,
        'status': 'failed' if failure else 'passed',
        'failure': failure,
        'duration_sec': round(time.monotonic() - start_sec, 3),
        'peak_resident_bytes': peak_bytes,
        'peak_threads': peak_threads,
        'directory': str(directory),
    }
    (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    return result


def main() -> int:
    """Select scenarios, preserve failures, and return a CI-compatible exit status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='List scenarios without ROS imports')
    parser.add_argument(
        '--case',
        action='append',
        default=[],
        help='Run a case by exact listed name; repeatable',
    )
    parser.add_argument('--seed', type=int, default=20261119)
    parser.add_argument('--fuzz-count', type=int, default=64)
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=DEFAULT_TIMEOUT_SEC,
        help='Per-scenario process-group deadline',
    )
    parser.add_argument(
        '--artifact-dir',
        type=Path,
        default=Path('run-logs') / f'blind-{time.time_ns()}',
    )
    arguments = parser.parse_args()
    names = discover()
    if arguments.list:
        print('\n'.join(names))
        return 0
    if (
        arguments.fuzz_count < 1
        or arguments.fuzz_count > MAXIMUM_FUZZ_COUNT
        or not 1.0 <= arguments.timeout_sec <= MAXIMUM_TIMEOUT_SEC
    ):
        parser.error('fuzz-count must be 1..4096 and timeout-sec must be 1..600')
    selected = arguments.case or names
    if any(name not in names for name in selected):
        parser.error('Unknown case; use --list for exact names')
    directory = arguments.artifact_dir.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    manifest = {
        'suite_sha256': {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(SUITE_DIRECTORY.iterdir())
            if path.suffix in {'.py', '.toml'}
        },
        'seed': arguments.seed,
        'fuzz_count': arguments.fuzz_count,
        'cases': selected,
        'timeout_sec': arguments.timeout_sec,
        'memory_limit_bytes': MEMORY_LIMIT_BYTES,
        'thread_limit': THREAD_LIMIT,
        'log_limit_bytes': LOG_LIMIT_BYTES,
        'source_access': (
            'README files, CONTRIBUTING.md, repository skills, public ROS schemas '
            'and installed executable metadata'
        ),
    }
    (directory / 'manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8'
    )
    results: list[dict[str, str | int | float]] = []
    for index, name in enumerate(selected):
        print(f'[{index + 1}/{len(selected)}] {name}', flush=True)
        result = run_case(
            name,
            directory / f'{index:03d}-{name}',
            arguments.seed,
            arguments.fuzz_count,
            arguments.timeout_sec,
        )
        results.append(result)
        print(
            f'  {result["status"]}: {result["duration_sec"]} sec {result["failure"]}',
            flush=True,
        )
        (directory / 'results.json').write_text(
            json.dumps(results, indent=2) + '\n', encoding='utf-8'
        )
    failures = sum(result['status'] == 'failed' for result in results)
    suite = ET.Element(
        'testsuite',
        name='blind_node_torture',
        tests=str(len(results)),
        failures=str(failures),
    )
    for result in results:
        case = ET.SubElement(
            suite,
            'testcase',
            name=str(result['case']),
            time=str(result['duration_sec']),
        )
        if result['status'] == 'failed':
            ET.SubElement(case, 'failure', message=str(result['failure'])).text = str(
                result['directory']
            )
    ET.ElementTree(suite).write(directory / 'junit.xml', encoding='unicode', xml_declaration=True)
    print(f'{len(results) - failures} passed, {failures} failed. Artifacts: {directory}')
    return int(failures > 0)


if __name__ == '__main__':
    raise SystemExit(main())
