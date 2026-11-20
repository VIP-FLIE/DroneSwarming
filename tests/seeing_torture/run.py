# Copyright 2026 DroneSwarming maintainers.
"""Supervise each adversarial case in a process group and retain machine-readable findings."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from harness import verify_isolation

if TYPE_CHECKING:
    from harness import JsonValue

CASE_TIMEOUT_SEC = 35.0
MAXIMUM_REPETITIONS = 100
TEST_MODULES = (
    'test_vehicle_faults',
    'test_executor_faults',
    'test_planner_faults',
    'test_pipeline_faults',
)


def test_names() -> list[str]:
    """Discover only this suite's explicitly named modules."""
    names: list[str] = []
    for module in TEST_MODULES:
        suite = unittest.defaultTestLoader.loadTestsFromName(module)
        for group in suite:
            if isinstance(group, unittest.TestSuite):
                names.extend(case.id() for case in group)
            else:
                raise TypeError(str(group))
    return names


def run_worker(name: str, directory: Path) -> int:
    """Execute one case with ordinary failures preserved as failing assertions."""
    os.environ['SEEING_CASE_DIRECTORY'] = str(directory)
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros')
    suite = unittest.defaultTestLoader.loadTestsFromName(name)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    outcome: dict[str, JsonValue] = {
        'name': name,
        'tests_run': result.testsRun,
        'failures': [trace for _, trace in result.failures],
        'errors': [trace for _, trace in result.errors],
        'skipped': [reason for _, reason in result.skipped],
    }
    (directory / 'result.json').write_text(json.dumps(outcome, indent=2) + '\n')
    if result.errors or result.skipped or result.testsRun != 1:
        return 2
    return 0 if result.wasSuccessful() else 1


def supervise(name: str, directory: Path) -> dict[str, JsonValue]:
    """Kill the entire case group on timeout and after normal completion."""
    directory.mkdir(parents=True)
    started_sec: float = time.monotonic()
    with (directory / 'console.log').open('wb') as log:
        process = subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                '--worker',
                name,
                '--output',
                str(directory),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        timed_out = False
        try:
            exit_code: int = process.wait(timeout=CASE_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            timed_out = True
            exit_code = 124
        finally:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
    result_path: Path = directory / 'result.json'
    details: dict[str, JsonValue] = (
        json.loads(result_path.read_text()) if result_path.exists() else {}
    )
    status: str = (
        'TIMEOUT'
        if timed_out
        else 'PASS'
        if exit_code == 0
        else 'FAIL'
        if exit_code == 1
        else 'ERROR'
    )
    return {
        'name': name,
        'status': status,
        'exit_code': exit_code,
        'duration_sec': round(time.monotonic() - started_sec, 3),
        'artifacts': directory.name,
        **details,
    }


def source_manifest() -> dict[str, str]:
    """Record hashes of the production files used by the isolated workspace build."""
    root = Path('/tmp/seeing-workspace/src')
    suffixes: set[str] = {'.cpp', '.hpp', '.py', '.msg', '.srv', '.action'}
    return {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob('*'))
        if path.is_file() and path.suffix in suffixes
    }


def main() -> int:
    """Run filtered cases, write JSON and Markdown, and fail on every unresolved finding."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', default='', help='Run case names containing this substring')
    parser.add_argument('--repeat', type=int, default=1, help='Repeat selected cases, at most 100')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--worker', help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    verify_isolation()
    if arguments.worker:
        return run_worker(arguments.worker, arguments.output)
    names: list[str] = [name for name in test_names() if arguments.case in name]
    if not names:
        parser.error('No seeing torture cases matched --case')
    if not 1 <= arguments.repeat <= MAXIMUM_REPETITIONS:
        parser.error('--repeat must be between 1 and 100')
    if arguments.list:
        print('\n'.join(names))
        return 0
    arguments.output.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, JsonValue]] = []
    report: dict[str, JsonValue] = {
        'suite': 'seeing_torture',
        'mutation_seed': 20261119,
        'source_sha256': source_manifest(),
        'suite_sha256': {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(Path(__file__).parent.glob('*.py'))
        },
        'results': results,
    }
    for repetition in range(1, arguments.repeat + 1):
        for index, name in enumerate(names, start=1):
            directory: Path = arguments.output / f'{repetition:03d}-{index:03d}-{name}'
            result: dict[str, JsonValue] = supervise(name, directory)
            results.append(result)
            print(f'{result["status"]}: {name} ({result["duration_sec"]}s)', flush=True)
            (arguments.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    counts: dict[str, int] = {
        status: sum(result['status'] == status for result in results)
        for status in ('PASS', 'FAIL', 'ERROR', 'TIMEOUT')
    }
    report['counts'] = counts
    (arguments.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    lines: list[str] = [
        '# Seeing torture results',
        '',
        f'Counts: {json.dumps(counts, sort_keys=True)}',
        '',
        '| Case | Status | Seconds |',
        '| --- | --- | --- |',
        *[
            f'| [{result["name"]}](./{result["artifacts"]}/console.log) '
            f'| {result["status"]} | {result["duration_sec"]} |'
            for result in results
        ],
        '',
    ]
    (arguments.output / 'report.md').write_text('\n'.join(lines))
    print(json.dumps(counts, sort_keys=True))
    if counts['ERROR']:
        return 2
    return 1 if counts['FAIL'] or counts['TIMEOUT'] else 0


if __name__ == '__main__':
    sys.exit(main())
