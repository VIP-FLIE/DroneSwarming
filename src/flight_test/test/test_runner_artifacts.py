# Copyright 2026 DroneSwarming maintainers.
"""Check standalone flight artifact directory creation."""

import fcntl
import json
import stat
from pathlib import Path

import pytest
from flight_test.runner import main, resolve_artifact_directory


def test_flights_in_one_run_get_distinct_artifact_directories(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Keep repeated flights separate within the active simulation run."""
    monkeypatch.setenv('DRONE_RUN_DIR', str(tmp_path))

    first_directory: Path = resolve_artifact_directory('FT-00', None)
    second_directory: Path = resolve_artifact_directory('FT-00', None)

    assert first_directory != second_directory
    assert first_directory.parent == tmp_path / 'flight-tests'
    assert second_directory.parent == first_directory.parent
    assert first_directory.is_dir()
    assert second_directory.is_dir()
    assert stat.S_IMODE(first_directory.stat().st_mode) == 0o755
    assert stat.S_IMODE(second_directory.stat().st_mode) == 0o755


def test_flight_requires_a_run_directory_when_no_destination_is_given(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Reject a flight with no active log destination."""
    monkeypatch.delenv('DRONE_RUN_DIR', raising=False)

    with pytest.raises(ValueError, match='DRONE_RUN_DIR is required'):
        resolve_artifact_directory('FT-01', None)


def test_explicit_artifact_directory_cannot_replace_an_existing_result(
    tmp_path: Path,
) -> None:
    """Preserve an existing case result when the destination is repeated."""
    artifact_directory: Path = tmp_path / 'FT-02'
    artifact_directory.mkdir()
    existing_result: Path = artifact_directory / 'case-result.json'
    existing_result.write_text('existing result', encoding='utf-8')

    with pytest.raises(FileExistsError):
        resolve_artifact_directory('FT-02', artifact_directory)

    assert existing_result.read_text(encoding='utf-8') == 'existing result'


def test_explicit_artifact_directory_is_readable_by_host_user(tmp_path: Path) -> None:
    """Allow host inspection of case reports created by the container user."""
    artifact_directory: Path = resolve_artifact_directory('FT-02', tmp_path / 'FT-02')

    assert stat.S_IMODE(artifact_directory.stat().st_mode) == 0o755


def test_concurrent_flight_is_rejected(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Prevent two standalone cases from commanding the same application."""
    run_directory: Path = tmp_path / 'run'
    lock_directory: Path = run_directory / 'flight-tests'
    lock_directory.mkdir(parents=True)
    monkeypatch.setenv('DRONE_RUN_DIR', str(run_directory))
    artifact_directory: Path = tmp_path / 'results' / 'FT-03'
    with (lock_directory / '.flight-case.lock').open(
        'a', encoding='utf-8'
    ) as lock_stream:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

        status: int = main(
            ['--case', 'FT-03', '--artifact-dir', str(artifact_directory)]
        )

    result: dict[str, object] = json.loads(
        (artifact_directory / 'case-result.json').read_text(encoding='utf-8')
    )
    assert status == 1
    assert 'another flight case is already running' in str(result['failure'])
