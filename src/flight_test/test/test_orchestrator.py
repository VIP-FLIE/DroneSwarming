# Copyright 2026 DroneSwarming maintainers.
"""Test per-run routing and preservation of PX4 logs."""

from pathlib import Path

import pytest
from flight_test.orchestrator import FlightTestError, prepare_px4_log_directory

from flight_test import orchestrator


def configure_log_paths(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> tuple[Path, Path]:
    """Replace container paths with temporary test paths."""
    px4_rootfs: Path = tmp_path / 'px4-rootfs'
    run_log_root: Path = tmp_path / 'run-logs'
    px4_rootfs.mkdir()
    run_log_root.mkdir()
    monkeypatch.setattr(orchestrator, 'PX4_ROOTFS_DIRECTORY', px4_rootfs)
    monkeypatch.setattr(orchestrator, 'RUN_LOG_ROOT', run_log_root)
    return px4_rootfs, run_log_root


def test_prepare_log_directory_archives_existing_logs_and_routes_both_paths(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Preserve existing ULogs before routing both PX4 paths to the current run."""
    px4_rootfs, run_log_root = configure_log_paths(monkeypatch, tmp_path)
    run_directory: Path = run_log_root / 'current-run'
    run_directory.mkdir()
    rootfs_log: Path = px4_rootfs / 'log'
    rootfs_log_file: Path = rootfs_log / '2026-11-04' / 'flight.ulg'
    rootfs_log_file.parent.mkdir(parents=True)
    rootfs_log_file.write_bytes(b'rootfs log')
    microsd_log: Path = px4_rootfs / 'fs/microsd/log'
    microsd_log_file: Path = microsd_log / 'previous.ulg'
    microsd_log_file.parent.mkdir(parents=True)
    microsd_log_file.write_bytes(b'microsd log')

    prepare_px4_log_directory(run_directory)

    run_log_directory: Path = run_directory / 'px4/log'
    assert rootfs_log.is_symlink()
    assert microsd_log.is_symlink()
    assert rootfs_log.resolve() == run_log_directory.resolve()
    assert microsd_log.resolve() == run_log_directory.resolve()
    assert (
        run_directory / 'px4/preexisting-rootfs-log/2026-11-04/flight.ulg'
    ).read_bytes() == b'rootfs log'
    assert (run_directory / 'px4/preexisting-fs-microsd-log/previous.ulg').read_bytes() == (
        b'microsd log'
    )


def test_prepare_log_directory_repoints_only_prior_run_symlinks(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Repoint an archived run link while preserving its existing ULog file."""
    px4_rootfs, run_log_root = configure_log_paths(monkeypatch, tmp_path)
    run_directory: Path = run_log_root / 'current-run'
    run_directory.mkdir()
    previous_log_directory: Path = run_log_root / 'previous-run/px4/log'
    previous_log_directory.mkdir(parents=True)
    previous_log_file: Path = previous_log_directory / 'previous.ulg'
    previous_log_file.write_bytes(b'previous run')
    (px4_rootfs / 'log').symlink_to(previous_log_directory, target_is_directory=True)

    prepare_px4_log_directory(run_directory)

    assert (px4_rootfs / 'log').resolve() == (run_directory / 'px4/log').resolve()
    assert previous_log_file.read_bytes() == b'previous run'


def test_prepare_log_directory_rejects_unrelated_symlink(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Reject PX4 log links that point outside the run archive."""
    px4_rootfs, run_log_root = configure_log_paths(monkeypatch, tmp_path)
    run_directory: Path = run_log_root / 'current-run'
    run_directory.mkdir()
    unrelated_directory: Path = tmp_path / 'unrelated'
    unrelated_directory.mkdir()
    unrelated_file: Path = unrelated_directory / 'keep.ulg'
    unrelated_file.write_bytes(b'keep')
    (px4_rootfs / 'log').symlink_to(unrelated_directory, target_is_directory=True)

    with pytest.raises(FlightTestError, match='points outside the run archive'):
        prepare_px4_log_directory(run_directory)

    assert unrelated_file.read_bytes() == b'keep'
