import shutil
import subprocess
import textwrap
from pathlib import Path
from unittest.mock import patch

import pytest

from _deploy_helpers import (
    read_project_name,
    bin_path_for,
    build_type_for,
    read_boot_mode,
    resolve_dfu_util,
    wait_for_dfu_device,
)


def test_read_project_name_extracts_from_set_directive(tmp_path: Path):
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(textwrap.dedent("""
        cmake_minimum_required(VERSION 3.22)
        set(CMAKE_PROJECT_NAME motion-console-fw)
        project(${CMAKE_PROJECT_NAME})
    """))
    assert read_project_name(cmake) == "motion-console-fw"


def test_read_project_name_raises_when_missing(tmp_path: Path):
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text("# nothing here\n")
    with pytest.raises(RuntimeError, match="CMAKE_PROJECT_NAME"):
        read_project_name(cmake)


def test_bin_path_for_returns_repo_relative(tmp_path: Path):
    repo = tmp_path
    result = bin_path_for(repo, "Debug", "motion-console-fw")
    assert result == repo / "build" / "Debug" / "motion-console-fw.bin"


def _write_cache(build_dir: Path, body: str) -> Path:
    build_dir.mkdir(parents=True, exist_ok=True)
    cache = build_dir / "CMakeCache.txt"
    cache.write_text(textwrap.dedent(body), encoding="utf-8")
    return cache


def test_read_boot_mode_detects_baremetal(tmp_path: Path):
    _write_cache(tmp_path / "Debug-BareMetal", """
        CMAKE_BUILD_TYPE:STRING=Debug
        BARE_METAL:BOOL=ON
    """)
    assert read_boot_mode(tmp_path / "Debug-BareMetal") == "baremetal"


def test_read_boot_mode_detects_slot(tmp_path: Path):
    _write_cache(tmp_path / "Debug", """
        CMAKE_BUILD_TYPE:STRING=Debug
        BARE_METAL:BOOL=OFF
    """)
    assert read_boot_mode(tmp_path / "Debug") == "slot"


def test_read_boot_mode_raises_when_cache_missing(tmp_path: Path):
    with pytest.raises(RuntimeError, match="CMakeCache.txt"):
        read_boot_mode(tmp_path / "never-configured")


def test_read_boot_mode_raises_on_stale_cache_without_the_option(tmp_path: Path):
    """A cache predating the BARE_METAL option says nothing about layout.

    Guessing here is how you flash a slot image to 0x08000000, so refuse and
    make the caller reconfigure.
    """
    _write_cache(tmp_path / "Debug", """
        CMAKE_BUILD_TYPE:STRING=Debug
    """)
    with pytest.raises(RuntimeError, match="BARE_METAL"):
        read_boot_mode(tmp_path / "Debug")


@pytest.mark.parametrize("config,expected", [
    ("Debug", "Debug"),
    ("Release", "Release"),
    ("Debug-BareMetal", "Debug"),
    ("Release-BareMetal", "Release"),
])
def test_build_type_for_strips_the_preset_suffix(config: str, expected: str):
    assert build_type_for(config) == expected


def test_resolve_dfu_util_uses_override_when_given(tmp_path: Path):
    fake = tmp_path / "dfu-util"
    fake.write_text("#!/bin/sh\necho dfu-util fake\n")
    fake.chmod(0o755)
    assert resolve_dfu_util(str(fake)) == str(fake)


def test_resolve_dfu_util_raises_when_missing(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda _: None)
    monkeypatch.setattr("_deploy_helpers._bundled_dfu_util", lambda: None)
    with pytest.raises(RuntimeError, match="dfu-util not found"):
        resolve_dfu_util(None)


def test_resolve_dfu_util_raises_when_override_does_not_exist(tmp_path: Path):
    nonexistent = tmp_path / "does-not-exist" / "dfu-util"
    with pytest.raises(RuntimeError, match="dfu-util override path"):
        resolve_dfu_util(str(nonexistent))


def test_resolve_dfu_util_prefers_bundled_over_path(tmp_path: Path, monkeypatch):
    bundled = tmp_path / "bundled" / "dfu-util.exe"
    bundled.parent.mkdir(parents=True)
    bundled.write_text("")
    monkeypatch.setattr("_deploy_helpers._bundled_dfu_util", lambda: bundled)
    monkeypatch.setattr(shutil, "which", lambda _: "C:/path/dfu-util.exe")
    assert resolve_dfu_util(None) == str(bundled)


def _fake_run_returning(stdout: str, returncode: int = 0):
    def _run(cmd, **kw):
        return subprocess.CompletedProcess(cmd, returncode, stdout=stdout, stderr="")
    return _run


def test_wait_for_dfu_device_succeeds_when_listed():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=_fake_run_returning("Found DFU: [0483:df11] ...\n")):
        assert wait_for_dfu_device("dfu-util", timeout=0.5, poll_interval=0.05) is True


def test_wait_for_dfu_device_times_out_when_absent():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=_fake_run_returning("No DFU capable USB device available\n")):
        assert wait_for_dfu_device("dfu-util", timeout=0.2, poll_interval=0.05) is False


def test_wait_for_dfu_device_returns_false_when_dfu_util_missing():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=FileNotFoundError("dfu-util gone")):
        assert wait_for_dfu_device("dfu-util", timeout=0.5,
                                   poll_interval=0.05) is False
