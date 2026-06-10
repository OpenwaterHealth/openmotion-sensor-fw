import shutil
import subprocess
import textwrap
from pathlib import Path
from unittest.mock import patch

import pytest

from _deploy_helpers import (
    read_project_name,
    bin_path_for,
    resolve_dfu_util,
    resolve_programmer_cli,
    wait_for_dfu_device,
    wait_for_dfu_device_cube,
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


def test_bin_path_for_fw_only_uses_raw_bin(tmp_path: Path):
    repo = tmp_path
    result = bin_path_for(repo, "Debug", "motion-sensor-fw", fw_only=True)
    assert result == repo / "build" / "Debug" / "motion-sensor-fw-raw.bin"


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


def test_resolve_programmer_cli_uses_override_when_given(tmp_path: Path):
    fake = tmp_path / "STM32_Programmer_CLI.exe"
    fake.write_text("")
    assert resolve_programmer_cli(str(fake)) == str(fake)


def test_resolve_programmer_cli_raises_when_override_does_not_exist(tmp_path: Path):
    nonexistent = tmp_path / "nope" / "STM32_Programmer_CLI.exe"
    with pytest.raises(RuntimeError, match="--programmer-cli path"):
        resolve_programmer_cli(str(nonexistent))


def test_resolve_programmer_cli_prefers_path_lookup(monkeypatch):
    monkeypatch.setattr(shutil, "which",
                        lambda _: "C:/tools/STM32_Programmer_CLI.exe")
    assert resolve_programmer_cli(None) == "C:/tools/STM32_Programmer_CLI.exe"


def test_resolve_programmer_cli_returns_none_when_absent(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda _: None)
    monkeypatch.setattr("_deploy_helpers._CUBE_CLI_DEFAULT_PATHS", ())
    assert resolve_programmer_cli(None) is None


def test_resolve_programmer_cli_falls_back_to_default_paths(tmp_path: Path,
                                                            monkeypatch):
    installed = tmp_path / "STM32_Programmer_CLI.exe"
    installed.write_text("")
    monkeypatch.setattr(shutil, "which", lambda _: None)
    monkeypatch.setattr("_deploy_helpers._CUBE_CLI_DEFAULT_PATHS", (installed,))
    assert resolve_programmer_cli(None) == str(installed)


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


_CUBE_LIST_FOUND = """\
=====  DFU Interface   =====

Total number of available STM32 device in DFU mode: 1

  Device Index           : USB1
  Product ID             : DFU in FS Mode
"""


def test_wait_for_dfu_device_cube_succeeds_when_listed():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=_fake_run_returning(_CUBE_LIST_FOUND)):
        assert wait_for_dfu_device_cube("cli", timeout=0.5,
                                        poll_interval=0.05) is True


def test_wait_for_dfu_device_cube_times_out_when_absent():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=_fake_run_returning(
                   "Total number of available STM32 device in DFU mode: 0\n")):
        assert wait_for_dfu_device_cube("cli", timeout=0.2,
                                        poll_interval=0.05) is False


def test_wait_for_dfu_device_cube_returns_false_when_cli_missing():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=FileNotFoundError("cli gone")):
        assert wait_for_dfu_device_cube("cli", timeout=0.5,
                                        poll_interval=0.05) is False
