"""Pure helper functions for scripts/deploy.py."""
from __future__ import annotations

import platform
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Optional

_PROJECT_NAME_RE = re.compile(
    r"set\s*\(\s*CMAKE_PROJECT_NAME\s+([^\s\)]+)\s*\)"
)


def read_project_name(cmake_path: Path) -> str:
    """Extract CMAKE_PROJECT_NAME from a CMakeLists.txt.

    Mirrors the regex used by generate_vscode_files.py so VS Code and the
    deploy script always agree on the binary name.
    """
    text = cmake_path.read_text(encoding="utf-8")
    m = _PROJECT_NAME_RE.search(text)
    if not m:
        raise RuntimeError(
            f"Could not find CMAKE_PROJECT_NAME set(...) directive in {cmake_path}"
        )
    return m.group(1)


_BARE_METAL_CACHE_RE = re.compile(
    r"^BARE_METAL:BOOL=(\w+)\s*$", re.MULTILINE
)

# CMake preset suffix marking the bare-metal variants (see CMakePresets.json).
_BARE_METAL_SUFFIX = "-BareMetal"


def build_type_for(config: str) -> str:
    """Return the CMAKE_BUILD_TYPE for a preset name.

    ``Debug-BareMetal`` is a *preset* name, not a build type — the build type
    is still ``Debug``. Mirrors what .github/workflows/build-firmware.yml
    passes to ``cmake --build --config``.
    """
    if config.endswith(_BARE_METAL_SUFFIX):
        return config[: -len(_BARE_METAL_SUFFIX)]
    return config


def read_boot_mode(build_dir: Path) -> str:
    """Return "baremetal" or "slot" for an already-configured build dir.

    The CMakeCache is the only honest source: BARE_METAL can be set by the
    preset *or* by a bare ``-DBARE_METAL=ON``, and the preset name alone
    doesn't prove which linker script was used.

    Raises RuntimeError if the directory was never configured, or if the
    cache predates the BARE_METAL option — in that case the layout is
    genuinely unknown, and guessing is how a slot image ends up flashed to
    0x08000000.
    """
    cache = build_dir / "CMakeCache.txt"
    if not cache.is_file():
        raise RuntimeError(
            f"No CMakeCache.txt in {build_dir} — that build directory has "
            f"never been configured. Run: cmake --preset {build_dir.name}"
        )

    m = _BARE_METAL_CACHE_RE.search(cache.read_text(encoding="utf-8",
                                                    errors="replace"))
    if not m:
        raise RuntimeError(
            f"{cache} has no BARE_METAL entry, so it predates the boot-mode "
            "split and its flash layout cannot be determined. Reconfigure "
            f"from scratch: cmake --preset {build_dir.name}"
        )

    return "baremetal" if m.group(1).upper() in ("ON", "TRUE", "1", "YES") else "slot"


def bin_path_for(repo_root: Path, config: str, project: str,
                 fw_only: bool = False) -> Path:
    """Return the absolute path to the produced .bin for a given config.

    With fw_only=True, return the pre-merge firmware-only image
    (``{project}-raw.bin``) — no FPGA bitstream, so flashing it leaves the
    bitstream sectors untouched and is much faster.
    """
    name = f"{project}-raw.bin" if fw_only else f"{project}.bin"
    return repo_root / "build" / config / name


def _bundled_dfu_util() -> Optional[Path]:
    """Look for the dfu-util binary that ships with the omotion package.

    Returns None if omotion isn't importable or no matching platform binary
    is bundled. The SDK ships binaries for win32, win64, and darwin-x86_64;
    Linux callers fall through to PATH lookup.
    """
    try:
        import omotion
    except ImportError:
        return None

    pkg_dir = Path(omotion.__file__).resolve().parent

    system = platform.system().lower()
    if system.startswith("windows"):
        machine = platform.machine().lower()
        subdir = "win64" if "64" in machine else "win32"
        exe = "dfu-util.exe"
    elif system.startswith("darwin"):
        subdir = "darwin-x86_64"
        exe = "dfu-util"
    else:
        return None

    candidate = pkg_dir / "dfu-util" / subdir / exe
    return candidate if candidate.is_file() else None


def resolve_dfu_util(override: str | None) -> str:
    """Return an absolute path to dfu-util, or raise if not found.

    Search order:
      1. --dfu-util override (must exist if given)
      2. The binary bundled inside the installed omotion package
      3. dfu-util on PATH
    """
    if override:
        if not Path(override).is_file():
            raise RuntimeError(
                f"dfu-util override path does not exist: {override}"
            )
        return override

    bundled = _bundled_dfu_util()
    if bundled is not None:
        return str(bundled)

    found = shutil.which("dfu-util")
    if not found:
        raise RuntimeError(
            "dfu-util not found: not bundled with the installed omotion "
            "package, not on PATH, and no --dfu-util override given. "
            "Install omotion (pip install -e ../openmotion-sdk), install "
            "dfu-util on PATH (Windows: scoop install dfu-util), or pass "
            "--dfu-util PATH."
        )
    return found


# Default install locations for STM32CubeProgrammer's CLI, checked when it
# isn't on PATH. The CLI flashes the same ROM DFU bootloader ~10x faster than
# dfu-util because it doesn't sleep through every bwPollTimeout the bootloader
# reports between 2 KB chunks.
_CUBE_CLI_DEFAULT_PATHS = (
    Path("C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"),
    Path("C:/Program Files (x86)/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"),
    Path("/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
    Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"),
)


def resolve_programmer_cli(override: str | None) -> Optional[str]:
    """Return a path to STM32_Programmer_CLI, or None if not installed.

    Search order:
      1. --programmer-cli override (must exist if given)
      2. STM32_Programmer_CLI on PATH
      3. Known default install locations

    Unlike resolve_dfu_util, a missing CLI is not an error — the caller
    falls back to dfu-util.
    """
    if override:
        if not Path(override).is_file():
            raise RuntimeError(
                f"--programmer-cli path does not exist: {override}"
            )
        return override

    found = shutil.which("STM32_Programmer_CLI")
    if found:
        return found

    for candidate in _CUBE_CLI_DEFAULT_PATHS:
        if candidate.is_file():
            return str(candidate)
    return None


def wait_for_dfu_device_cube(cli: str, timeout: float = 10.0,
                             poll_interval: float = 0.3) -> bool:
    """Poll `STM32_Programmer_CLI -l usb` until an STM32 DFU device appears.

    Returns True if a device was found, False on timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                [cli, "-l", "usb"],
                capture_output=True, text=True, check=False,
            )
            if "Device Index" in result.stdout:
                return True
        except FileNotFoundError:
            return False
        time.sleep(poll_interval)
    return False


DFU_VID_PID_STR = "0483:df11"


def wait_for_dfu_device(dfu_util: str, timeout: float = 10.0,
                        poll_interval: float = 0.3) -> bool:
    """Poll `dfu-util --list` until the STM32 DFU device appears or timeout.

    Returns True if the device was found, False on timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                [dfu_util, "--list"],
                capture_output=True, text=True, check=False,
            )
            if DFU_VID_PID_STR in result.stdout.lower():
                return True
        except FileNotFoundError:
            # dfu-util went missing between resolve and now — fail fast
            return False
        time.sleep(poll_interval)
    return False
