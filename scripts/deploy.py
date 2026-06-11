#!/usr/bin/env python3
"""deploy.py — Build, DFU-flash, and recover a MOTION sensor module.

Flashes via STM32_Programmer_CLI (STM32CubeProgrammer) when installed —
~10x faster than dfu-util against the ROM bootloader — and falls back to
dfu-util otherwise.

Usage:
    python scripts/deploy.py --device left|right
                             [--config Debug|Release] [--no-build]
                             [--fw-only] [--use-dfu-util]
                             [--programmer-cli PATH] [--dfu-util PATH]
                             [--power-cycle-cmd CMD]
                             [--post-reset] [--no-confirm]
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from _deploy_helpers import (  # noqa: E402
    bin_path_for,
    read_project_name,
    resolve_dfu_util,
    resolve_programmer_cli,
    wait_for_dfu_device,
    wait_for_dfu_device_cube,
    DFU_VID_PID_STR,
)

# Boot includes FPGA SRAM erase (~5 s), so give the comeback some headroom.
COMEBACK_TIMEOUT_S = 15.0
DFU_ENUM_TIMEOUT_S = 10.0
FLASH_BASE = 0x08000000


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Build, enter DFU, flash, and reset a MOTION sensor module."
    )
    p.add_argument("--device", choices=("left", "right"), required=True,
                   help="Which sensor side to flash (required to avoid wrong-side mistakes)")
    p.add_argument("--config", choices=("Debug", "Release"), default="Debug")
    p.add_argument("--no-build", action="store_true")
    p.add_argument("--fw-only", action="store_true",
                   help="Flash the firmware-only image (no FPGA bitstream) — "
                        "much faster; use when the bitstream hasn't changed")
    p.add_argument("--use-dfu-util", action="store_true",
                   help="Force dfu-util even if STM32_Programmer_CLI is installed")
    p.add_argument("--programmer-cli", default=None,
                   help="Path to STM32_Programmer_CLI (default: PATH, then "
                        "standard install locations)")
    p.add_argument("--dfu-util", default=None)
    p.add_argument("--post-reset", action="store_true",
                   help="If sensor doesn't re-enumerate after leaveDFU, print extra hints")
    p.add_argument("--power-cycle-cmd", default=None,
                   help="Shell command that power-cycles the sensor (e.g. a "
                        "smart-plug script). Run automatically if the sensor "
                        "doesn't re-enumerate after flashing — the ROM "
                        "bootloader's jump-to-app is unreliable, so on a "
                        "bench this makes deploys hands-free.")
    p.add_argument("--no-confirm", action="store_true")
    return p.parse_args()


def _run(cmd: list[str], **kw) -> int:
    print(f"+ {' '.join(cmd)}")
    return subprocess.call(cmd, **kw)


def _git_describe() -> str:
    try:
        out = subprocess.check_output(
            ["git", "describe", "--tags", "--dirty", "--always"],
            cwd=REPO_ROOT, text=True,
        )
        return out.strip()
    except Exception:
        return "unknown"


def _confirm(device: str, bin_file: Path) -> bool:
    print()
    print(f"  Device : {device} sensor")
    print(f"  Binary : {bin_file}  ({bin_file.stat().st_size} bytes)")
    print(f"  Source : {_git_describe()}")
    print()
    answer = input(f"Deploy {device} sensor? (y/N): ").strip().lower()
    return answer == "y"


def _build(config: str) -> None:
    build_dir = REPO_ROOT / "build" / config
    if not build_dir.exists():
        raise RuntimeError(
            f"Build dir {build_dir} does not exist. Run cmake configure first."
        )
    rc = _run(["cmake", "--build", str(build_dir), "--config", config])
    if rc != 0:
        raise RuntimeError(f"cmake --build failed with exit code {rc}")


def _sensor_handle(interface, device: str):
    return interface.left if device == "left" else interface.right


def _enter_dfu_sensor(device: str, timeout: float) -> bool:
    """Trigger the named sensor into DFU mode via omotion."""
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=False)
    try:
        sensor = _sensor_handle(interface, device)
        # start(wait=True) only blocks on already-CONNECTING handles, so it
        # races against the connection monitor's first sweep. Poll the
        # specific handle until CONNECTED or timeout.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not sensor.is_connected():
            time.sleep(0.2)
        if not sensor.is_connected():
            print(f"❌ {device} sensor not connected — cannot trigger DFU.")
            return False

        print(f"[*] Requesting DFU mode on {device} sensor …")
        try:
            return bool(sensor.enter_dfu())
        except Exception as exc:
            print(f"❌ enter_dfu raised: {exc}")
            return False
    finally:
        interface.stop()


def _wait_for_sensor_comeback(device: str, timeout: float) -> bool:
    """Construct the interface ONCE and poll the sensor handle."""
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=False)
    try:
        sensor = _sensor_handle(interface, device)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if sensor.is_connected():
                return True
            time.sleep(0.5)
        return False
    finally:
        interface.stop()


def _force_utf8_stdout() -> None:
    """Windows default cp1252 stdout chokes on the emoji glyphs printed
    below. Reconfigure to UTF-8 with replace-on-error so a missing-glyph
    terminal degrades gracefully instead of crashing the script."""
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass


def main() -> int:
    _force_utf8_stdout()
    args = parse_args()

    try:
        project = read_project_name(REPO_ROOT / "CMakeLists.txt")
        bin_file = bin_path_for(REPO_ROOT, args.config, project,
                                fw_only=args.fw_only)
        cube_cli = None
        if not args.use_dfu_util:
            cube_cli = resolve_programmer_cli(args.programmer_cli)
        # dfu-util is only required when CubeProgrammer isn't available.
        dfu_util = None if cube_cli else resolve_dfu_util(args.dfu_util)
    except RuntimeError as e:
        print(f"❌ {e}")
        return 1

    try:
        import omotion  # noqa: F401
    except ImportError:
        print("❌ omotion not installed in this Python env. "
              "Run `pip install -e ../openmotion-sdk` (path may differ).")
        return 1

    if not args.no_build:
        try:
            _build(args.config)
        except RuntimeError as e:
            print(f"❌ {e}")
            return 1

    if not bin_file.exists():
        print(f"❌ Binary not found: {bin_file}")
        return 1

    if not args.no_confirm:
        if not _confirm(args.device, bin_file):
            print("Aborted.")
            return 0

    if not _enter_dfu_sensor(args.device, timeout=DFU_ENUM_TIMEOUT_S):
        if not args.power_cycle_cmd:
            return 1
        # Sensor may be hung (e.g. from an earlier failed flash) — recover it.
        print(f"[*] {args.device} sensor unreachable; power-cycling …")
        print(f"+ {args.power_cycle_cmd}")
        subprocess.call(args.power_cycle_cmd, shell=True)
        if not _enter_dfu_sensor(args.device, timeout=DFU_ENUM_TIMEOUT_S + 10):
            return 1

    print(f"[*] Waiting for DFU device ({DFU_VID_PID_STR}) to enumerate …")
    if cube_cli:
        found = wait_for_dfu_device_cube(cube_cli, timeout=DFU_ENUM_TIMEOUT_S)
    else:
        found = wait_for_dfu_device(dfu_util, timeout=DFU_ENUM_TIMEOUT_S)
    if not found:
        print("❌ DFU device did not appear within "
              f"{DFU_ENUM_TIMEOUT_S:.0f}s.")
        print("   On Windows, bind WinUSB to the STM32 DFU interface "
              "(PID 0xDF11) via Zadig.")
        return 1

    if cube_cli:
        print(f"[*] Flashing {bin_file.name} via STM32_Programmer_CLI …")
        cmd = [
            cube_cli,
            "-c", "port=usb1",
            "-w", str(bin_file), f"0x{FLASH_BASE:08x}",
            "-g", f"0x{FLASH_BASE:08x}",
        ]
    else:
        print(f"[*] Flashing {bin_file.name} via dfu-util …")
        cmd = [
            dfu_util,
            "-d", DFU_VID_PID_STR,
            "-a", "0",
            "-s", f"0x{FLASH_BASE:08x}:leave",
            "-D", str(bin_file),
        ]
    rc = _run(cmd)
    # dfu-util exit 74 ("Error during download get_status") after a successful
    # download is a known STM32 ROM bootloader quirk: the device jumps to user
    # firmware on :leave before dfu-util can read its final status. Trust the
    # device's comeback as the source of truth rather than the exit code.
    if rc != 0:
        print(f"[!] flasher exited {rc}; checking whether {args.device} sensor came back …")

    print(f"[*] Flash done. Waiting for {args.device} sensor to come back …")
    if _wait_for_sensor_comeback(args.device, timeout=COMEBACK_TIMEOUT_S):
        print(f"✅ {args.device.capitalize()} sensor is back online.")
        return 0

    if args.power_cycle_cmd:
        print(f"⚠️  {args.device} sensor did not re-enumerate; power-cycling …")
        print(f"+ {args.power_cycle_cmd}")
        subprocess.call(args.power_cycle_cmd, shell=True)
        if _wait_for_sensor_comeback(args.device,
                                     timeout=COMEBACK_TIMEOUT_S + 10):
            print(f"✅ {args.device.capitalize()} sensor is back online "
                  "(after power cycle).")
            return 0
        print(f"❌ {args.device} sensor still offline after power cycle.")
        return 1

    # No software path to power-cycle a single sensor — surface a clear hint.
    print(f"⚠️  {args.device} sensor did not re-enumerate.")
    print("   Please toggle power on the console "
          "(this power-cycles the sensors).")
    if args.post_reset:
        print("   --post-reset: no additional automatic recovery is available "
              "for sensors; manual power cycle required.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
