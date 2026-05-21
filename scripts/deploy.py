#!/usr/bin/env python3
"""deploy.py — Build, DFU-flash, and recover a MOTION sensor module.

Usage:
    python scripts/deploy.py --device left|right
                             [--config Debug|Release] [--no-build]
                             [--dfu-util PATH] [--post-reset] [--no-confirm]
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
    wait_for_dfu_device,
    DFU_VID_PID_STR,
)

COMEBACK_TIMEOUT_S = 10.0
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
    p.add_argument("--dfu-util", default=None)
    p.add_argument("--post-reset", action="store_true",
                   help="If sensor doesn't re-enumerate after leaveDFU, print extra hints")
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
    interface.start(wait=True, wait_timeout=timeout)
    try:
        sensor = _sensor_handle(interface, device)
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


def main() -> int:
    args = parse_args()

    try:
        project = read_project_name(REPO_ROOT / "CMakeLists.txt")
        bin_file = bin_path_for(REPO_ROOT, args.config, project)
        dfu_util = resolve_dfu_util(args.dfu_util)
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
        return 1

    print(f"[*] Waiting for DFU device ({DFU_VID_PID_STR}) to enumerate …")
    if not wait_for_dfu_device(dfu_util, timeout=DFU_ENUM_TIMEOUT_S):
        print("❌ DFU device did not appear within "
              f"{DFU_ENUM_TIMEOUT_S:.0f}s.")
        print("   On Windows, bind WinUSB to the STM32 DFU interface "
              "(PID 0xDF11) via Zadig.")
        return 1

    print(f"[*] Flashing {bin_file.name} via dfu-util …")
    cmd = [
        dfu_util,
        "-d", DFU_VID_PID_STR,
        "-a", "0",
        "-s", f"0x{FLASH_BASE:08x}:leave",
        "-D", str(bin_file),
    ]
    rc = _run(cmd)
    if rc != 0:
        print(f"❌ dfu-util exited with {rc}. Device is still in DFU; rerun deploy.py to retry.")
        return 1

    print(f"[*] leaveDFU sent. Waiting for {args.device} sensor to come back …")
    if _wait_for_sensor_comeback(args.device, timeout=COMEBACK_TIMEOUT_S):
        print(f"✅ {args.device.capitalize()} sensor is back online.")
        return 0

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
