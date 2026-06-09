#!/usr/bin/env python3
"""Hypothesis test for the NVCM dropped-row bug.

Re-runs the burn on sacrificial camera 2 with a driver that sleeps 5 ms
after every LSC_PROG_INCR_NV (0x70) row-program transaction, emulating
the busy-wait the i2c_parser polling loop fails to perform.

Expected if hypothesis is correct: the row-1 data (01 2C 00 43 ...) now
lands at NVCM addr 1 and ORs over the previously mis-burned bits:
  82 91 15 F8 | 01 2C 00 43 = 83 BD 15 FB
"""
import sys
import time

sys.path.insert(0, r"C:\Users\ethan\Projects\openmotion-sdk")
sys.path.insert(0, r"C:\Users\ethan\Projects\openmotion-sdk\scripts")

from omotion import MotionInterface
from omotion.i2c_parser import isp_entry_point, ERR_MESSAGES
from test_factory_prog import HardwareDriver

ALGO = r"C:\Users\ethan\Projects\openmotion-camera-fpga\HistoFPGAFw\impl1\impl1_algo.iea"
DATA = r"C:\Users\ethan\Projects\openmotion-camera-fpga\HistoFPGAFw\impl1\impl1_data.ied"
CAM = 2  # sacrificial - NVCM already mis-burned, unrecoverable


class PacedDriver(HardwareDriver):
    """HardwareDriver that respects the NVCM row-program busy window."""

    prog_count = 0

    def stop(self) -> None:
        buf = bytes(self._write_buf)
        super().stop()
        if buf[:1] == b"\x70":
            PacedDriver.prog_count += 1
            time.sleep(0.005)


def main() -> int:
    iface = MotionInterface()
    iface.start(wait=False)
    time.sleep(3)
    sensor = iface.left
    if not sensor.is_connected():
        print("no sensor")
        iface.stop()
        return 1

    sensor.enable_camera_power(1 << (CAM - 1))
    time.sleep(0.5)

    driver = PacedDriver(sensor)
    driver.select_camera(CAM)

    t0 = time.monotonic()
    try:
        ret = isp_entry_point(ALGO, DATA, driver=driver)
    finally:
        elapsed = time.monotonic() - t0
        iface.stop()

    print(f"\nresult={ret} ({ERR_MESSAGES.get(ret, 'OK')})")
    print(f"row programs paced: {PacedDriver.prog_count}, elapsed {elapsed:.1f}s")
    return 0 if ret >= 0 else 1


if __name__ == "__main__":
    sys.exit(main())
