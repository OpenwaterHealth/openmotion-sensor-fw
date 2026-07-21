"""Child reader for test_histo_wedge_recovery_hil — meant to be hard-killed.

Brings up two cameras, streams histograms, prints ``CHILD_STREAMING <bytes>``
once frames are flowing, then reads until killed. It never tears anything
down: the parent kills it with SIGKILL/taskkill, which is the only faithful
way to produce "host reader process died mid-transfer".

MotionInterface.stop() is NOT a substitute — ConnectionMonitor._teardown()
drives every handle to DISCONNECTED so the transport releases cleanly, which
is exactly the orderly shutdown this scenario must avoid.
"""
import queue
import sys
import time

CAM_MASK = 0x03
STREAM_TIMEOUT_S = 60.0


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else "left"

    from omotion import MotionInterface
    from omotion.MotionProcessing import HISTOGRAM_BYTES

    iface = MotionInterface()
    iface.start(wait=False)
    sensor = None
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline and sensor is None:
        sensor = next((s for s in iface.connected_sensors()
                       if getattr(s, "side", None) == side), None)
        time.sleep(0.3)
    if sensor is None:
        print("CHILD_FAIL no sensor", flush=True)
        return 1

    if not sensor.enable_camera_power(CAM_MASK):
        print("CHILD_FAIL camera power", flush=True)
        return 1
    time.sleep(0.5)
    if not sensor.program_fpga(camera_position=CAM_MASK, manual_process=False):
        print("CHILD_FAIL program_fpga", flush=True)
        return 1
    time.sleep(0.1)
    if not sensor.camera_configure_registers(CAM_MASK):
        print("CHILD_FAIL configure", flush=True)
        return 1

    q = queue.Queue()
    sensor.uart.histo.start_streaming(q, expected_size=HISTOGRAM_BYTES)
    if not sensor.enable_camera(CAM_MASK) or not sensor.enable_aggregator_fsin():
        print("CHILD_FAIL stream start", flush=True)
        return 1

    got = 0
    announced = False
    t_end = time.monotonic() + STREAM_TIMEOUT_S
    while time.monotonic() < t_end:
        try:
            chunk = q.get(timeout=0.2)
            if chunk:
                got += len(chunk)
        except queue.Empty:
            pass
        if not announced and got > 3 * HISTOGRAM_BYTES:
            print(f"CHILD_STREAMING {got}", flush=True)
            announced = True

    # Only reached if the parent never killed us -- that is a test bug, and
    # the parent treats it as one.
    print("CHILD_NOT_KILLED", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
