"""HIL: drip-scan image receive mode (OW_CAMERA_IMAGE_MODE = 0x30).

test_image_mode_enter_exit_and_histogram_resume -- runs against ANY FPGA
bitstream. Starts a normal 2-camera histogram scan, enters image mode
mid-stream (firmware must suspend histogram streaming and re-arm 2408-B
receives), verifies TYPE_IMAGE (0x03) envelopes arrive on the HISTO endpoint
(the FPGA, still in histogram mode, pushes 4100-B envelopes that the 2408-B
DMA chops into one full 'line' + a stalled partial -- which also exercises
the ~2 ms line timeout: gap counters must grow), exits image mode, and
verifies histogram frames resume WITHOUT the host re-enabling anything.

test_image_line_stream_smoke -- line-rate smoke with the FPGA in sweep mode.
Requires the feature/8 camera-FPGA bitstream (register map v2 at I2C 0x5A,
VERSION >= 0x02); skips otherwise. No sensor retiming (that is SDK-side
orchestration): with production HTS the FPGA overruns and pushes lines at
drain rate, which is exactly what a transport smoke needs. Validates the
2424-B envelope framing, the envelope CRC (util_crc16 convention: bytes
0..2419, i.e. excluding EOH), and the inner line magic/version.

Camera mask 0x03 = cam 0 (USART2 link) + cam 1 (SPI6 link via BDMA/SRAM4)
-- one camera from each link family, including the special-cased one.

Deploy first (firmware-only flash is enough for test 1):

    python scripts/deploy.py --device left --fw-only --no-confirm `
      --power-cycle-cmd "python C:\\Users\\ethan\\Projects\\openmotion-bloodflow-app\\tests\\shelly.py cycle"

Run on the bench:

    $env:OPENMOTION_HIL = "1"
    $env:OW_SENSOR_SIDE = "left"
    $env:OPENMOTION_POWER_CYCLE_CMD = "python C:\\Users\\ethan\\Projects\\openmotion-bloodflow-app\\tests\\shelly.py cycle"
    pytest tests/test_image_mode_hil.py -v -s

Power-cycle the sensor between streaming HIL runs (repo CLAUDE.md: wedge
risk when chaining streaming tests).
"""
import os
import queue
import struct
import subprocess
import threading
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

CONNECT_TIMEOUT_S = 15.0
CAM_MASK = 0x03          # cam 0 = USART2, cam 1 = SPI6/BDMA/SRAM4
CAMS = [0, 1]

OW_CAMERA_IMAGE_MODE = 0x30   # firmware Core/Inc/common.h (SDK constant lands with the companion SDK feature)
IMAGE_PKT = 2424              # envelope size, camera_manager.h IMAGE_PKT_TOTAL_SIZE
RESP_LEN = 36                 # image_mode_resp_t

# One 2-camera histogram frame: 10-B header + 2*(SOH+cam+4096+temp4+EOH) + CRC2+EOF
HISTO_FRAME_BYTES = 10 + 2 * (1 + 1 + 4096 + 4 + 1) + 3
PHASE_SECONDS = 3.0
MIN_HISTO_BYTES = 25 * HISTO_FRAME_BYTES   # ~120 frames nominal; 25 tolerates USB hiccups

# FPGA register map (feature/5, extended by feature/8 -- see camera-fpga
# tools/full_frame_capture/fpga_link.py)
FPGA_ADDR = 0x5A
REG_VERSION, REG_CTRL, REG_LINE_L, REG_LINE_H = 0x01, 0x03, 0x04, 0x05


def _make_crc_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _make_crc_table()


def _crc16(buf):
    crc = 0xFFFF
    for b in buf:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def _extract_image_packets(buf: bytes):
    """Scan a raw byte stream for well-formed 2424-B TYPE_IMAGE envelopes."""
    pkts, i = [], 0
    while True:
        j = buf.find(b"\xaa\x03", i)
        if j < 0 or j + IMAGE_PKT > len(buf):
            break
        pkt = buf[j:j + IMAGE_PKT]
        size = int.from_bytes(pkt[2:6], "little")
        if size == IMAGE_PKT and pkt[10] == 0xFF and pkt[2420] == 0xEE and pkt[2423] == 0xDD:
            pkts.append(pkt)
            i = j + IMAGE_PKT
        else:
            i = j + 1
    return pkts


def _envelope_crc_ok(pkt: bytes) -> bool:
    # Firmware convention (mirrors send_histogram_data): CRC over bytes
    # 0..2419 -- SOF through last payload byte, EXCLUDING the EOH.
    return _crc16(pkt[:2420]) == (pkt[2421] | (pkt[2422] << 8))


# Keep MotionInterface instances alive for the process lifetime (Win32
# hotplug wndproc thunk issue -- see test_histo_wedge_recovery_hil.py).
_INTERFACE_KEEPALIVE = []


@pytest.fixture
def sensor():
    from omotion import MotionInterface

    cycle_cmd = os.environ.get("OPENMOTION_POWER_CYCLE_CMD")
    if cycle_cmd:
        subprocess.run(cycle_cmd, shell=True, check=True, timeout=60)
        time.sleep(8.0)  # re-enumeration settle

    side = os.environ.get("OW_SENSOR_SIDE", "left")
    interface = MotionInterface()
    _INTERFACE_KEEPALIVE.append(interface)
    interface.start(wait=False)
    handle = interface.left if side == "left" else interface.right
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not handle.is_connected():
        time.sleep(0.2)
    if not handle.is_connected():
        interface.stop()
        pytest.fail(f"{side} sensor not reachable in {CONNECT_TIMEOUT_S:.0f}s")
    yield handle
    try:
        _image_mode(handle, enable=False)          # never leave image mode on
        handle.disable_aggregator_fsin()
        handle.disable_camera(CAM_MASK)
        handle.uart.histo.stop_streaming()
        handle.uart.histo.drain_final(IMAGE_PKT)
        handle.disable_camera_power(CAM_MASK)
    except Exception:
        pass
    interface.stop()


def _bring_up(sensor):
    """Power -> FPGA (FORCED) -> configure.

    Bench finding: the fleet cameras are NVCM-programmed, so the stock
    (non-forced) program path detects NVCM and silently skips the SRAM load,
    leaving the burned production image running. OW_FPGA_PROG_SRAM with
    reserved=2 (#68 hunk, firmware fe3f64e+) forces the flash-resident
    bitstream into SRAM (~10 s per camera)."""
    from omotion.config import OW_FPGA, OW_FPGA_PROG_SRAM
    from omotion.MotionSensor import _ERROR_TYPES

    assert sensor.enable_camera_power(CAM_MASK) is True
    time.sleep(0.5)
    # ONE CAMERA PER COMMAND: each force blocks the MCU main loop ~10.5 s;
    # a multi-camera mask in a single command blocks >20 s and stalls the
    # COMMS USB endpoint (bench-reproduced twice). Per-camera is also the
    # fleet-validated pattern (smoke_test.py / feature/5 RUNBOOK).
    for cam in CAMS:
        r = sensor._send(packetType=OW_FPGA, command=OW_FPGA_PROG_SRAM,
                         addr=(1 << cam), reserved=2, timeout=120)
        assert r is not None and r.packetType not in _ERROR_TYPES, (
            f"forced SRAM load failed on cam {cam}")
    time.sleep(0.1)
    assert sensor.camera_configure_registers(CAM_MASK) is True


def _image_mode(sensor, enable, mask=0):
    """Send OW_CAMERA_IMAGE_MODE; return (active, mask, gap_counts)."""
    from omotion.config import OW_CAMERA
    from omotion.MotionSensor import _ERROR_TYPES

    r = sensor._send(
        packetType=OW_CAMERA, command=OW_CAMERA_IMAGE_MODE,
        reserved=1 if enable else 0,
        data=bytes([mask]) if enable else None)
    assert r is not None and r.packetType not in _ERROR_TYPES, (
        f"OW_CAMERA_IMAGE_MODE {'enable' if enable else 'disable'} failed")
    assert r.data_len == RESP_LEN, f"unexpected response length {r.data_len}"
    payload = bytes(r.data[:RESP_LEN])
    active, rmask = payload[0], payload[1]
    gaps = struct.unpack("<8I", payload[4:36])
    return active, rmask, gaps


class _ByteCollector:
    """Accumulate raw HISTO-endpoint bytes off the stream queue."""

    def __init__(self, histo, expected_size):
        self._histo = histo
        self._expected = expected_size
        self._queue = queue.Queue(maxsize=1024)
        self._stop = threading.Event()
        self.buf = bytearray()
        self._thread = threading.Thread(target=self._drain, daemon=True)

    def _drain(self):
        while not self._stop.is_set():
            try:
                chunk = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if chunk:
                self.buf.extend(bytes(chunk))

    def start(self):
        self._thread.start()
        self._histo.start_streaming(self._queue, expected_size=self._expected)

    def stop(self) -> bytes:
        self._histo.stop_streaming()
        self._stop.set()
        self._thread.join(timeout=2.0)
        return bytes(self.buf)


def _fpga_read(sensor, cam, reg):
    val = sensor.i2c_read_register(FPGA_ADDR, reg, read_len=1,
                                   reg_addr_size=1, mux_channel=cam)
    if val is False or not val:
        return None
    return val[0]


def _fpga_write(sensor, cam, reg, value):
    # feature/5 write trick: 16-bit 'register address' = [reg, value].
    val = sensor.i2c_read_register(FPGA_ADDR, ((reg & 0xFF) << 8) | (value & 0xFF),
                                   read_len=1, reg_addr_size=2, mux_channel=cam)
    assert val is not False and val, f"cam{cam}: FPGA reg 0x{reg:02X} write failed"


@requires_bench
def test_image_mode_enter_exit_and_histogram_resume(sensor):
    assert sensor.ping() is True
    _bring_up(sensor)
    histo = sensor.uart.histo
    histo.flush_stale_data(HISTO_FRAME_BYTES)

    # Phase 1: normal histogram streaming must work first.
    col = _ByteCollector(histo, HISTO_FRAME_BYTES)
    col.start()
    assert sensor.enable_camera(CAM_MASK) is True
    assert sensor.enable_aggregator_fsin() is True
    time.sleep(PHASE_SECONDS)
    phase1 = col.stop()
    assert len(phase1) >= MIN_HISTO_BYTES, (
        f"histogram stream never started: {len(phase1)} B in {PHASE_SECONDS}s")

    # Phase 2: enter image mode MID-STREAM. Firmware suspends histograms.
    active, rmask, gaps0 = _image_mode(sensor, enable=True, mask=CAM_MASK)
    assert active == 1 and rmask == CAM_MASK
    assert all(g == 0 for g in gaps0), "gap counters must reset on enter"

    col = _ByteCollector(histo, IMAGE_PKT)
    col.start()
    time.sleep(PHASE_SECONDS)
    phase2 = col.stop()
    img_pkts = _extract_image_packets(phase2)
    # FPGA is still in histogram mode: each 40 Hz 4100-B envelope yields one
    # full 2408-B 'line' packet (then a stalled partial -> timeout resync).
    assert len(img_pkts) >= 10, (
        f"no TYPE_IMAGE packets while image mode active (got {len(img_pkts)})")
    assert all(p[11] in CAMS for p in img_pkts), "cam_id field out of mask"
    assert any(_envelope_crc_ok(p) for p in img_pkts), (
        "no image envelope passed the util_crc16 check -- framing broken")

    # The stalled-partial pattern must have exercised the line timeout.
    _, _, gaps1 = _image_mode(sensor, enable=False)
    assert any(gaps1[c] > 0 for c in CAMS), (
        f"line timeout never fired (gaps={gaps1}) -- watchdog not working")

    # Phase 3: histograms must RESUME with no host re-enable (firmware
    # restored event_bits_enabled + re-armed 4100-B receptions on exit).
    histo.flush_stale_data(HISTO_FRAME_BYTES)
    col = _ByteCollector(histo, HISTO_FRAME_BYTES)
    col.start()
    time.sleep(PHASE_SECONDS)
    phase3 = col.stop()
    assert len(phase3) >= MIN_HISTO_BYTES, (
        f"histogram stream did not resume after image-mode exit: {len(phase3)} B")
    assert sensor.ping() is True, "command interface unhealthy after image mode"


@requires_bench
def test_image_line_stream_smoke(sensor):
    assert sensor.ping() is True
    _bring_up(sensor)

    ver = _fpga_read(sensor, CAMS[0], REG_VERSION)
    if ver is None or ver < 0x02:
        pytest.skip(
            f"camera FPGA register map v2 required (VERSION={ver!r}); load the "
            "feature/8 bitstream (camera-fpga tools/full_frame_capture/"
            "update_bitstream.py) and re-run")

    histo = sensor.uart.histo
    histo.flush_stale_data(IMAGE_PKT)
    assert sensor.enable_camera(CAM_MASK) is True
    assert sensor.enable_aggregator_fsin() is True

    active, _, _ = _image_mode(sensor, enable=True, mask=CAM_MASK)
    assert active == 1

    col = _ByteCollector(histo, IMAGE_PKT)
    col.start()
    for cam in CAMS:
        _fpga_write(sensor, cam, REG_LINE_L, 0)
        _fpga_write(sensor, cam, REG_LINE_H, 0)   # H commits the pair
        _fpga_write(sensor, cam, REG_CTRL, 0x03)  # bit0 image + bit1 SWEEP
    time.sleep(PHASE_SECONDS)
    for cam in CAMS:
        _fpga_write(sensor, cam, REG_CTRL, 0x00)
    raw = col.stop()

    pkts = _extract_image_packets(raw)
    # No sensor retiming here (production HTS): the FPGA overruns and pushes
    # at drain rate (~1.4 kHz/camera) -- thousands of lines in 3 s. Demand a
    # conservative floor so USB hiccups don't flake the test.
    assert len(pkts) >= 100, f"sweep produced only {len(pkts)} line packets"
    crc_ok = [p for p in pkts if _envelope_crc_ok(p)]
    assert len(crc_ok) >= len(pkts) // 2, (
        f"only {len(crc_ok)}/{len(pkts)} envelopes CRC-clean")
    # Inner line header: magic 0xB6, format version 0x01 at the line offset.
    magic_ok = [p for p in crc_ok if p[12] == 0xB6 and p[13] == 0x01]
    assert magic_ok, "no envelope carried a valid FPGA line header (B6 01)"
    _image_mode(sensor, enable=False)
    assert sensor.ping() is True
