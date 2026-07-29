"""Off-bench checks pinning the drip-scan image-mode wire format.

These parse the REAL firmware sources (no compilation) and fail if constants
the SDK/FPGA sides depend on ever drift:

* test_image_line_and_packet_constants -- IMAGE_LINE_SIZE / TYPE_IMAGE exist
  in Core/Inc/common.h with the pinned values; the envelope offset macros
  exist in camera_manager.h; the 2424-B envelope fits one HISTO USB transfer
  (USB_HISTO_MAX_SIZE) and the in-place receive slot (HISTOGRAM_DATA_SIZE).
* test_crc16_table_is_ccitt_false -- the 256-entry crc16_tab in
  Core/Src/utils.c is exactly the CRC-16/CCITT-FALSE table (poly 0x1021,
  MSB-first) and util_crc16's algorithm (init 0xFFFF, no reflection, no final
  XOR) reproduces the documented vectors. The FPGA's per-line CRC and the
  SDK's envelope verifier must both match these byte-for-byte.

Run anywhere: pytest tests/test_image_packet_format.py -v
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON_H = ROOT / "Core" / "Inc" / "common.h"
CAMERA_MANAGER_H = ROOT / "Core" / "Inc" / "camera_manager.h"
USBD_HISTO_H = ROOT / "USB" / "Class" / "HISTO" / "Inc" / "usbd_histo.h"
UTILS_C = ROOT / "Core" / "Src" / "utils.c"


def _define(text, name):
    m = re.search(rf"#define\s+{name}\s+\(?\s*(0x[0-9A-Fa-f]+|\d+)", text)
    assert m, f"#define {name} not found"
    return int(m.group(1), 0)


def test_image_line_and_packet_constants():
    common = COMMON_H.read_text()
    cam_h = CAMERA_MANAGER_H.read_text()
    histo_h = USBD_HISTO_H.read_text()

    image_line_size = _define(common, "IMAGE_LINE_SIZE")
    type_image = _define(common, "TYPE_IMAGE")
    # 6-B line header + 2400-B packed RAW10 (4 px -> 5 B) + 2-B line CRC.
    assert image_line_size == 2408
    # Must equal the SDK's OW_IMAGE_PACKET (omotion/config.py).
    assert type_image == 0x03

    # Envelope math: SOF+type+size4 (6) + timestamp4 + SOH + cam_id = 12,
    # then the line, then EOH (1) + CRC16 (2) + EOF (1).
    line_offset = 6 + 4 + 2
    total = line_offset + image_line_size + 1 + 3
    assert total == 2424
    assert "IMAGE_PKT_LINE_OFFSET" in cam_h
    assert "IMAGE_PKT_TOTAL_SIZE" in cam_h

    usb_max = _define(histo_h, "USB_HISTO_MAX_SIZE")
    assert total <= usb_max, "image packet must fit one HISTO transfer"

    slot = _define(cam_h, "HISTOGRAM_DATA_SIZE")
    assert total <= slot, (
        "the envelope is built in place inside each camera's existing "
        "receive-buffer slot, so it must fit HISTOGRAM_DATA_SIZE")


def _parse_fw_crc_table():
    text = UTILS_C.read_text()
    m = re.search(r"crc16_tab\[256\]\s*=\s*\{(.*?)\};", text, re.S)
    assert m, "crc16_tab not found in utils.c"
    entries = [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]{4}", m.group(1))]
    assert len(entries) == 256
    return entries


def _make_ccitt_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


def _crc16(table, buf):
    crc = 0xFFFF
    for b in buf:
        crc = ((crc << 8) ^ table[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def test_crc16_table_is_ccitt_false():
    fw_table = _parse_fw_crc_table()
    assert fw_table == _make_ccitt_table(), (
        "utils.c crc16_tab is not the CRC-16/CCITT-FALSE (poly 0x1021) table")
    # Standard CRC-16/CCITT-FALSE check value.
    assert _crc16(fw_table, b"123456789") == 0x29B1
    # Image-line vector: header B6 01 2A 00 07 00 (line 42, flags 0,
    # frame_cnt 7) + 2400 zero pixel bytes = 2406 input bytes.
    line = bytes([0xB6, 0x01, 0x2A, 0x00, 0x07, 0x00]) + bytes(2400)
    assert _crc16(fw_table, line) == 0x030C
