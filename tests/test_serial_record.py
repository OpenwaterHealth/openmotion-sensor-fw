"""Host unit tests for the sensor serial record codec (no hardware).

Locks the OW_CMD_SERIAL wire/flash format defined by Core/Src/sensor_serial.c.
Run with plain ``pytest tests/test_serial_record.py`` — no bench required.
"""
import struct

import pytest

from serial_record import (
    SERIAL_MAX_LEN,
    SERIAL_RECORD_MAGIC,
    SERIAL_RECORD_SIZE,
    SERIAL_RECORD_VERSION,
    crc16_ccitt,
    decode_record,
    encode_record,
    serial_is_valid_chars,
)


def test_crc16_ccitt_known_vector():
    """CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1."""
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_crc16_empty_is_init_value():
    assert crc16_ccitt(b"") == 0xFFFF


def test_record_is_exactly_32_bytes():
    assert len(encode_record("SN0001")) == SERIAL_RECORD_SIZE == 32


def test_encode_decode_round_trip():
    for serial in ("A", "SN0001", "OW" + "9" * (SERIAL_MAX_LEN - 2)):
        assert decode_record(encode_record(serial)) == serial


def test_encode_layout_matches_firmware_struct():
    """Field offsets/values match serial_record_t exactly."""
    rec = encode_record("SN42")
    magic, version, length, serial_bytes, reserved = struct.unpack("<HBB24sH", rec[:-2])
    (crc,) = struct.unpack("<H", rec[-2:])
    assert magic == SERIAL_RECORD_MAGIC == 0x4E53  # 'SN'
    assert version == SERIAL_RECORD_VERSION == 1
    assert length == 4
    assert serial_bytes[:4] == b"SN42"
    assert serial_bytes[4:] == b"\x00" * (SERIAL_MAX_LEN - 4)  # zero-padded tail
    assert reserved == 0
    assert crc == crc16_ccitt(rec[:-2])


def test_max_length_serial_is_accepted():
    serial = "Z" * SERIAL_MAX_LEN
    assert decode_record(encode_record(serial)) == serial


@pytest.mark.parametrize("bad", ["", "A" * (SERIAL_MAX_LEN + 1)])
def test_encode_rejects_bad_length(bad):
    with pytest.raises(ValueError):
        encode_record(bad)


@pytest.mark.parametrize("bad", ["sn0001", "SN-01", "SN 01", "SN_1", "café"])
def test_encode_rejects_illegal_chars(bad):
    with pytest.raises(ValueError):
        encode_record(bad)


@pytest.mark.parametrize(
    "serial,ok",
    [("ABC123", True), ("0", True), ("Z", True), ("abc", False), ("A!", False), ("", False)],
)
def test_serial_is_valid_chars(serial, ok):
    assert serial_is_valid_chars(serial) is ok


def test_decode_rejects_wrong_size():
    assert decode_record(b"\x00" * 31) is None
    assert decode_record(b"\x00" * 33) is None


def test_decode_rejects_bad_magic():
    rec = bytearray(encode_record("SN0001"))
    rec[0] ^= 0xFF  # corrupt magic (CRC still old -> also fails, but magic check first)
    assert decode_record(bytes(rec)) is None


def test_decode_rejects_bad_version():
    rec = bytearray(encode_record("SN0001"))
    rec[2] = 0xFE  # version byte
    # fix CRC so only the version check can reject it
    crc = crc16_ccitt(bytes(rec[:-2]))
    rec[-2:] = struct.pack("<H", crc)
    assert decode_record(bytes(rec)) is None


def test_decode_rejects_zero_length():
    rec = bytearray(encode_record("SN0001"))
    rec[3] = 0  # len byte
    crc = crc16_ccitt(bytes(rec[:-2]))
    rec[-2:] = struct.pack("<H", crc)
    assert decode_record(bytes(rec)) is None


def test_decode_rejects_crc_mismatch():
    rec = bytearray(encode_record("SN0001"))
    rec[-1] ^= 0xFF  # flip a CRC bit
    assert decode_record(bytes(rec)) is None


def test_decode_rejects_erased_flash():
    """All-0xFF (erased flash) must read as unprogrammed, not a valid record."""
    assert decode_record(b"\xff" * SERIAL_RECORD_SIZE) is None
