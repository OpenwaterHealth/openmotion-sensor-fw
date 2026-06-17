"""Reference codec for the sensor hardware-serial record (OW_CMD_SERIAL).

This is the host-side mirror of ``Core/Inc/sensor_serial.h`` / ``sensor_serial.c``
and matches the console's record byte-for-byte. It exists so the wire format is
locked by a host-runnable test and so the SDK has a tested reference to adopt
when the console serial feature lands.

On-wire record (packed, little-endian), exactly 32 bytes::

    offset size field
    0      2    magic    = 0x4E53 ('SN')
    2      1    version  = 1
    3      1    len      = 1..24, valid chars in serial[]
    4      24   serial   = ASCII uppercase alnum, NOT NUL-terminated
    28     2    reserved = 0
    30     2    crc16    = CRC16-CCITT over bytes [0..30)

Note that ``OW_CMD_SERIAL`` returns/accepts only the raw ASCII serial bytes
(``len`` of them) over the wire; the 32-byte record is the firmware's on-flash
representation. The command-level helpers below operate on the ASCII form.
"""
from __future__ import annotations

import struct

SERIAL_RECORD_MAGIC = 0x4E53
SERIAL_RECORD_VERSION = 1
SERIAL_MAX_LEN = 24
SERIAL_RECORD_SIZE = 32

# OW_CMD_SERIAL reserved-byte modes (mirror if_commands.c).
SERIAL_READ = 0
SERIAL_WRITE_GUARDED = 1
SERIAL_WRITE_FORCE = 2

_RECORD_FMT = "<HBB24sH"  # magic, version, len, serial[24], reserved (CRC appended)
assert struct.calcsize(_RECORD_FMT) == SERIAL_RECORD_SIZE - 2


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no final XOR.

    Matches ``serial_crc16`` in sensor_serial.c and ``crc16_ccitt`` in
    motion_config.c. Check value for b"123456789" is 0x29B1.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def serial_is_valid_chars(serial: str) -> bool:
    """True if every char is uppercase-alnum (0-9, A-Z) — the firmware rule."""
    if not serial:
        return False
    return all(("0" <= c <= "9") or ("A" <= c <= "Z") for c in serial)


def encode_record(serial: str) -> bytes:
    """Encode an ASCII serial into the 32-byte on-flash record.

    Raises ValueError if the serial is empty, too long, or has illegal chars.
    """
    if not (1 <= len(serial) <= SERIAL_MAX_LEN):
        raise ValueError(f"serial length must be 1..{SERIAL_MAX_LEN}, got {len(serial)}")
    if not serial_is_valid_chars(serial):
        raise ValueError("serial must be uppercase alphanumeric (0-9, A-Z)")

    serial_bytes = serial.encode("ascii").ljust(SERIAL_MAX_LEN, b"\x00")
    body = struct.pack(
        _RECORD_FMT,
        SERIAL_RECORD_MAGIC,
        SERIAL_RECORD_VERSION,
        len(serial),
        serial_bytes,
        0,  # reserved
    )
    crc = crc16_ccitt(body)
    return body + struct.pack("<H", crc)


def decode_record(record: bytes) -> str | None:
    """Decode a 32-byte record, returning the ASCII serial, or None if invalid.

    "Invalid" means wrong size/magic/version/len or a CRC mismatch — the same
    conditions ``record_valid()`` rejects in firmware.
    """
    if len(record) != SERIAL_RECORD_SIZE:
        return None
    magic, version, length, serial_bytes, _reserved = struct.unpack(
        _RECORD_FMT, record[:-2]
    )
    (crc_stored,) = struct.unpack("<H", record[-2:])

    if magic != SERIAL_RECORD_MAGIC:
        return None
    if version != SERIAL_RECORD_VERSION:
        return None
    if not (1 <= length <= SERIAL_MAX_LEN):
        return None
    if crc16_ccitt(record[:-2]) != crc_stored:
        return None
    return serial_bytes[:length].decode("ascii")
