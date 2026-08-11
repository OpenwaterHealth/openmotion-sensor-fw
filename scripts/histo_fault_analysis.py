"""Small, dependency-free helpers for histogram fault-injection HIL tests."""

from collections.abc import Iterable, Sequence
from typing import TypeVar


Row = TypeVar("Row", bound=Sequence[object])


def packetize_rows(rows: Iterable[Row]) -> list[list[Row]]:
    """Recover packet boundaries, including consecutive frozen timestamps.

    Histogram callbacks do not expose a packet identifier. A timestamp change
    normally starts a packet, while a repeated camera is the authoritative
    boundary when two consecutive packets carry the same timestamp.
    """
    packets: list[list[Row]] = []
    packet: list[Row] = []
    cameras: set[int] = set()
    timestamp = None

    for row in rows:
        camera = int(row[0])
        row_timestamp = float(row[2])
        if packet and (camera in cameras or row_timestamp != timestamp):
            packets.append(packet)
            packet = []
            cameras.clear()
        packet.append(row)
        cameras.add(camera)
        timestamp = row_timestamp

    if packet:
        packets.append(packet)
    return packets
