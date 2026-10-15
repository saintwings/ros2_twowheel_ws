"""Decoder for the binary IMU stream of the esp32-bno085 firmware.

Packet layout (41 bytes, little-endian):
    A5 5A | version:u8 | sequence:u16 | timestamp_us:u64 | 6x float32 |
    accel_status:u8 | gyro_status:u8 | CRC-16/CCITT-FALSE:u16
The CRC covers version through gyro_status.

- ax, ay, az: accelerometer, m/s^2 (gravity included)
- gx, gy, gz: calibrated gyroscope, rad/s
- *_status: 0=unreliable, 1=low, 2=medium, 3=high accuracy
"""

import struct
from dataclasses import dataclass

PACKET_SYNC = b"\xA5\x5A"
PACKET_SIZE = 41
PROTOCOL_VERSION = 1
_BODY_FORMAT = "<BHQ6fBB"


@dataclass
class ImuSample:
    sequence: int
    timestamp_us: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    accel_status: int
    gyro_status: int


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class PacketParser:
    """Incremental parser: feed raw serial bytes, get decoded samples back.

    Text the firmware prints at boot, and corrupted packets, are skipped
    by resyncing on the sync word and checking the CRC.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.crc_errors = 0

    def feed(self, raw):
        self._buffer.extend(raw)
        samples = []
        buf = self._buffer
        while True:
            sync_index = buf.find(PACKET_SYNC)
            if sync_index < 0:
                keep_sync_prefix = buf[-1:] == PACKET_SYNC[:1]
                buf[:] = PACKET_SYNC[:1] if keep_sync_prefix else b""
                break
            if sync_index:
                del buf[:sync_index]
            if len(buf) < PACKET_SIZE:
                break

            packet = bytes(buf[:PACKET_SIZE])
            expected_crc = struct.unpack_from("<H", packet, PACKET_SIZE - 2)[0]
            if crc16_ccitt(packet[2:-2]) != expected_crc:
                self.crc_errors += 1
                del buf[0]
                continue

            del buf[:PACKET_SIZE]
            fields = struct.unpack(_BODY_FORMAT, packet[2:-2])
            if fields[0] != PROTOCOL_VERSION:
                continue
            samples.append(ImuSample(*fields[1:]))
        return samples


def encode_packet(sample):
    """Build a packet exactly as the firmware does (used by tests)."""
    body = struct.pack(
        _BODY_FORMAT, PROTOCOL_VERSION, sample.sequence, sample.timestamp_us,
        sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz,
        sample.accel_status, sample.gyro_status,
    )
    return PACKET_SYNC + body + struct.pack("<H", crc16_ccitt(body))
