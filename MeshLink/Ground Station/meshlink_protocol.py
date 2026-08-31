from dataclasses import dataclass
from enum import IntEnum
import struct

PROTOCOL_VERSION = 4
MAX_PAYLOAD = 120
HEARTBEAT_INTERVAL = 60  # seconds
HEADER_FMT = "<BBBBBBBHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

class MessageType(IntEnum):
    HEARTBEAT = 0x01
    TELEMETRY = 0x02
    COMMAND = 0x03
    MISSION_UPLOAD = 0x04
    GUIDED_LOITER = 0x05
    ACK = 0x06
    STATUS = 0x07

class CommandID(IntEnum):
    RTL = 0x01
    LOITER = 0x02
    AUTO = 0x03
    REQUEST_STATUS = 0x04
    REQUEST_TELEMETRY = 0x05
    REBOOT_BRIDGE = 0x06

class AckStatus(IntEnum):
    SUCCESS = 0x00
    MISSING_FRAGMENT = 0x01
    CRC_FAILURE = 0x02
    INVALID_COMMAND = 0x03
    INVALID_PACKET = 0x04

class VehicleID(IntEnum):
    GCS = 0x00
    BROADCAST = 0xFE
    INVALID = 0xFF

@dataclass
class PacketHeader:
    version: int
    source: int
    destination: int
    type: int
    sequence: int
    total_parts: int
    part: int
    payload_length: int
    crc16: int

    def pack(self) -> bytes:
        return struct.pack(
            HEADER_FMT,
            self.version,
            self.source,
            self.destination,
            self.type,
            self.sequence,
            self.total_parts,
            self.part,
            self.payload_length,
            self.crc16,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "PacketHeader":
        if len(data) < HEADER_SIZE:
            raise ValueError("Header too short")
        fields = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        return cls(*fields)

@dataclass
class MeshLinkPacket:
    header: PacketHeader
    payload: bytes

    def pack(self) -> bytes:
        header = PacketHeader(
            version=self.header.version,
            source=self.header.source,
            destination=self.header.destination,
            type=self.header.type,
            sequence=self.header.sequence,
            total_parts=self.header.total_parts,
            part=self.header.part,
            payload_length=len(self.payload),
            crc16=0,
        )
        raw_without_crc = struct.pack(
            "<BBBBBBHH",
            header.version,
            header.source,
            header.destination,
            header.type,
            header.sequence,
            header.total_parts,
            header.part,
            header.payload_length,
        ) + self.payload
        header.crc16 = crc16(raw_without_crc)
        return struct.pack(
            HEADER_FMT,
            header.version,
            header.source,
            header.destination,
            header.type,
            header.sequence,
            header.total_parts,
            header.part,
            header.payload_length,
            header.crc16,
        ) + self.payload

    @classmethod
    def unpack(cls, raw: bytes) -> "MeshLinkPacket":
        header = PacketHeader.unpack(raw[:HEADER_SIZE])
        payload = raw[HEADER_SIZE:HEADER_SIZE + header.payload_length]
        if len(payload) != header.payload_length:
            raise ValueError("Payload length mismatch")
        computed = crc16(raw[:HEADER_SIZE - 2] + payload)
        if computed != header.crc16:
            raise ValueError("CRC mismatch")
        return cls(header, payload)


def crc16(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_command_payload(command_id: int, parameter: int = 0) -> bytes:
    return struct.pack("<BBH", command_id, parameter, 0)


def parse_telemetry_payload(payload: bytes) -> dict:
    if len(payload) != 19:
        raise ValueError(f"Telemetry payload must be 19 bytes, got {len(payload)}")
    latitude, longitude, altitude_m, groundspeed_cms, battery_cV, flight_mode, current_waypoint, rssi, heading_step = struct.unpack(
        "<iiHHHBHbB",
        payload,
    )
    heading_deg = heading_step * 5.0
    return {
        'latitude': latitude / 1e7,
        'longitude': longitude / 1e7,
        'altitude_m': altitude_m,
        'groundspeed_cms': groundspeed_cms,
        'battery_cV': battery_cV,
        'flight_mode': flight_mode,
        'current_waypoint': current_waypoint,
        'rssi': rssi,
        'heading_step': heading_step,
        'heading_deg': heading_deg,
    }


def build_guided_loiter_payload(latitude: int, longitude: int, altitude_m: int, radius_m: int) -> bytes:
    return struct.pack("<iiHH", latitude, longitude, altitude_m, radius_m)


def build_mission_upload_fragment(total_waypoints: int, first_waypoint_index: int, waypoints: list) -> bytes:
    header = struct.pack("<HHBB", total_waypoints, first_waypoint_index, len(waypoints), 0)
    body = b"".join(struct.pack("<iiHB", lat, lon, alt, wtype) for lat, lon, alt, wtype in waypoints)
    return header + body


def parse_ack_payload(payload: bytes) -> tuple:
    if len(payload) != 4:
        raise ValueError("Invalid ACK payload size")
    acked_sequence, status_code, _reserved = struct.unpack("<BBH", payload)
    return acked_sequence, status_code
