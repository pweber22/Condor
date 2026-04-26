import struct
import serial
import time

# =========================
# CONFIG
# =========================
PORT = "COM8"      
BAUD = 115200

# =========================
# MESSAGE TYPES (must match ICD)
# =========================
HEARTBEAT      = 0x01
TELEMETRY      = 0x02
COMMAND        = 0x03
MISSION_UPLOAD = 0x04
GUIDED_LOITER  = 0x05
ACK            = 0x06

# =========================
# HEADER FORMAT
# Matches C struct exactly:
#
# uint8  version
# uint8  type
# uint8  seq
# uint8  total_parts
# uint8  part_number
# uint16 payload_length
# uint16 checksum
# =========================
HEADER_FMT = "<BBBBBHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# =========================
# PAYLOAD FORMATS
# =========================

TELEMETRY_FMT = "<ii h H B H H"
# lat, lon, alt, gs, mode, batt, wp

GUIDED_FMT = "<ii h H"
# lat, lon, alt, radius

COMMAND_FMT = "<BBH"
# cmd_id, param, reserved

ACK_FMT = "<BB"
# seq, status

# =========================
# MAVLINK-STYLE PRINTERS
# =========================

def print_mavlink(name, **fields):
    line = f"[MAVLINK:{name}] "
    for k, v in fields.items():
        line += f"{k}={v} "
    print(line)


# =========================
# DECODERS
# =========================

def decode_telemetry(payload):
    lat, lon, alt, gs, mode, batt, wp = struct.unpack(TELEMETRY_FMT, payload)

    print_mavlink("GLOBAL_POSITION_INT",
        lat=lat,
        lon=lon,
        alt=alt,
        ground_speed=gs,
        mode=mode,
        battery_cV=batt,
        waypoint=wp
    )


def decode_guided(payload):
    lat, lon, alt, radius = struct.unpack(GUIDED_FMT, payload)

    print_mavlink("SET_POSITION_TARGET",
        lat=lat,
        lon=lon,
        alt=alt,
        radius=radius,
        mode="GUIDED_LOITER"
    )


def decode_command(payload):
    cmd_id, param, _ = struct.unpack(COMMAND_FMT, payload)

    cmd_map = {
        0x01: "RTL",
        0x02: "LOITER",
        0x03: "AUTO",
        0x04: "REBOOT"
    }

    print_mavlink("COMMAND_LONG",
        command=cmd_map.get(cmd_id, "UNKNOWN"),
        param=param
    )


def decode_ack(payload):
    seq, status = struct.unpack(ACK_FMT, payload)

    status_map = {
        0x00: "SUCCESS",
        0x01: "MISSING_FRAGMENT",
        0x02: "CRC_FAIL",
        0x03: "INVALID"
    }

    print_mavlink("ACK",
        seq=seq,
        status=status_map.get(status, "UNKNOWN")
    )


# =========================
# MAIN LOOP
# =========================

def main():

    ser = serial.Serial(PORT, BAUD, timeout=1)

    print("MeshLink decoder started...\n")

    while True:

        if ser.in_waiting < HEADER_SIZE:
            time.sleep(0.01)
            continue

        raw_header = ser.read(HEADER_SIZE)

        if len(raw_header) != HEADER_SIZE:
            continue

        version, mtype, seq, total, part, length, checksum = struct.unpack(
            HEADER_FMT, raw_header
        )

        payload = ser.read(length)

        # =========================
        # ROUTING
        # =========================
        if mtype == TELEMETRY:
            decode_telemetry(payload)

        elif mtype == GUIDED_LOITER:
            decode_guided(payload)

        elif mtype == COMMAND:
            decode_command(payload)

        elif mtype == ACK:
            decode_ack(payload)

        else:
            print(f"[UNKNOWN] type={mtype} seq={seq}")


if __name__ == "__main__":
    main()