# MeshLink

## Meshtastic ↔ MAVLink Long-Endurance UAV Communications Protocol

**Version:** 1.0
**Date:** April 26, 2026
**Purpose:** Define the communications protocol between ground control software and airborne systems for low-bandwidth UAV command, telemetry, and mission transfer over Meshtastic/LoRa.

---

# 1. System Overview

## 1.1 Architecture

```text
Mission Planner
    ↕ UDP MAVLink
Python Ground Bridge
    ↕ Meshtastic Serial/API
Ground Meshtastic Node
    ↕ LoRa MeshLink via Meshtastic
Airborne Meshtastic Node
    ↕ UART
Teensy Bridge MCU
    ↕ UART MAVLink
ArduPilot Flight Controller
```

---

## 1.2 Design Goals

* Support ultra-low-bandwidth telemetry (~1 msg/minute baseline)
* Enable:

  * Telemetry downlink
  * Command uplink
  * Mission upload/update
* Maintain compatibility with stock Meshtastic firmware
* Avoid full MAVLink-over-LoRa inefficiency
* Ensure robust fragmentation/reassembly
* Minimize power consumption and airtime

---

# 2. Protocol Layers

## 2.1 MAVLink Layer

Used locally only:

* Ground: Mission Planner ↔ Python bridge
* Air: Teensy ↔ Flight Controller

---

## 2.2 Custom Bridge Protocol Layer

Purpose:

* Convert MAVLink data into compact UAV-specific packets
* Fragment large data

---

## 2.3 Meshtastic Transport Layer

Used only for packet transport.

Responsibilities:

* Radio transmission
* Routing
* Encryption (optional)
* Delivery metadata

---

# 3. Message Types

| Type ID | Name           | Direction     | Reliability      |
| ------- | -------------- | ------------- | ---------------- |
| 0x01    | HEARTBEAT      | Air → Ground  | Optional repeat  |
| 0x02    | TELEMETRY      | Air → Ground  | No retry         |
| 0x03    | COMMAND        | Ground → Air  | Repeat + ACK     |
| 0x04    | MISSION_UPLOAD | Ground → Air  | Fragmented + ACK |
| 0x05    | GUIDED_LOITER  | Ground → Air  | Repeat + ACK     |
| 0x06    | ACK            | Bidirectional | Reliable         |
| 0x07    | STATUS/ERROR   | Bidirectional | Optional         |

---

# 4. Packet Structure

## 4.1 Standard Packet Header

```c
struct PacketHeader {
    uint8_t protocol_version;   // Current version = 1
    uint8_t message_type;       // Type ID
    uint8_t sequence_id;        // Message sequence number
    uint8_t total_parts;        // Total fragments
    uint8_t part_number;        // Current fragment index
    uint16_t payload_length;    // Bytes in payload
    uint16_t checksum;          // CRC16
};
```

### Header Size:

**9 bytes**

---

## 4.2 Maximum Payload Size

**120 bytes max payload per Meshtastic transmission**

### Reasoning:

* Avoids exceeding practical LoRa limits
* Preserves reliability margin
* Reduces transmission latency

---

# 5. Message Definitions

---

# 5.1 HEARTBEAT Packet

## Purpose:

* Link monitoring
* Connection health
* Vehicle alive indicator

```c
struct HeartbeatPayload {
    uint32_t timestamp;
    uint8_t system_status;
    uint8_t flight_mode;
    uint8_t battery_cV;
};
```

### Size:

7 bytes

### Frequency:

* Normal: every 60 sec
* Optional failsafe: every 15 sec

---

# 5.2 TELEMETRY Packet

## Purpose:

Transmit critical aircraft state.

```c
struct TelemetryPayload {
    int32_t latitude;         // degrees * 1e7
    int32_t longitude;        // degrees * 1e7
    int16_t altitude_m;       // meters MSL
    uint16_t groundspeed_cms; // cm/s
    uint16_t battery_cV;      // centivolts
    uint8_t flight_mode;
    uint16_t current_waypoint;
};
```

### Size:

17 bytes

### Required Fields:

* Position
* Altitude
* Battery
* Mode

---

# 5.3 COMMAND Packet

## Purpose:

Ground-issued aircraft control commands.

```c
struct CommandPayload {
    uint8_t command_id;
    uint8_t parameter;
    uint16_t reserved;
};
```

---

## Command IDs

| ID   | Command       |
| ---- | ------------- |
| 0x01 | RTL           |
| 0x02 | LOITER        |
| 0x03 | AUTO          |
| 0x04 | REBOOT BRIDGE |

---

## Reliability:

* Sent 3 times?
* Sequence deduplication required
* ACK strongly recommended

---

# 5.4 MISSION_UPLOAD Packet

## Purpose:

Transmit waypoint sets or mission blobs.

### Format:

Binary serialized mission blob.

```c
struct MissionWaypoint {
    int32_t latitude;
    int32_t longitude;
    int16_t altitude_m;
    uint8_t waypoint_type;
};
```

### Fragmentation:

* Required when >120 bytes
* Full mission reassembled before FC upload

---

## Airborne Reassembly Requirements

* Cache by sequence_id
* Verify all fragments
* Validate checksum
* Reject incomplete missions after timeout
* ACK successful reconstruction

---

# 5.5 GUIDED_LOITER Packet
## Purpose:
Command aircraft to move to a specified location and loiter (orbit/hold) at that point using guided control semantics.

This is a single-shot guided intent, not a continuous control stream.

```c
struct GuidedLoiterPayload {
    int32_t latitude;     // degrees * 1e7
    int32_t longitude;    // degrees * 1e7
    int16_t altitude_m;   // meters
    uint16_t radius_m;    // loiter radius
};
```

## Size:
14 bytes

## Behavior:
On reception, the bridge shall:
1. Switch Ardupilot mode to GUIDED
2. Send position target to aircraft
3. Optinally transition to LOITER after arrival 
4. Maintain hold until new command received

## Reliability:
* Sent 3–5 times for redundancy
* Requires ACK confirmation (recommended)
* Must be deduplicated using sequence_id

# 5.6 ACK Packet

```c
struct AckPayload {
    uint8_t acked_sequence_id;
    uint8_t status_code;
};
```

### Status Codes:

| Code | Meaning          |
| ---- | ---------------- |
| 0x00 | Success          |
| 0x01 | Missing fragment |
| 0x02 | CRC failure      |
| 0x03 | Invalid command  |

---

# 6. Fragmentation Rules

## 6.1 Trigger Condition

If:

```text
payload_length > MAX_PAYLOAD
```

Then:

* Split into multiple fragments
* Maintain identical sequence_id
* Set total_parts accordingly

---

## 6.2 Receiver Logic

```text
Receive fragment
→ Validate checksum
→ Store by sequence_id + part_number
→ If all parts present:
    Reassemble
    Validate full payload
    Process
```

---

## 6.3 Timeout

Recommended:

* 5–10 minutes for mission packets
* 2 minutes for commands

Incomplete sequences discarded after timeout.

---

# 7. Reliability Strategy

---

## Telemetry

* Stateless
* No retransmission
* Latest packet wins

---

## Commands

* Repeat send
* Deduplicate by sequence_id
* Optional ACK

---

## Missions

* Fragment ACK
* Missing fragment retry
* Full validation required

---

# 8. Python Ground Bridge Responsibilities

## 8.1 MAVLink Input

Receives:

* HEARTBEAT
* GPS
* SYS_STATUS
* COMMANDS
* MISSION_ITEMS

---

## 8.2 Translation

* Extract required state only
* Compress to bridge protocol
* Fragment as needed

---

## 8.3 Meshtastic Output

* Send packets via Meshtastic serial/API
* Track acknowledgments
* Retry commands/missions

---

## 8.4 Mission Planner Output

* Reconstruct MAVLink-compatible telemetry
* Forward via UDP
* Maintain heartbeat continuity

---

# 9. Airborne Teensy Responsibilities

## 9.1 Downlink

* Parse MAVLink from FC
* Generate telemetry packets
* Forward to Meshtastic node

---

## 9.2 Uplink

* Receive Meshtastic packets
* Reassemble
* Translate to MAVLink
* Inject into ArduPilot

---

## 9.3 Mission Handling

* Buffer full mission
* Perform MAVLink mission upload locally
* Manage FC handshake

---

# 10. Recommended Data Rates

| Data Type             | Rate           |
| --------------------- | -------------- |
| Heartbeat             | 1/min          |
| Telemetry             | 1/min baseline |
| Critical mode changes | Immediate      |
| Mission upload        | On demand      |

---

# 11. Security Considerations

* Use Meshtastic encryption where possible
* Validate all command IDs
* Reject malformed packets
* CRC mandatory
* Optional command authorization layer recommended

---

# 12. Future Expansion

Potential additions:

* Delta encoding
* Adaptive telemetry rates
* Sensor payload extensions
* Forward error correction
* Lightweight compression

---

# 13. Failure Modes

| Failure               | Response               |
| --------------------- | ---------------------- |
| Lost telemetry        | Ground timeout alert   |
| Lost command          | Retry                  |
| Lost mission fragment | Request resend         |
| CRC error             | Reject packet          |
| Bridge MCU reset      | Reinitialize heartbeat |

---

# 14. Development Priorities

## Prototype Phase

* Telemetry
* Heartbeats
* Basic commands

## Intermediate Phase

* Reliable command ACK
* Fragmentation
* Mission uploads

## Advanced Phase

* Compression
* Dynamic routing
* Power optimization

---

# 15. Summary

## Core Philosophy:

### Do NOT send:

* Full MAVLink streams
* High-rate telemetry
* Unbounded data

### DO send:

* Compact state snapshots
* Reliable command packets
* Fragmented mission blobs

---