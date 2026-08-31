# MeshLink

## Native MeshLink Ground Control Station Protocol

**Version:** 4
**Date:** July 10, 2026
**Purpose:** Define the native MeshLink protocol between a ground control station and airborne systems for long-range, low-bandwidth UAV fleet operations.

---

# 1. System Overview

## 1.1 Architecture

```text
MeshLink Ground Control Station
    ↕ Meshtastic Serial/API
Ground Meshtastic Node
    ↕ LoRa MeshLink
Airborne Meshtastic Node
    ↕ UART
Airborne Bridge
    ↕ UART MAVLink
ArduPilot Flight Controller
```

## 1.2 Design Goals

* Support ultra-low-bandwidth telemetry and control
* Enable:
  * Telemetry display
  * Guided loiter commands
  * Mission upload
  * Fleet management
* Keep MeshLink supervisory, not transparent MAVLink
* Use compact, deterministic packet formats
* Ensure reliable command and mission delivery
* Minimize airtime and retransmission

---

# 2. Protocol Layers

## 2.1 MAVLink Layer

Used only on the aircraft:

* Airborne Bridge ↔ ArduPilot Flight Controller

The ground station never sends or receives MAVLink directly, and MAVLink packets are never transmitted over Meshtastic.

## 2.2 MeshLink Control Layer

Used for all ground-to-air and air-to-ground exchanges:

* Vehicle tracking
* Command intent
* Telemetry state
* Mission payload transfer
* Acknowledgements and status

## 2.3 Meshtastic Transport Layer

Used for packet transport only.

MeshLink packets are sent as raw binary payloads on Meshtastic custom port `369`.
This keeps the protocol compact and avoids the previous text-message hex wrapper.

Responsibilities:

* Radio transmission
* Routing
* Optional encryption
* Delivery metadata
* Binary MeshLink payload transport on custom port 369

---

# 3. Message Types

| Type ID | Name           | Direction     | Reliability      |
| ------- | -------------- | ------------- | ---------------- |
| 0x01    | HEARTBEAT      | Air → Ground  | Optional repeat  |
| 0x02    | TELEMETRY      | Air → Ground  | Best-effort      |
| 0x03    | COMMAND        | Ground → Air  | Reliable + ACK   |
| 0x04    | MISSION_UPLOAD | Ground → Air  | Fragmented + ACK |
| 0x05    | GUIDED_LOITER  | Ground → Air  | Reliable + ACK   |
| 0x06    | ACK            | Bidirectional | Reliable         |
| 0x07    | STATUS         | Bidirectional | Optional         |

---

# 4. Packet Structure

## 4.1 Standard Packet Header

All MeshLink packets use the same header format:

```c
struct PacketHeader
{
    uint8_t version;
    uint8_t source;
    uint8_t destination;
    uint8_t type;
    uint8_t sequence;
    uint8_t total_parts;
    uint8_t part;
    uint16_t payload_length;
    uint16_t crc16;
};
```

## 4.2 Header Field Definitions

* `version`: MeshLink protocol version. Current value = 4.
* `source`: logical sender Vehicle ID.
* `destination`: logical recipient Vehicle ID.
* `type`: packet type identifier.
* `sequence`: packet sequence number.
* `total_parts`: number of fragments in the sequence.
* `part`: current fragment index (1-based).
* `payload_length`: length of payload in bytes.
* `crc16`: CRC-16 over header and payload.

## 4.3 Packet Size

Header size: **11 bytes**

Maximum payload: **120 bytes**

Total maximum wire size: **131 bytes**

> Note: MeshLink packets are sent directly as binary payloads on Meshtastic port 369,
> so the payload remains compact and does not incur the previous hex-text encoding overhead.

---

# 5. Network Rules

## 5.1 Vehicle Addressing

* `0x00` = Ground Control Station
* `0x01`–`0xEF` = Aircraft
* `0xF0`–`0xFD` = Reserved
* `0xFE` = Broadcast
* `0xFF` = Invalid

Aircraft must process packets only when `destination` is their own Vehicle ID or `0xFE`.

## 5.2 Broadcast Rules

* Broadcast packets may be used for heartbeat and status polling.
* Broadcast must never command flight-critical behavior.
* Command, mission upload, and guided loiter packets MUST be unicast.

## 5.3 ACK Routing

* ACK packets use `source` and `destination` to return to the original sender.
* ACK payloads include the acknowledged sequence number.
* Airborne Bridge must ACK command and mission upload packets.

## 5.4 Sequence ID Handling

* `sequence` is an 8-bit counter per source/destination pair.
* Wrap-around is allowed.
* A new command or mission upload should use a new `sequence` value.

## 5.5 Duplicate Suppression

* Receivers must suppress duplicate packets by `source`, `destination`, `type`, and `sequence`.
* Fragments are reassembled once per unique packet sequence.
* Duplicate ACKs may be ignored if the original ACK was already processed.

---

# 6. Message Definitions

## 6.1 HEARTBEAT Packet

### Purpose

* Provide alive indication
* Report link and power state
* Monitor aircraft health

### Payload

```c
struct HeartbeatPayload
{
    uint32_t timestamp;
    uint8_t vehicle_id;
    uint8_t system_status;
    uint8_t flight_mode;
    uint16_t battery_cV;
};
```

### Size

10 bytes

### Reliability

* Periodic
* Best-effort
* Repeated when link quality degrades

### Notes

* `vehicle_id` must match packet `source`.
* `battery_cV` is centivolts.

## 6.2 TELEMETRY Packet

### Purpose

* Report aircraft state
* Support tracking and mission progress

### Payload

```c
struct TelemetryPayload
{
    int32_t latitude;          // degrees * 1e7
    int32_t longitude;         // degrees * 1e7
    int16_t altitude_m;        // meters MSL
    uint16_t groundspeed_cms;  // cm/s
    uint16_t battery_cV;       // centivolts
    uint8_t flight_mode;
    uint16_t current_waypoint;
    int8_t rssi;               // optional, dBm or 0x7F if unavailable
    uint8_t heading_step;      // heading in 5° increments, 0..71 (0°..355°)
};
```

### Size

19 bytes

### Reliability

* Best-effort
* No retransmission

### Notes

* `current_waypoint` indicates mission progress.
* `heading_step` stores heading in 5° increments, so `heading_step = 18` means 90.0°.
* `rssi` remains optional and may be set to a reserved value when unavailable.

## 6.3 COMMAND Packet

### Purpose

* Convey operator intent
* Trigger high-level aircraft action

### Payload

```c
struct CommandPayload
{
    uint8_t command_id;
    uint8_t parameter;
    uint16_t reserved;
};
```

### Size

4 bytes

### Reliability

* Reliable
* Requires ACK

### Notes

* Commands are abstract and not raw MAVLink commands.
* Commands MUST be unicast.

### Command IDs

| ID   | Command           |
| ---- | ----------------- |
| 0x01 | RTL               |
| 0x02 | LOITER            |
| 0x03 | AUTO              |
| 0x04 | REQUEST_STATUS    |
| 0x05 | REQUEST_TELEMETRY|
| 0x06 | REBOOT_BRIDGE     |

## 6.4 MISSION_UPLOAD Packet

### Purpose

* Upload mission or waypoint data to the aircraft

### Payload

```c
struct MissionWaypoint
{
    int32_t latitude;
    int32_t longitude;
    int16_t altitude_m;
    uint8_t waypoint_type;
};
```

### Size

Variable. Each waypoint is 11 bytes.

### Reliability

* Reliable
* Fragmented
* Requires ACK

### Notes

* Entire mission must be reassembled before forwarding to the flight controller.
* Mission data is treated as opaque payload by the airborne bridge until reassembly completes.

## 6.5 GUIDED_LOITER Packet

### Purpose

* Command the aircraft to loiter at a specified point

### Payload

```c
struct GuidedLoiterPayload
{
    int32_t latitude;     // degrees * 1e7
    int32_t longitude;    // degrees * 1e7
    int16_t altitude_m;   // meters
    uint16_t radius_m;    // meters
};
```

### Size

14 bytes

### Reliability

* Reliable
* Requires ACK

### Notes

* The airborne bridge translates this packet into MAVLink guided/loiter behavior.
* Packet must be unicast.

## 6.6 ACK Packet

### Purpose

* Confirm receipt of reliable packets
* Report success or error conditions

### Payload

```c
struct AckPayload
{
    uint8_t acked_sequence;
    uint8_t status_code;
    uint16_t reserved;
};
```

### Size

4 bytes

### Reliability

* Reliable

### Notes

* ACK must be sent from the packet recipient back to the sender.
* `acked_sequence` references the original packet sequence field.

### Status Codes

| Code | Meaning          |
| ---- | ---------------- |
| 0x00 | Success          |
| 0x01 | Missing fragment |
| 0x02 | CRC failure      |
| 0x03 | Invalid command  |
| 0x04 | Invalid packet   |

## 6.7 STATUS Packet

### Purpose

* Report subsystem state
* Signal non-critical alerts or diagnostics

### Payload

```c
struct StatusPayload
{
    uint8_t status_code;
    uint8_t subsystem;
    uint16_t reserved;
};
```

### Size

4 bytes

### Reliability

* Optional

### Notes

* STATUS may be sent from either aircraft or ground station.
* It is suitable for link health, mode transitions, or diagnostic alerts.

---

# 7. Reliability Strategy

## 7.1 Telemetry

* Best-effort delivery
* Latest packet wins
* No guaranteed retransmission

## 7.2 Commands

* Reliable delivery
* Retransmit until ACK received or limit reached
* Deduplicate by `source`/`destination`/`type`/`sequence`

## 7.3 Missions

* Fragmented transfer
* Reassemble before flight controller handoff
* ACK each complete upload
* Retry missing fragments

## 7.4 Heartbeat

* Periodic status reports
* Repeated during degraded link conditions

---

# 8. MeshLink Ground Control Station Responsibilities

## 8.1 Fleet Management

* Maintain vehicle list
* Track vehicle status and last-seen time
* Assign logical Vehicle IDs

## 8.2 Mission Planning

* Build mission payloads
* Fragment mission uploads
* Confirm full delivery via ACK

## 8.3 Vehicle Tracking

* Display telemetry and waypoint progress
* Monitor heartbeat status
* Detect link degradation

## 8.4 Logging

* Record received telemetry
* Log command dispatch and acknowledgements
* Archive mission uploads and status events

## 8.5 Reliable Retransmission

* Resend unacknowledged commands
* Resend mission fragments when missing
* Use sequence IDs for retransmission state

## 8.6 Link Monitoring

* Evaluate RSSI/link quality if available
* Generate alerts for lost aircraft
* Use heartbeat and status packets for health checks

---

# 9. Airborne Bridge Responsibilities

## 9.1 Packet Translation

* Translate MeshLink packets to MAVLink for ArduPilot
* Translate MAVLink state into MeshLink telemetry and heartbeat

## 9.2 Downlink

* Collect aircraft state from ArduPilot
* Compress state into MeshLink telemetry
* Send heartbeat and telemetry to the ground station

## 9.3 Uplink

* Receive command, mission, and guided loiter packets
* Reassemble fragmented missions
* Validate packet CRC and sequence
* Forward valid commands to ArduPilot

## 9.4 Mission Handling

* Buffer complete mission payloads
* Perform local MAVLink mission upload
* Manage flight controller mission handshake

## 9.5 Implementation Independence

* The airborne bridge is generic and not tied to a specific MCU.
* Any platform able to bridge UART MAVLink and Meshtastic may implement this section.

---

# 10. Recommended Data Rates

| Data Type             | Rate                |
| --------------------- | ------------------- |
| Heartbeat             | 1/min               |
| Telemetry             | 1/min baseline      |
| Guided loiter         | On demand           |
| Mission upload        | On demand           |
| Command delivery      | Immediate retry     |

---

# 11. Security Considerations

* Use Meshtastic encryption when available
* Validate all packet fields and CRC
* Reject malformed or invalid destination packets
* Ensure broadcast packets do not trigger flight-critical actions
* Keep Vehicle IDs stable across radio hardware changes

---

# 12. Failure Modes

| Failure               | Response                                |
| --------------------- | --------------------------------------- |
| Lost telemetry        | Ground timeout alert                    |
| Lost command          | Retransmit, alert if retries fail       |
| Lost mission fragment | Request resend or abort mission upload  |
| CRC error             | Reject packet                            |
| Missing ACK           | Retransmit reliable packet              |
| Link degradation      | Increase heartbeat frequency            |

---

# 13. Summary

## MeshLink Philosophy

MeshLink is not a transparent MAVLink radio. It is a supervisory control protocol optimized for extremely low-bandwidth long-range autonomous aircraft.

## Core Protocol Tenets

* Send compact state and operator intent only
* Avoid full autopilot stream forwarding
* Use logical Vehicle IDs, not radio hardware IDs
* Keep mission uploads reliable and bounded
* Keep telemetry best-effort and low-rate

### DO send:

* Compact state snapshots
* Reliable command packets
* Fragmented mission blobs

---