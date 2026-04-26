#pragma once
#include <Arduino.h>

namespace MeshLink {

// =========================
// CONFIG
// =========================
static const uint8_t PROTOCOL_VERSION = 1;
static const uint16_t MAX_PAYLOAD = 120;

// =========================
// MESSAGE TYPES
// =========================
enum MessageType : uint8_t {
    HEARTBEAT      = 0x01,
    TELEMETRY      = 0x02,
    COMMAND        = 0x03,
    MISSION_UPLOAD = 0x04,
    GUIDED_LOITER  = 0x05,
    ACK            = 0x06,
    STATUS_ERROR   = 0x07
};

// =========================
// HEADER
// =========================
#pragma pack(push, 1)
struct PacketHeader {
    uint8_t  protocol_version;  // Current version
    uint8_t  message_type;      // Type ID
    uint8_t  sequence_id;       // Message sequence number
    uint8_t  total_parts;       // Total fragments
    uint8_t  part_number;       // Current fragment index
    uint16_t payload_length;    // Bytes in payload
    uint16_t checksum;          // CRC16
};
#pragma pack(pop)

// =========================
// PAYLOADS
// =========================

#pragma pack(push, 1)

struct HeartbeatPayload {
    uint32_t timestamp;
    uint8_t system_status;
    uint8_t flight_mode;
};

struct TelemetryPayload {
    int32_t latitude;           // degrees * 1e7
    int32_t longitude;          // degrees * 1e7
    int16_t altitude_m;         // meters MSL
    uint16_t groundspeed_cms;   // groundspeed cm/s
    uint16_t battery_cV;        // centivolts
    uint8_t flight_mode;
    uint16_t current_waypoint;
};

struct CommandPayload {
    uint8_t command_id;
    uint8_t parameter;
    uint16_t reserved;
};

struct MissionWaypoint {
    int32_t latitude;
    int32_t longitude;
    int16_t altitude_m;
    uint8_t waypoint_type;
};

struct GuidedLoiterPayload {
    int32_t latitude;           // degrees * 1e7
    int32_t longitude;          // degrees * 1e7
    int16_t altitude_m;         // meters MSL
    uint16_t radius_m;          // loiter radius
};

struct AckPayload {
    uint8_t acked_sequence_id;
    uint8_t status_code;
};

#pragma pack(pop)

// =========================
// COMMAND IDS
// =========================
enum CommandID : uint8_t {
    CMD_RTL           = 0x01,
    CMD_LOITER        = 0x02,
    CMD_AUTO          = 0x03,
    CMD_REBOOT_BRIDGE = 0x04
};

// =========================
// ACK STATUS
// =========================
enum AckStatus : uint8_t {
    ACK_SUCCESS        = 0x00,
    ACK_MISSING_FRAG   = 0x01,
    ACK_CRC_FAIL       = 0x02,
    ACK_INVALID_CMD    = 0x03
};

} // namespace MeshLink