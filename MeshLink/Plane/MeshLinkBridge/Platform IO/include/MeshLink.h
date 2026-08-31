
#define MESHLINK_MSG_HEARTBEAT 0x01
#define MESHLINK_MSG_TELEMETRY 0x02
#define MESHLINK_MSG_COMMAND 0x03
#define MESHLINK_MSG_MISSION_UPLOAD 0x04
#define MESHLINK_MSG_GUIDED_LOITER 0x05
#define MESHLINK_MSG_ACK 0x06
#define MESHLINK_MSG_STATUS 0x07

#define MESHLINK_CMD_RTL 0x01
#define MESHLINK_CMD_LOITER 0x02
#define MESHLINK_CMD_AUTO 0x03
#define MESHLINK_CMD_REQUEST_STATUS 0x04
#define MESHLINK_CMD_REQUEST_TELEMETRY 0x05
#define MESHLINK_CMD_REBOOT_BRIDGE 0x06

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

extern String messageTypes[];
extern String commandIDs[];


struct telemetryPayload
{
    int32_t latitude;          // degrees * 1e7
    int32_t longitude;         // degrees * 1e7
    int16_t altitude_m;        // meters MSL
    uint16_t groundspeed_cms;  // cm/s
    uint16_t battery_cV;       // centivolts
    uint8_t flight_mode;
    uint16_t current_waypoint;
    int8_t rssi;               // optional, dBm or 0x7F if unavailable
    uint8_t heading_step;      // heading in 5° increments, 0..71
};

struct CommandPayload
{
    uint8_t command_id;
    uint8_t parameter;
    uint16_t reserved;
};

struct MissionWaypoint
{
    int32_t latitude;
    int32_t longitude;
    int16_t altitude_m;
    uint8_t waypoint_type;
};

struct GuidedLoiterPayload
{
    int32_t latitude;     // degrees * 1e7
    int32_t longitude;    // degrees * 1e7
    int16_t altitude_m;   // meters
    uint16_t radius_m;    // meters
};

struct AckPayload
{
    uint8_t acked_sequence;
    uint8_t status_code;
    uint16_t reserved;
};

struct StatusPayload
{
    uint8_t status_code;
    uint8_t subsystem;
    uint16_t reserved;
};
