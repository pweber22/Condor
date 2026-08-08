#define MESHLINK_VEHICLE_ID 0x01

#define MESHTASTIC_SERIAL Serial2
#define MAVLINK_SERIAL Serial5
#define MAVLINK_BRIDGE_SYS_ID 0xff
#define MAVLINK_BRIDGE_COMP_ID 0x68

#define TELEMETRY_PERIOD_MS 20000       // time between telemetry packets in milliseconds

#define LOITER_DIRECTION 0              //   0 = clockwise, 1 = counter-clockwise
#define LOITER_RADIUS 10.0f            // loiter radius in meters