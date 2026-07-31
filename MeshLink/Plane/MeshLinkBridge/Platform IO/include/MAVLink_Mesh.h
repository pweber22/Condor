#define MAX_MAVLINK_BUF_SIZE 263;
#define MAX_MESHLINK_BUF_SIZE 256;

#include <MAVLink.h>

class VehicleStatus
{
public:

    //-------------------------
    // Connection
    //-------------------------

    bool connected = false;
    uint32_t lastHeartbeatMs = 0;

    uint8_t systemID = 0;
    uint8_t componentID = 0;

    //-------------------------
    // Flight Status
    //-------------------------

    uint8_t baseMode = 0;
    uint32_t customMode = 0;
    uint8_t systemState = 0;

    bool armed = false;

    //-------------------------
    // GPS
    //-------------------------

    int32_t latitude = 0;      // degrees * 1e7
    int32_t longitude = 0;     // degrees * 1e7
    int32_t altitudeMSL = 0;   // mm
    int32_t relativeAltitude = 0; // mm

    //-------------------------
    // Attitude
    //-------------------------

    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    //-------------------------
    // Airspeed
    //-------------------------

    float airspeed = 0;
    float groundspeed = 0;
    int16_t heading = 0;
    float throttle = 0;

    //-------------------------
    // Battery
    //-------------------------

    uint16_t batteryVoltage = 0;     // mV
    int16_t batteryCurrent = 0;      // cA
    int8_t batteryRemaining = -1;    // %

    //-------------------------
    // Update Functions
    //-------------------------

    void updateHeartbeat(const mavlink_message_t& msg)
    {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        systemID = msg.sysid;
        componentID = msg.compid;

        baseMode = hb.base_mode;
        customMode = hb.custom_mode;
        systemState = hb.system_status;

        armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

        connected = true;
        lastHeartbeatMs = millis();
    }

    void updateGPS(const mavlink_message_t& msg)
    {
        mavlink_global_position_int_t gps;
        mavlink_msg_global_position_int_decode(&msg, &gps);

        latitude = gps.lat;
        longitude = gps.lon;
        altitudeMSL = gps.alt;
        relativeAltitude = gps.relative_alt;
        heading = gps.hdg;
    }

    void updateAttitude(const mavlink_message_t& msg)
    {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&msg, &att);

        roll = att.roll;
        pitch = att.pitch;
        yaw = att.yaw;
    }

    void updateVFR(const mavlink_message_t& msg)
    {
        mavlink_vfr_hud_t hud;
        mavlink_msg_vfr_hud_decode(&msg, &hud);

        airspeed = hud.airspeed;
        groundspeed = hud.groundspeed;
        throttle = hud.throttle;
        heading = hud.heading;
    }

    void updateBattery(const mavlink_message_t& msg)
    {
        mavlink_sys_status_t sys;
        mavlink_msg_sys_status_decode(&msg, &sys);

        batteryVoltage = sys.voltage_battery;
        batteryCurrent = sys.current_battery;
        batteryRemaining = sys.battery_remaining;
    }

    //-------------------------
    // Generic Dispatcher
    //-------------------------

    void update(mavlink_message_t& msg)
    {
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_HEARTBEAT:
                updateHeartbeat(msg);
                break;

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                updateGPS(msg);
                break;

            case MAVLINK_MSG_ID_ATTITUDE:
                updateAttitude(msg);
                break;

            case MAVLINK_MSG_ID_VFR_HUD:
                updateVFR(msg);
                break;

            case MAVLINK_MSG_ID_SYS_STATUS:
                updateBattery(msg);
                break;

            default:
                break;
        }
    }

    //-------------------------
    // Connection Check
    //-------------------------

    bool isConnected() const
    {
        return connected;
    }

    //-------------------------
    // Timeout Check
    //-------------------------

    void updateTimeout(uint32_t timeoutMs = 3000)
    {
        if (millis() - lastHeartbeatMs > timeoutMs)
            connected = false;
    }
};