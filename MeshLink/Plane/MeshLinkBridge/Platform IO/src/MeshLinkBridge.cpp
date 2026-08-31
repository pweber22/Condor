#include <Arduino.h>
#include "MeshLink.h"
#include "MeshLinkConfig.h"
#include "MAVLink_Mesh.h"
#include "MeshtasticBridge.h"
#include "MeshLinkProtocol.h"
#include <string.h>
#include <stdlib.h>
#include <MAVLink_minimal.h>

// MAVLink-side buffers and parser budgets. The budgets keep either UART from
// monopolizing loop() when a device sends a burst of traffic.
const unsigned int MAV_BUF_SIZE = 280;
const unsigned int MAV_READ_BUDGET = 64;
const unsigned int MESH_READ_BUDGET = 32;
static uint8_t meshBuf[256];
static uint8_t mavBuf[MAV_BUF_SIZE];
static size_t meshFrameLen = 0;
static uint8_t receivedMeshPayload[128];
static size_t receivedMeshPayloadLen = 0;

const uint8_t MY_VEHICLE_ID = MESHLINK_VEHICLE_ID;
const uint8_t GROUND_STATION_ID = 0x00;
const uint8_t PROTOCOL_VERSION = MESHLINK_PROTOCOL_VERSION;
const uint8_t MESHTASTIC_SERIAL_MAGIC_1 = 0x94;
const uint8_t MESHTASTIC_SERIAL_MAGIC_2 = 0xC3;
long base_latitude = 347000000;
long base_longitude = -1184000000;

uint8_t g_sequence = 0;
unsigned long lastTM = 0;
unsigned long lastHB = 0;

VehicleStatus vehicle;
String messageTypes[] = {"HEARTBEAT", "TLEMETRY", "COMMAND", "MISSION_UPLOAD", "GUIDED_LOITER", "ACK", "STATUS"};
String commandIDs[] = {"RTL", "LOITER", "AUTO", "REQUEST_STATUS", "REQUEST_TELEMETRY", "REBOOT_BRIDGE"};

// MAVLink bridge state and business operations.
int byteCount();
telemetryPayload generateTM();
telemetryPayload getTM();
mavlink_message_t mavMsg;
mavlink_status_t mavStatus;
void sendTM(telemetryPayload);
bool mavSerialToBuffer();
void processMavBuffer();
void rebootBridge();
void sendCommandLong(uint16_t, float, float, float, float, float, float, float);
void printCommandAck(const mavlink_message_t &msg);
static void sendMavlinkStatusText(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
void RTL();
void loiter();
void FMAuto();
void guidedLoiter(int32_t, int32_t, int32_t, uint16_t);
void sendHeartbeat();

static void handleMeshLinkPayload(const uint8_t *payload, size_t payloadLen);
static void handleRadioText(const char *text);
static void handleRadioRssi(int32_t rssi);

/* Pass decoded custom packets to the MeshLink command dispatcher. */
static void handleMeshLinkPayload(const uint8_t *payload, size_t payloadLen) {
  processMeshLinkPayload(payload, payloadLen);
}

/* Pass received radio text to the MAVLink status-text sender. */
static void handleRadioText(const char *text) {
  if (text != nullptr) sendMavlinkStatusText(text, MAV_SEVERITY_INFO);
}

/* Store the RSSI metadata attached to each Meshtastic packet. */
static void handleRadioRssi(int32_t rssi) {
  vehicle.updateRadioRssi(rssi);
}

// Convert incoming radio text into a MAVLink status notification.
static void sendMavlinkStatusText(const char *text, uint8_t severity) {
  if (text == nullptr) return;
  char msgText[50];
  snprintf(msgText, sizeof(msgText), "%s", text);
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(MAVLINK_BRIDGE_SYS_ID, MAVLINK_BRIDGE_COMP_ID,
                              &msg, severity, msgText, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(mavBuf, &msg);
  MAVLINK_SERIAL.write(mavBuf, len);
}

// Initialize the debug console, Meshtastic transport, and flight-controller UART.
void setup() {
  Serial.begin(115200);
  meshtasticSetCallbacks(handleMeshLinkPayload, handleRadioText, handleRadioRssi);
  meshtasticBegin();
  MAVLINK_SERIAL.begin(115200);
  delay(100);
  Serial.println(F("MeshLink bridge boot"));
  Serial.println(F("Telemetry, flight controller, and command bridge initialized"));
  randomSeed(analogRead(0));
  lastTM = millis();
}

// Run each transport for a bounded amount of work, then service periodic jobs.
void loop() {
  meshtasticProcess();

  if (mavSerialToBuffer()) {
    processMavBuffer();
  }

  if (millis() - lastTM >= TELEMETRY_PERIOD_MS) {
    telemetryPayload tm = getTM();
    sendTM(tm);
    meshtasticSendLocation(vehicle.latitude, vehicle.longitude,
                 vehicle.altitudeMSL / 1000);
    meshtasticSendTime(vehicle.timeUnixUsec);
    lastTM = millis();
  }

  if (millis() - lastHB >= 1000) {
    sendHeartbeat();
    lastHB = millis();
  }
}

bool meshSerialToBuffer() {
  static uint8_t state = 0;
  static uint16_t frameLen = 0;
  static uint16_t currentIndex = 0;
  unsigned int bytesRead = 0;

  while (MESHTASTIC_SERIAL.available() > 0 && bytesRead++ < MESH_READ_BUDGET) {
    uint8_t incomingByte = (uint8_t)MESHTASTIC_SERIAL.read();

    switch (state) {
      case 0:
        if (incomingByte == MESHTASTIC_SERIAL_MAGIC_1) {
          state = 1;
        }
        break;
      case 1:
        if (incomingByte == MESHTASTIC_SERIAL_MAGIC_2) {
          state = 2;
        } else {
          state = 0;
          if (incomingByte == MESHTASTIC_SERIAL_MAGIC_1) {
            state = 1;
          }
        }
        break;
      case 2:
        frameLen = (uint16_t)incomingByte << 8;
        state = 3;
        break;
      case 3:
        frameLen |= incomingByte;
        currentIndex = 0;
        if (frameLen == 0 || frameLen >= MAX_MESHLINK_BUF_SIZE) {
          state = 0;
          frameLen = 0;
          currentIndex = 0;
          Serial.println(F("Invalid Meshtastic protobuf frame"));
        } else {
          state = 4;
        }
        break;
      case 4:
        if (currentIndex < frameLen) {
          meshBuf[currentIndex++] = incomingByte;
        }
        if (currentIndex >= frameLen) {
          meshFrameLen = currentIndex;
          meshBuf[currentIndex] = '\0';
          state = 0;
          frameLen = 0;
          return true;
        }
        break;
      default:
        state = 0;
        break;
    }
  }

  return false;
}

// Read and parse a bounded number of MAVLink bytes from the flight controller.
bool mavSerialToBuffer() {
  unsigned int bytesRead = 0;

  while (MAVLINK_SERIAL.available() > 0 && bytesRead++ < MAV_READ_BUDGET) {
    char c = (char)MAVLINK_SERIAL.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavMsg, &mavStatus)) {
        //Serial.print(F("MAVLink message received: ID "));
        //Serial.println(mavMsg.msgid);
        return true;
      }
    }

  return false;
}

void processMeshBuffer() {
  if (receivedMeshPayloadLen == 0) {
    return;
  }

  if (receivedMeshPayloadLen < 11) {
    Serial.println(F("Ignoring non-MeshLink Meshtastic packet"));
    return;
  }

  uint8_t packetBytes[128];
  memcpy(packetBytes, receivedMeshPayload, receivedMeshPayloadLen);
  PacketHeader packet;
  packet.version = packetBytes[0];
  packet.source = packetBytes[1];
  packet.destination = packetBytes[2];
  packet.type = packetBytes[3];
  packet.sequence = packetBytes[4];
  packet.total_parts = packetBytes[5];
  packet.part = packetBytes[6];
  packet.payload_length = (uint16_t)packetBytes[7] | ((uint16_t)packetBytes[8] << 8);
  packet.crc16 = (uint16_t)packetBytes[9] | ((uint16_t)packetBytes[10] << 8);

  Serial.print(F("Message type: "));
  if (packet.type >= 1 && packet.type <= 7) {
    Serial.println(messageTypes[packet.type - 1]);
  } else {
    Serial.println(F("UNKNOWN"));
  }

  if (packet.destination != MY_VEHICLE_ID && packet.destination != 0xFE) {
    Serial.print(F("Packet not for this vehicle (dest: "));
    Serial.print(packet.destination);
    Serial.print(F(", my id: "));
    Serial.print(MY_VEHICLE_ID);
    Serial.println(F(")"));
    return;
  }

  if (packet.type == MESHLINK_MSG_GUIDED_LOITER) {
    Serial.println(F("Guided loiter command received"));
    int32_t lat = packetBytes[11] | ((int32_t)packetBytes[12] << 8) | ((int32_t)packetBytes[13] << 16) | ((int32_t)packetBytes[14] << 24);
    int32_t lon = packetBytes[15] | ((int32_t)packetBytes[16] << 8) | ((int32_t)packetBytes[17] << 16) | ((int32_t)packetBytes[18] << 24);
    int16_t alt = (int16_t)(packetBytes[19] | ((int16_t)packetBytes[20] << 8));
    uint16_t radius = (uint16_t)(packetBytes[21] | ((uint16_t)packetBytes[22] << 8));
    guidedLoiter(lat, lon, alt, radius);
  }

  if (packet.type == MESHLINK_MSG_COMMAND) {
    if (receivedMeshPayloadLen > 11) {
      uint8_t commandId = packetBytes[11];
      Serial.print(F("Command id: "));
      Serial.println(commandId);
      if (commandId == MESHLINK_CMD_RTL) RTL();
      if (commandId == MESHLINK_CMD_LOITER) loiter();
      if (commandId == MESHLINK_CMD_AUTO) FMAuto();
      if (commandId == MESHLINK_CMD_REQUEST_TELEMETRY) {
        Serial.println(F("Telemetry request received, replying"));
        telemetryPayload tm = getTM();
        sendTM(tm);
      }
      if (commandId == MESHLINK_CMD_REBOOT_BRIDGE) {
        Serial.println(F("Reboot bridge command received"));
        rebootBridge();
      }
    } else {
      Serial.println(F("Command packet had no payload"));
    }
  }
}

void printCommandAck(const mavlink_message_t &msg) {
  if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
    return;
  }

  mavlink_command_ack_t ack;
  mavlink_msg_command_ack_decode(&msg, &ack);

  Serial.print(F("MAVLink ACK: cmd="));
  Serial.print(ack.command);
  Serial.print(F(" result="));
  Serial.print(ack.result);
  Serial.print(F(" progress="));
  Serial.print(ack.progress);
  Serial.print(F(" target_system="));
  Serial.print(ack.target_system);
  Serial.print(F(" target_component="));
  Serial.println(ack.target_component);
}

// Apply the most recently parsed MAVLink message to bridge state.
void processMavBuffer(){
      if (mavMsg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        printCommandAck(mavMsg);
      }
      vehicle.update(mavMsg);
}

void printPacketSummary(const uint8_t *data, size_t len) {
  Serial.print(F("Packet bytes: "));
  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  if (len >= 11) {
    Serial.print(F("Version: "));
    Serial.println(data[0]);
    Serial.print(F("Source: "));
    Serial.println(data[1]);
    Serial.print(F("Destination: "));
    Serial.println(data[2]);
    Serial.print(F("Type: "));
    Serial.println(data[3]);
    Serial.print(F("Sequence: "));
    Serial.println(data[4]);
    Serial.print(F("Part: "));
    Serial.println(data[6]);
    Serial.print(F("Payload length: "));
    Serial.println(((uint16_t)data[7]) | ((uint16_t)data[8] << 8));
    Serial.print(F("CRC16: "));
    Serial.println(((uint16_t)data[9]) | ((uint16_t)data[10] << 8));
  }
}

bool hexNibble(char c, uint8_t &out) {
  if (c >= '0' && c <= '9') {
    out = (uint8_t)(c - '0');
    return true;
  }
  if (c >= 'a' && c <= 'f') {
    out = (uint8_t)(10 + (c - 'a'));
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    out = (uint8_t)(10 + (c - 'A'));
    return true;
  }
  return false;
}

telemetryPayload generateTM() {
  telemetryPayload tm;
  tm.latitude = base_latitude + (int32_t)(millis() / random(8));
  tm.longitude = base_longitude + (int32_t)(millis() / random(6));
  tm.altitude_m = (int16_t)(200 + (millis() / 1000) % 100);
  tm.groundspeed_cms = (uint16_t)(800 + (millis() / 1000) % 400);
  tm.battery_cV = (uint16_t)(1380 - (millis() / 1000) % 50);
  tm.flight_mode = 4;
  tm.current_waypoint = 7;
  tm.rssi = -55;
  tm.heading_step = 18;  // 90.0°
  return tm;
}

// Build the compact telemetry structure from the latest vehicle state.
telemetryPayload getTM() {
  telemetryPayload tm;
  tm.latitude = vehicle.latitude;
  tm.longitude = vehicle.longitude;
  tm.altitude_m = (int16_t)(vehicle.relativeAltitude / 1000);
  tm.groundspeed_cms = (uint16_t)(vehicle.groundspeed * 100);
  tm.battery_cV = (uint16_t)(vehicle.batteryVoltage / 10);
  tm.flight_mode = vehicle.customMode;
  tm.current_waypoint = 0;
  tm.rssi = vehicle.radioRssi;
  int16_t headingDeg = vehicle.heading;
  if (headingDeg < 0) headingDeg = 0;
  if (headingDeg > 359) headingDeg = 359;
  tm.heading_step = (uint8_t)(headingDeg / 5);
  return tm;
}

// Serialize and send one custom MeshLink telemetry packet over Meshtastic.
void sendTM(telemetryPayload tm) {
  uint8_t payload[19];
  uint8_t *p = payload;

  memcpy(p, &tm.latitude, sizeof(tm.latitude));
  p += sizeof(tm.latitude);
  memcpy(p, &tm.longitude, sizeof(tm.longitude));
  p += sizeof(tm.longitude);
  memcpy(p, &tm.altitude_m, sizeof(tm.altitude_m));
  p += sizeof(tm.altitude_m);
  memcpy(p, &tm.groundspeed_cms, sizeof(tm.groundspeed_cms));
  p += sizeof(tm.groundspeed_cms);
  memcpy(p, &tm.battery_cV, sizeof(tm.battery_cV));
  p += sizeof(tm.battery_cV);
  *p++ = tm.flight_mode;
  memcpy(p, &tm.current_waypoint, sizeof(tm.current_waypoint));
  p += sizeof(tm.current_waypoint);
  *p++ = (uint8_t)tm.rssi;
  *p++ = tm.heading_step;

  uint8_t packetBytes[11 + sizeof(payload)];
  packetBytes[0] = PROTOCOL_VERSION;
  packetBytes[1] = MY_VEHICLE_ID;
  packetBytes[2] = GROUND_STATION_ID;
  packetBytes[3] = MESHLINK_MSG_TELEMETRY;
  packetBytes[4] = ++g_sequence;
  packetBytes[5] = 1;
  packetBytes[6] = 0;
  packetBytes[7] = (uint8_t)(sizeof(payload) & 0xFF);
  packetBytes[8] = (uint8_t)((sizeof(payload) >> 8) & 0xFF);
  packetBytes[9] = 0;
  packetBytes[10] = 0;
  memcpy(packetBytes + 11, payload, sizeof(payload));

  uint8_t crcInput[9 + sizeof(payload)];
  memcpy(crcInput, packetBytes, 9);
  memcpy(crcInput + 9, payload, sizeof(payload));
  uint16_t crc = crc16(crcInput, sizeof(crcInput));
  packetBytes[9] = (uint8_t)(crc & 0xFF);
  packetBytes[10] = (uint8_t)((crc >> 8) & 0xFF);

  char hexString[2 * (11 + sizeof(payload)) + 1];
  bytesToHex(packetBytes, sizeof(packetBytes), hexString, sizeof(hexString));

  Serial.print(F("TX telemetry packet: "));
  Serial.println(hexString);

  meshtasticSendMeshLink(packetBytes, sizeof(packetBytes));
}

void rebootBridge(){
  Serial.println("Rebooting bridge..."); 
  USB1_USBCMD = 0;  // disconnect USB
  delay(50);
  SCB_AIRCR = 0x05FA0004; // Reboot
  delay(100);
}

// Send a MAVLink COMMAND_LONG to the detected flight controller.
void sendCommandLong(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
  uint8_t targetSystem = (vehicle.systemID != 0) ? vehicle.systemID : 1;
  uint8_t targetComponent = (vehicle.componentID != 0) ? vehicle.componentID : MAV_COMP_ID_AUTOPILOT1;

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    MAVLINK_BRIDGE_SYS_ID,
    MAVLINK_BRIDGE_COMP_ID,
    &msg,
    targetSystem,
    targetComponent,
    command,
    0,
    p1,
    p2,
    p3,
    p4,
    p5,
    p6,
    p7
  );

  uint16_t len = mavlink_msg_to_send_buffer(mavBuf, &msg);
  MAVLINK_SERIAL.write(mavBuf, len);
}

void RTL(){     // Return to launch
  Serial.println("Return To Launch");
  sendCommandLong(
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
  );
}

void loiter(){  // Start loiter at current position and altitude
  Serial.println("Loiter");
  sendCommandLong(
    MAV_CMD_DO_SET_MODE,
    (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    (float)AP_MODE_LOITER,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
  );
}

void FMAuto(){  // Auto flight mode
  Serial.println("Flight Mode Auto");
  sendCommandLong(
    MAV_CMD_DO_SET_MODE,
    (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    (float)AP_MODE_AUTO,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
  );
}

void guidedLoiter(int32_t latitude, int32_t longitude, int32_t altitude, uint16_t radius){  // Guided waypoint target for fixed-wing aircraft
  const float latDeg = (float)latitude / 1.0e7f;
  const float lonDeg = (float)longitude / 1.0e7f;
  const float altM = (float)altitude;

  Serial.print(F("Guided waypoint received: lat="));
  Serial.print(latDeg, 7);
  Serial.print(F(" lon="));
  Serial.print(lonDeg, 7);
  Serial.print(F(" alt="));
  Serial.print(altM, 2);
  Serial.print(F(" m radius="));
  Serial.println((float)radius);

  sendCommandLong(
    MAV_CMD_DO_SET_MODE,
    (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    (float)AP_MODE_GUIDED,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
  );

  uint8_t targetSystem = (vehicle.systemID != 0) ? vehicle.systemID : 1;
  uint8_t targetComponent = (vehicle.componentID != 0) ? vehicle.componentID : MAV_COMP_ID_AUTOPILOT1;

  mavlink_message_t missionCountMsg;
  mavlink_msg_mission_count_pack(
    MAVLINK_BRIDGE_SYS_ID,
    MAVLINK_BRIDGE_COMP_ID,
    &missionCountMsg,
    targetSystem,
    targetComponent,
    1,
    MAV_MISSION_TYPE_MISSION,
    0
  );
  uint16_t countLen = mavlink_msg_to_send_buffer(mavBuf, &missionCountMsg);
  MAVLINK_SERIAL.write(mavBuf, countLen);

  mavlink_message_t waypointMsg;
  mavlink_msg_mission_item_int_pack(
    MAVLINK_BRIDGE_SYS_ID,
    MAVLINK_BRIDGE_COMP_ID,
    &waypointMsg,
    targetSystem,
    targetComponent,
    0,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    MAV_CMD_NAV_WAYPOINT,
    2,
    1,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    (int32_t)(latDeg * 1.0e7f),
    (int32_t)(lonDeg * 1.0e7f),
    altM,
    MAV_MISSION_TYPE_MISSION
  );

  uint16_t waypointLen = mavlink_msg_to_send_buffer(mavBuf, &waypointMsg);
  MAVLINK_SERIAL.write(mavBuf, waypointLen);
}

void sendHeartbeat(){
  //Serial.println("Heartbeat Sent");
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(MAVLINK_BRIDGE_SYS_ID, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_STANDBY);
  uint16_t len = mavlink_msg_to_send_buffer(mavBuf, &msg);

  MAVLINK_SERIAL.write(mavBuf, len);
}

