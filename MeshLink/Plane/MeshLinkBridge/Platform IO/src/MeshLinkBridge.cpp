#include <Arduino.h>
#include "MeshLink.h"
#include "MeshLinkConfig.h"
#include "MAVLink_Mesh.h"
#include <string.h>
#include <stdlib.h>
#include <MAVLink_minimal.h>

const unsigned int MESH_BUF_SIZE = 256;
const unsigned int MAV_BUF_SIZE = 280;
static uint8_t meshBuf[MESH_BUF_SIZE];
static uint8_t mavBuf[MAV_BUF_SIZE];

const uint8_t MY_VEHICLE_ID = MESHLINK_VEHICLE_ID;
const uint8_t GROUND_STATION_ID = 0x00;
const uint8_t PROTOCOL_VERSION = 0x03; 
long base_latitude = 347000000;
long base_longitude = -1184000000;

uint8_t g_sequence = 0;
unsigned long lastTM = 0;
unsigned long lastHB = 0;

VehicleStatus vehicle;

int byteCount();
void printPacketSummary(const uint8_t*, size_t);
telemetryPayload generateTM();
telemetryPayload getTM();
mavlink_message_t mavMsg;
mavlink_status_t mavStatus;
void sendTM(telemetryPayload);
bool meshSerialToBuffer();
void processMeshBuffer();
bool mavSerialToBuffer();
void processMavBuffer();
int hexStringToBytes(const char*, uint8_t*, size_t);
void rebootBridge();
void sendCommandLong(uint16_t, float, float, float, float, float, float, float);
void printCommandAck(const mavlink_message_t &msg);
void RTL();
void loiter();
void FMAuto();
void guidedLoiter(int32_t, int32_t, int32_t, uint16_t);
void sendHeartbeat();


void setup() {
  Serial.begin(115200);
  MESHTASTIC_SERIAL.begin(115200);
  MAVLINK_SERIAL.begin(115200);
  delay(100);
  Serial.println(F("MeshLink bridge boot"));
  Serial.println(F("Telemetry, flight controller, and command bridge initialized"));
  randomSeed(analogRead(0));
  lastTM = millis();
}

void loop() {
  if (meshSerialToBuffer()) {
    processMeshBuffer();
  }

  if (mavSerialToBuffer()) {
    processMavBuffer();
  }

  if (millis() - lastTM >= TELEMETRY_PERIOD_MS) {
    telemetryPayload tm = getTM();
    sendTM(tm);
    lastTM = millis();
  }

  if (millis() - lastHB >= 1000) {
    sendHeartbeat();
    lastHB = millis();
  }
}

bool meshSerialToBuffer() {
  static unsigned int currentIndex = 0;

  while (MESHTASTIC_SERIAL.available() > 0) {
    char incomingByte = (char)MESHTASTIC_SERIAL.read();

    if (incomingByte == '\n' || incomingByte == '\r') {
      if (currentIndex > 0) {
        meshBuf[currentIndex] = '\0';
        currentIndex = 0;
        return true;
      }
      continue;
    }

    if (currentIndex < MAX_MESHLINK_BUF_SIZE - 1) {
      meshBuf[currentIndex++] = incomingByte;
    } else {
      Serial.println(F("Buffer overflow, clearing message"));
      currentIndex = 0;
    }
  }
  return false;
}

bool mavSerialToBuffer() {

  while (MAVLINK_SERIAL.available() > 0) {
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
  char *line = meshBuf;
  while (*line == ' ' || *line == '\t') {
    ++line;
  }

  Serial.print(F("RX: "));
  Serial.println(line);

  char *hexPayload = NULL;
  char *token = strtok(line, ":");
  while (token != NULL) {
    char *trimmed = token;
    while (*trimmed == ' ' || *trimmed == '\t') {
      ++trimmed;
    }

    if (strcmp(trimmed, "LINK") == 0) {
      hexPayload = strtok(NULL, ":");
      break;
    }

    token = strtok(NULL, ":");
  }

  if (hexPayload == NULL) {
    Serial.println(F("Ignoring non-MeshLink text"));
    return;
  }


  while (*hexPayload == ' ' || *hexPayload == '\t') {
    ++hexPayload;
  }

  size_t payloadLen = strlen(hexPayload);
  Serial.print(F("Hex payload length: "));
  Serial.println(payloadLen);

  if (payloadLen == 0 || (payloadLen % 2) != 0) {
    Serial.println(F("Invalid hex payload length"));
    return;
  }

  uint8_t packetBytes[128];
  int byteCount = hexStringToBytes(hexPayload, packetBytes, sizeof(packetBytes));
  if (byteCount <= 0) {
    Serial.println(F("Hex decode failed"));
    return;
  }

  printPacketSummary(packetBytes, byteCount);

  if (byteCount < 11) {
    Serial.println(F("Packet too short for header"));
    return;
  }

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
    if (byteCount > 11) {
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

int hexStringToBytes(const char *hex, uint8_t *out, size_t outCap) {
  size_t inputLen = strlen(hex);
  if (inputLen == 0 || (inputLen % 2) != 0) {
    return 0;
  }

  int count = 0;
  for (size_t i = 0; i + 1 < inputLen && count < (int)outCap; i += 2) {
    uint8_t hi = 0;
    uint8_t lo = 0;
    if (!hexNibble(hex[i], hi) || !hexNibble(hex[i + 1], lo)) {
      return 0;
    }
    out[count++] = (uint8_t)((hi << 4) | lo);
  }
  return count;
}

void bytesToHex(const uint8_t *data, size_t len, char *out, size_t outCap) {
  static const char hexChars[] = "0123456789ABCDEF";
  size_t pos = 0;
  for (size_t i = 0; i < len && pos + 2 < outCap; ++i) {
    out[pos++] = hexChars[(data[i] >> 4) & 0x0F];
    out[pos++] = hexChars[data[i] & 0x0F];
  }
  out[pos] = '\0';
}

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
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
  tm.link_quality = 92;
  return tm;
}

telemetryPayload getTM() {
  telemetryPayload tm;
  tm.latitude = vehicle.latitude;
  tm.longitude = vehicle.longitude;
  tm.altitude_m = (int16_t)(vehicle.relativeAltitude / 1000);
  tm.groundspeed_cms = (uint16_t)(vehicle.groundspeed * 100);
  tm.battery_cV = (uint16_t)(vehicle.batteryVoltage / 10);
  tm.flight_mode = vehicle.customMode;
  tm.current_waypoint = 0;
  tm.rssi = 0;
  tm.link_quality = 0;
  return tm;
}

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
  *p++ = tm.link_quality;

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
  MESHTASTIC_SERIAL.print(F("LINK:"));
  MESHTASTIC_SERIAL.println(hexString);
}

void rebootBridge(){
  Serial.println("Rebooting bridge..."); 
  USB1_USBCMD = 0;  // disconnect USB
  delay(50);
  SCB_AIRCR = 0x05FA0004; // Reboot
  delay(100);
}

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