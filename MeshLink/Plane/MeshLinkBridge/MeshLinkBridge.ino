#include "MeshLink.h"
#include <string.h>
#include <stdlib.h>

const unsigned int MAX_BUF_SIZE = 256;
static char buff[MAX_BUF_SIZE];

const uint8_t MY_VEHICLE_ID = 0x01;
const uint8_t GROUND_STATION_ID = 0x00;
const uint8_t PROTOCOL_VERSION = 0x03;
const unsigned long TELEMETRY_PERIOD_MS = 60000;

uint8_t g_sequence = 0;
unsigned long lastTM = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(100);
  Serial.println(F("MeshLink bridge boot"));
  Serial.println(F("Telemetry and command bridge initialized"));
  randomSeed(analogRead(0));
  lastTM = millis();
}

void loop() {
  if (readSerialToBuffer()) {
    processBuffer();
  }

  if (millis() - lastTM >= TELEMETRY_PERIOD_MS) {
    telemetryPayload tm = generateTM();
    sendTM(tm);
    lastTM = millis();
  }
}

bool readSerialToBuffer() {
  static unsigned int currentIndex = 0;

  while (Serial2.available() > 0) {
    char incomingByte = (char)Serial2.read();

    if (incomingByte == '\n' || incomingByte == '\r') {
      if (currentIndex > 0) {
        buff[currentIndex] = '\0';
        currentIndex = 0;
        return true;
      }
      continue;
    }

    if (currentIndex < MAX_BUF_SIZE - 1) {
      buff[currentIndex++] = incomingByte;
    } else {
      Serial.println(F("Buffer overflow, clearing message"));
      currentIndex = 0;
    }
  }
  return false;
}

void processBuffer() {
  char *line = buff;
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

  if (packet.type == MESHLINK_MSG_COMMAND) {
    if (byteCount > 11) {
      uint8_t commandId = packetBytes[11];
      Serial.print(F("Command id: "));
      Serial.println(commandId);
      if (commandId == MESHLINK_CMD_REQUEST_TELEMETRY) {
        Serial.println(F("Telemetry request received, replying"));
        telemetryPayload tm = generateTM();
        sendTM(tm);
      }
    } else {
      Serial.println(F("Command packet had no payload"));
    }
  }
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
  tm.latitude = 340000000 + (int32_t)(millis() / 3);
  tm.longitude = -1184000000 + (int32_t)(millis() / 5);
  tm.altitude_m = (int16_t)(200 + (millis() / 1000) % 100);
  tm.groundspeed_cms = (uint16_t)(800 + (millis() / 1000) % 400);
  tm.battery_cV = (uint16_t)(1180 + (millis() / 1000) % 50);
  tm.flight_mode = 4;
  tm.current_waypoint = 7;
  tm.rssi = -55;
  tm.link_quality = 92;
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
  Serial2.print(F("LINK:"));
  Serial2.println(hexString);
}
