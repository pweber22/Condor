#include "MeshLinkProtocol.h"
#include "MeshLink.h"
#include "MeshLinkConfig.h"
#include <string.h>

// These bridge functions remain in MeshLinkBridge.cpp because they control the
// aircraft and MAVLink stream. The protocol module only decodes and dispatches.
extern telemetryPayload getTM();
extern void sendTM(telemetryPayload tm);
extern void rebootBridge();
extern void RTL();
extern void loiter();
extern void FMAuto();
extern void guidedLoiter(int32_t latitude, int32_t longitude, int32_t altitude, uint16_t radius);

namespace {

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

}

int hexStringToBytes(const char *hex, uint8_t *out, size_t outCap) {
  size_t inputLen = strlen(hex);
  if (inputLen == 0 || (inputLen % 2) != 0) return 0;

  int count = 0;
  for (size_t i = 0; i + 1 < inputLen && count < (int)outCap; i += 2) {
    uint8_t high = 0;
    uint8_t low = 0;
    if (!hexNibble(hex[i], high) || !hexNibble(hex[i + 1], low)) return 0;
    out[count++] = (uint8_t)((high << 4) | low);
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
  if (outCap > 0) out[pos] = '\0';
}

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void processMeshLinkPayload(const uint8_t *payload, size_t payloadLen) {
  if (payload == nullptr || payloadLen < 11) {
    Serial.println(F("Ignoring short MeshLink packet"));
    return;
  }

  PacketHeader packet;
  packet.version = payload[0];
  packet.source = payload[1];
  packet.destination = payload[2];
  packet.type = payload[3];
  packet.sequence = payload[4];
  packet.total_parts = payload[5];
  packet.part = payload[6];
  packet.payload_length = (uint16_t)payload[7] | ((uint16_t)payload[8] << 8);
  packet.crc16 = (uint16_t)payload[9] | ((uint16_t)payload[10] << 8);

  if (packet.destination != MESHLINK_VEHICLE_ID && packet.destination != 0xFE) return;

  if (packet.type == MESHLINK_MSG_GUIDED_LOITER && payloadLen >= 23) {
    int32_t latitude = payload[11] | ((int32_t)payload[12] << 8) |
                       ((int32_t)payload[13] << 16) | ((int32_t)payload[14] << 24);
    int32_t longitude = payload[15] | ((int32_t)payload[16] << 8) |
                        ((int32_t)payload[17] << 16) | ((int32_t)payload[18] << 24);
    int16_t altitude = (int16_t)(payload[19] | ((int16_t)payload[20] << 8));
    uint16_t radius = (uint16_t)(payload[21] | ((uint16_t)payload[22] << 8));
    guidedLoiter(latitude, longitude, altitude, radius);
  }

  if (packet.type == MESHLINK_MSG_COMMAND && payloadLen > 11) {
    switch (payload[11]) {
      case MESHLINK_CMD_RTL: RTL(); break;
      case MESHLINK_CMD_LOITER: loiter(); break;
      case MESHLINK_CMD_AUTO: FMAuto(); break;
      case MESHLINK_CMD_REQUEST_TELEMETRY: sendTM(getTM()); break;
      case MESHLINK_CMD_REBOOT_BRIDGE: rebootBridge(); break;
      default: break;
    }
  }
}
