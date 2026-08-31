#include <Arduino.h>
#include "MeshtasticBridge.h"
#include "MeshLinkConfig.h"
#include <Meshtastic.h>
#include <string.h>

namespace {
const size_t FRAME_BUFFER_SIZE = 512;
const unsigned int READ_BUDGET = 32;
const uint8_t SERIAL_MAGIC_1 = 0x94;
const uint8_t SERIAL_MAGIC_2 = 0xC3;
const uint32_t MESH_BROADCAST_ADDR = 0xFFFFFFFFUL;
const uint32_t CONFIG_REQUEST_ID = 0x4D53484CUL;
const uint32_t CONFIG_RETRY_MS = 5000;
const uint32_t HEARTBEAT_MS = 60000;

uint8_t frameBuffer[FRAME_BUFFER_SIZE];
size_t frameLength = 0;
MeshtasticPayloadCallback payloadCallback = nullptr;
MeshtasticTextCallback textCallback = nullptr;
MeshtasticRssiCallback rssiCallback = nullptr;
uint32_t lastConfigRequest = 0;
uint32_t lastHeartbeat = 0;
bool configComplete = false;

// Encode an unsigned integer using protobuf varint format: seven data bits per
// byte, with the high bit indicating that another byte follows.
size_t writeVarint(uint8_t *out, size_t capacity, uint32_t value) {
  size_t count = 0;
  while (value >= 0x80) {
    if (count >= capacity) return 0;
    out[count++] = (uint8_t)((value & 0x7F) | 0x80);
    value >>= 7;
  }
  if (count >= capacity) return 0;
  out[count++] = (uint8_t)value;
  return count;
}

// Append a protobuf field whose wire type is varint.
size_t appendVarintField(uint8_t *out, size_t capacity, size_t pos,
                        uint32_t field, uint32_t value) {
  size_t tagLength = writeVarint(out + pos, capacity - pos, field << 3);
  if (tagLength == 0) return 0;
  pos += tagLength;
  size_t valueLength = writeVarint(out + pos, capacity - pos, value);
  return valueLength == 0 ? 0 : pos + valueLength;
}

// Append a signed protobuf integer after applying zig-zag encoding.
size_t appendSignedField(uint8_t *out, size_t capacity, size_t pos,
                         uint32_t field, int32_t value) {
  uint32_t encoded = ((uint32_t)value << 1) ^ (uint32_t)(value >> 31);
  return appendVarintField(out, capacity, pos, field, encoded);
}

// Append a length-delimited protobuf field, such as Data.payload.
size_t appendBytesField(uint8_t *out, size_t capacity, size_t pos,
                        uint32_t field, const uint8_t *data, size_t dataLength) {
  size_t tagLength = writeVarint(out + pos, capacity - pos, (field << 3) | 2);
  if (tagLength == 0) return 0;
  pos += tagLength;
  size_t lengthLength = writeVarint(out + pos, capacity - pos, (uint32_t)dataLength);
  if (lengthLength == 0) return 0;
  pos += lengthLength;
  if (pos + dataLength > capacity) return 0;
  memcpy(out + pos, data, dataLength);
  return pos + dataLength;
}

// Read one protobuf varint and advance offset; malformed input returns zero.
uint32_t readVarint(const uint8_t *data, size_t length, size_t &offset) {
  uint32_t value = 0;
  uint8_t shift = 0;
  while (offset < length && shift < 35) {
    uint8_t byte = data[offset++];
    value |= ((uint32_t)(byte & 0x7F)) << shift;
    if ((byte & 0x80) == 0) return value;
    shift += 7;
  }
  return 0;
}

// Build the Meshtastic MeshPacket and nested Data protobuf message.
size_t encodeMeshPacket(uint8_t *out, size_t capacity, const uint8_t *payload,
                        size_t payloadLength, uint32_t portNum) {
  uint8_t data[160];
  size_t dataLength = appendVarintField(data, sizeof(data), 0, 1, portNum);
  if (dataLength == 0) return 0;
  dataLength = appendBytesField(data, sizeof(data), dataLength, 2, payload, payloadLength);
  if (dataLength == 0) return 0;

  size_t packetLength = 0;
  packetLength = appendVarintField(out, capacity, packetLength, 1, MESHLINK_VEHICLE_ID);
  if (packetLength == 0) return 0;
  packetLength = appendVarintField(out, capacity, packetLength, 2, BROADCAST_ADDR);
  if (packetLength == 0) return 0;
  packetLength = appendVarintField(out, capacity, packetLength, 3, MESHTASTIC_CHANNEL_INDEX);
  if (packetLength == 0) return 0;
  return appendBytesField(out, capacity, packetLength, 4, data, dataLength);
}

// Add the Meshtastic 0x94C3 serial header and length around a ToRadio message.
size_t encodeFrame(uint8_t *out, size_t capacity, const uint8_t *payload,
                   size_t payloadLength, uint32_t portNum) {
  uint8_t packet[192];
  size_t packetLength = encodeMeshPacket(packet, sizeof(packet), payload, payloadLength, portNum);
  if (packetLength == 0) return 0;

  size_t radioLength = appendBytesField(out + 4, capacity - 4, 0, 1, packet, packetLength);
  if (radioLength == 0 || radioLength > 0xFFFF) return 0;
  out[0] = SERIAL_MAGIC_1;
  out[1] = SERIAL_MAGIC_2;
  out[2] = (uint8_t)(radioLength >> 8);
  out[3] = (uint8_t)radioLength;
  return radioLength + 4;
}

// Encode and write one non-blocking-sized ToRadio packet to the node UART.
void sendPacket(const uint8_t *payload, size_t payloadLength, uint32_t portNum) {
  if (payloadLength > sizeof(meshtastic_Data_payload_t)) return;
  meshtastic_ToRadio toRadio = meshtastic_ToRadio_init_default;
  toRadio.which_payload_variant = meshtastic_ToRadio_packet_tag;
  toRadio.packet = meshtastic_MeshPacket_init_default;
  toRadio.packet.from = MESHLINK_VEHICLE_ID;
  toRadio.packet.to = MESH_BROADCAST_ADDR;
  toRadio.packet.channel = MESHTASTIC_CHANNEL_INDEX;
  toRadio.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
  toRadio.packet.decoded = meshtastic_Data_init_default;
  toRadio.packet.decoded.portnum = (meshtastic_PortNum)portNum;
  toRadio.packet.decoded.payload.size = payloadLength;
  memcpy(toRadio.packet.decoded.payload.bytes, payload, payloadLength);
  toRadio.packet.id = micros();

  uint8_t frame[FRAME_BUFFER_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(frame + 4, sizeof(frame) - 4);
  if (!pb_encode(&stream, meshtastic_ToRadio_fields, &toRadio)) return;
  frame[0] = SERIAL_MAGIC_1;
  frame[1] = SERIAL_MAGIC_2;
  frame[2] = (uint8_t)(stream.bytes_written >> 8);
  frame[3] = (uint8_t)stream.bytes_written;
  MESHTASTIC_SERIAL.write(frame, stream.bytes_written + 4);
}

void parseData(const uint8_t *data, size_t length) {
  size_t offset = 0;
  uint32_t portNum = 0;
  const uint8_t *payload = nullptr;
  size_t payloadLength = 0;

  while (offset < length) {
    uint32_t key = readVarint(data, length, offset);
    uint32_t field = key >> 3;
    uint32_t wire = key & 7;
    if (wire == 0) {
      uint32_t value = readVarint(data, length, offset);
      if (field == 1) portNum = value;
    } else if (wire == 2) {
      uint32_t valueLength = readVarint(data, length, offset);
      if (valueLength > length - offset) return;
      if (field == 2) {
        payload = data + offset;
        payloadLength = valueLength;
      }
      offset += valueLength;
    } else {
      return;
    }
  }

  if (portNum == MESHTASTIC_PORTNUM_TEXT && textCallback != nullptr && payload != nullptr) {
    char text[65];
    size_t copyLength = payloadLength < sizeof(text) - 1 ? payloadLength : sizeof(text) - 1;
    memcpy(text, payload, copyLength);
    text[copyLength] = '\0';
    textCallback(text);
  } else if (portNum == MESHTASTIC_PORTNUM_MESHLINK && payloadCallback != nullptr && payload != nullptr) {
    payloadCallback(payload, payloadLength);
  }
}

void parseMeshPacket(const uint8_t *packet, size_t length) {
  size_t offset = 0;
  while (offset < length) {
    uint32_t key = readVarint(packet, length, offset);
    uint32_t field = key >> 3;
    uint32_t wire = key & 7;
    if (wire == 2) {
      uint32_t valueLength = readVarint(packet, length, offset);
      if (valueLength > length - offset) return;
      if (field == 4) parseData(packet + offset, valueLength);
      offset += valueLength;
    } else if (wire == 0) {
      readVarint(packet, length, offset);
    } else {
      return;
    }
  }
}

// Decode one complete FromRadio frame using the library's generated protobuf schema.
void processFrame() {
  meshtastic_FromRadio fromRadio = meshtastic_FromRadio_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(frameBuffer, frameLength);
  if (pb_decode(&stream, meshtastic_FromRadio_fields, &fromRadio)) {
    if (fromRadio.which_payload_variant == meshtastic_FromRadio_packet_tag) {
      meshtastic_MeshPacket *packet = &fromRadio.packet;
      if (packet->which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
          packet->channel == MESHTASTIC_CHANNEL_INDEX) {
        meshtastic_Data *data = &packet->decoded;
        if (data->portnum == MESHTASTIC_PORTNUM_TEXT && textCallback != nullptr) {
          char text[65];
          size_t copyLength = data->payload.size < sizeof(text) - 1 ? data->payload.size : sizeof(text) - 1;
          memcpy(text, data->payload.bytes, copyLength);
          text[copyLength] = '\0';
          textCallback(text);
        } else if (data->portnum == MESHTASTIC_PORTNUM_MESHLINK) {
          if (rssiCallback != nullptr) rssiCallback(packet->rx_rssi);
          if (payloadCallback != nullptr) {
            payloadCallback(data->payload.bytes, data->payload.size);
          }
        }
      }
    } else if (fromRadio.which_payload_variant == meshtastic_FromRadio_config_complete_id_tag &&
               fromRadio.config_complete_id == CONFIG_REQUEST_ID) {
      configComplete = true;
      Serial.println(F("Meshtastic configuration handshake complete"));
      char onlineMessage[32];
      snprintf(onlineMessage, sizeof(onlineMessage), "Vehicle %u online", MESHLINK_VEHICLE_ID);
      meshtasticSendText(onlineMessage);
    } else if (fromRadio.which_payload_variant == meshtastic_FromRadio_rebooted_tag) {
      configComplete = false;
    }
  }
  frameLength = 0;
}
}

// Initialize the node UART and schedule the initial configuration handshake.
void meshtasticBegin() {
  MESHTASTIC_SERIAL.begin(115200);
  lastConfigRequest = millis() - CONFIG_RETRY_MS;
  Serial.println(F("Meshtastic serial transport ready"));
}

// Consume at most READ_BUDGET bytes and service handshake/heartbeat timers.
void meshtasticProcess() {
  static uint8_t state = 0;
  static uint16_t expectedLength = 0;
  static size_t receivedLength = 0;
  unsigned int bytesRead = 0;

  while (MESHTASTIC_SERIAL.available() > 0 && bytesRead++ < READ_BUDGET) {
    uint8_t byte = (uint8_t)MESHTASTIC_SERIAL.read();
    if (state == 0) {
      if (byte == SERIAL_MAGIC_1) state = 1;
    } else if (state == 1) {
      state = byte == SERIAL_MAGIC_2 ? 2 : (byte == SERIAL_MAGIC_1 ? 1 : 0);
    } else if (state == 2) {
      expectedLength = (uint16_t)byte << 8;
      state = 3;
    } else if (state == 3) {
      expectedLength |= byte;
      receivedLength = 0;
      if (expectedLength == 0 || expectedLength > FRAME_BUFFER_SIZE) {
        state = 0;
      } else {
        state = 4;
      }
    } else if (state == 4) {
      frameBuffer[receivedLength++] = byte;
      if (receivedLength >= expectedLength) {
        frameLength = receivedLength;
        state = 0;
        processFrame();
      }
    }
  }

  uint32_t now = millis();
  if (!configComplete && now - lastConfigRequest >= CONFIG_RETRY_MS) {
    meshtastic_ToRadio request = meshtastic_ToRadio_init_default;
    request.which_payload_variant = meshtastic_ToRadio_want_config_id_tag;
    request.want_config_id = CONFIG_REQUEST_ID;
    uint8_t frame[32];
    pb_ostream_t stream = pb_ostream_from_buffer(frame + 4, sizeof(frame) - 4);
    if (pb_encode(&stream, meshtastic_ToRadio_fields, &request)) {
      frame[0] = SERIAL_MAGIC_1;
      frame[1] = SERIAL_MAGIC_2;
      frame[2] = (uint8_t)(stream.bytes_written >> 8);
      frame[3] = (uint8_t)stream.bytes_written;
      MESHTASTIC_SERIAL.write(frame, stream.bytes_written + 4);
      lastConfigRequest = now;
    }
  } else if (configComplete && now - lastHeartbeat >= HEARTBEAT_MS) {
    meshtastic_ToRadio heartbeat = meshtastic_ToRadio_init_default;
    heartbeat.which_payload_variant = meshtastic_ToRadio_heartbeat_tag;
    heartbeat.heartbeat = meshtastic_Heartbeat_init_default;
    uint8_t frame[16];
    pb_ostream_t stream = pb_ostream_from_buffer(frame + 4, sizeof(frame) - 4);
    if (pb_encode(&stream, meshtastic_ToRadio_fields, &heartbeat)) {
      frame[0] = SERIAL_MAGIC_1;
      frame[1] = SERIAL_MAGIC_2;
      frame[2] = (uint8_t)(stream.bytes_written >> 8);
      frame[3] = (uint8_t)stream.bytes_written;
      MESHTASTIC_SERIAL.write(frame, stream.bytes_written + 4);
      lastHeartbeat = now;
    }
  }
}

void meshtasticSendMeshLink(const uint8_t *payload, size_t payloadLength) {
  sendPacket(payload, payloadLength, MESHTASTIC_PORTNUM_MESHLINK);
}

void meshtasticSendText(const char *text) {
  if (text == nullptr) return;
  size_t textLength = strlen(text);
  if (textLength == 0 || textLength > sizeof(meshtastic_Data_payload_t)) return;
  sendPacket((const uint8_t *)text, textLength, MESHTASTIC_PORTNUM_TEXT);
}

void meshtasticSendLocation(int32_t latitude, int32_t longitude, int32_t altitudeM) {
  if (latitude == 0 && longitude == 0) return;
  uint8_t position[48];
  size_t positionLength = appendSignedField(position, sizeof(position), 0, 1, latitude);
  positionLength = appendSignedField(position, sizeof(position), positionLength, 2, longitude);
  positionLength = appendVarintField(position, sizeof(position), positionLength, 3, (uint32_t)altitudeM);
  if (positionLength == 0) return;
  sendPacket(position, positionLength, MESHTASTIC_PORTNUM_POSITION);
}

void meshtasticSendTime(uint64_t unixUsec) {
  uint32_t unixSeconds = (uint32_t)(unixUsec / 1000000ULL);
  if (unixSeconds == 0) return;
  uint8_t timePayload[8];
  size_t timeLength = appendVarintField(timePayload, sizeof(timePayload), 0, 1, unixSeconds);
  if (timeLength > 0) sendPacket(timePayload, timeLength, MESHTASTIC_PORTNUM_TIME);
}

void meshtasticSetCallbacks(MeshtasticPayloadCallback newPayloadCallback,
                            MeshtasticTextCallback newTextCallback,
                            MeshtasticRssiCallback newRssiCallback) {
  payloadCallback = newPayloadCallback;
  textCallback = newTextCallback;
  rssiCallback = newRssiCallback;
}
