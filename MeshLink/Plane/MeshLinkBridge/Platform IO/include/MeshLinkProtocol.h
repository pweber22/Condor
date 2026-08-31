#ifndef MESHLINK_PROTOCOL_H
#define MESHLINK_PROTOCOL_H

#include <Arduino.h>

// Dispatch a decoded custom MeshLink payload to the bridge command handlers.
void processMeshLinkPayload(const uint8_t *payload, size_t payloadLen);

// Convert a hexadecimal string into bytes. Returns the number of bytes written.
int hexStringToBytes(const char *hex, uint8_t *out, size_t outCap);

// Convert bytes to an uppercase hexadecimal string.
void bytesToHex(const uint8_t *data, size_t len, char *out, size_t outCap);

// Calculate the CRC-16/CCITT checksum used by MeshLink packets.
uint16_t crc16(const uint8_t *data, size_t len);

#endif
