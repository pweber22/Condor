#ifndef MESHTASTIC_BRIDGE_H
#define MESHTASTIC_BRIDGE_H

#include <Arduino.h>

using MeshtasticPayloadCallback = void (*)(const uint8_t *payload, size_t payloadLen);
using MeshtasticTextCallback = void (*)(const char *text);
using MeshtasticRssiCallback = void (*)(int32_t rssi);

void meshtasticBegin();
void meshtasticProcess();
void meshtasticSendMeshLink(const uint8_t *payload, size_t payloadLen);
void meshtasticSendText(const char *text);
void meshtasticSendLocation(int32_t latitude, int32_t longitude, int32_t altitudeM);
void meshtasticSendTime(uint64_t unixUsec);
void meshtasticSetCallbacks(MeshtasticPayloadCallback payloadCallback,
                            MeshtasticTextCallback textCallback,
                            MeshtasticRssiCallback rssiCallback);

#endif
