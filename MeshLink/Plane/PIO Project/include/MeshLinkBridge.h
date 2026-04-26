#pragma once
#include <Arduino.h>
#include "MeshLinkProtocol.h"

class MeshLinkBridge {
public:
    MeshLinkBridge(Stream &fc, Stream &mesh);

    void update();

    // OUTBOUND (air → ground)
    void sendHeartbeat();
    void sendTelemetry(const MeshLink::TelemetryPayload &t);

private:
    Stream &fc;
    Stream &mesh;

    uint8_t seq = 0;

    uint8_t nextSeq();

    void sendPacket(uint8_t type, const uint8_t *data, uint16_t len);

    // INBOUND
    void handleIncoming();

    void handleCommand(const uint8_t *data);
    void handleGuidedLoiter(const uint8_t *data);
    void handleAck(const uint8_t *data);
};