#include <MeshLinkBridge.h>

using namespace MeshLink;

MeshLinkBridge::MeshLinkBridge(Stream &fc, Stream &mesh)
    : fc(fc), mesh(mesh) {}

uint8_t MeshLinkBridge::nextSeq() {
    return seq++;
}

// =========================
// MAIN LOOP
// =========================
void MeshLinkBridge::update() {
    handleIncoming();
}

// =========================
// PACKET SENDER
// =========================
void MeshLinkBridge::sendPacket(uint8_t type, const uint8_t *data, uint16_t len) {

    MeshLink::PacketHeader hdr;
    hdr.protocol_version = MeshLink::PROTOCOL_VERSION;
    hdr.message_type = type;
    hdr.sequence_id = nextSeq();
    hdr.total_parts = 1;
    hdr.part_number = 0;
    hdr.payload_length = len;
    hdr.checksum = 0;

    mesh.write((uint8_t*)&hdr, sizeof(hdr));
    mesh.write(data, len);
}

// =========================
// HEARTBEAT
// =========================
void MeshLinkBridge::sendHeartbeat() {

    MeshLink::HeartbeatPayload hb;
    hb.timestamp = millis();
    hb.system_status = 1;
    hb.flight_mode = 0;

    sendPacket(MeshLink::HEARTBEAT, (uint8_t*)&hb, sizeof(hb));
}

// =========================
// TELEMETRY
// =========================
void MeshLinkBridge::sendTelemetry(const MeshLink::TelemetryPayload &t) {
    sendPacket(MeshLink::TELEMETRY, (uint8_t*)&t, sizeof(t));
}

// =========================
// INCOMING PARSER (simplified)
// =========================
void MeshLinkBridge::handleIncoming() {

    while (mesh.available()) {

        uint8_t buf[128];
        int len = mesh.readBytes(buf, sizeof(buf));

        if (len < static_cast<int>(sizeof(MeshLink::PacketHeader))) return;

        auto *hdr = (MeshLink::PacketHeader*)buf;
        uint8_t *payload = buf + sizeof(MeshLink::PacketHeader);

        switch (hdr->message_type) {

            case MeshLink::COMMAND:
                handleCommand(payload);
                break;

            case MeshLink::GUIDED_LOITER:
                handleGuidedLoiter(payload);
                break;

            case MeshLink::ACK:
                handleAck(payload);
                break;
        }
    }
}

// =========================
// COMMANDS
// =========================
void MeshLinkBridge::handleCommand(const uint8_t *data) {

    auto *cmd = (MeshLink::CommandPayload*)data;

    switch (cmd->command_id) {

        case MeshLink::CMD_RTL:
            fc.println("RTL");
            break;

        case MeshLink::CMD_LOITER:
            fc.println("LOITER");
            break;

        case MeshLink::CMD_AUTO:
            fc.println("AUTO");
            break;

        case MeshLink::CMD_REBOOT_BRIDGE:
            SCB_AIRCR = 0x05FA0004;
            break;
    }
}

// =========================
// GUIDED LOITER
// =========================
void MeshLinkBridge::handleGuidedLoiter(const uint8_t *data) {

    auto *g = (MeshLink::GuidedLoiterPayload*)data;

    fc.println("GUIDED");

    fc.print("LAT:");
    fc.println(g->latitude);

    fc.print("LON:");
    fc.println(g->longitude);

    fc.print("ALT:");
    fc.println(g->altitude_m);

    fc.print("RADIUS:");
    fc.println(g->radius_m);
}

// =========================
// ACK
// =========================
void MeshLinkBridge::handleAck(const uint8_t *data) {

    auto *ack = (MeshLink::AckPayload*)data;
    (void)ack;

    // placeholder for dedup / retry logic
}