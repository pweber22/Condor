#include <Arduino.h>
#include <MeshLinkBridge.h>

#define FC_SERIAL Serial
#define MESH_SERIAL Serial2

MeshLinkBridge Bridge(FC_SERIAL, MESH_SERIAL);

// Fake telemetry generator
void sendFakeTelemetry() {

    MeshLink::TelemetryPayload t;

    t.latitude = 340000000;
    t.longitude = -118000000;
    t.altitude_m = 120;
    t.groundspeed_cms = 2500;
    t.flight_mode = 3;
    t.battery_cV = 396;
    t.current_waypoint = 5;

    Bridge.sendTelemetry(t);

    Serial.println("[TEST] Telemetry sent");
}

// COMMAND PARSER (from Serial Monitor)
void handleInput(String cmd) {

    cmd.trim();

    Serial.print("[INPUT] ");
    Serial.println(cmd);

    // ---- FAKE TELEMETRY TRIGGER ----
    if (cmd == "TELEMETRY") {
        sendFakeTelemetry();
        return;
    }

    // ---- PASS THROUGH TO BRIDGE ----
    // We simulate FC input by writing into Serial buffer
    // (bridge reads from FC serial internally)

    FC_SERIAL.println(cmd);
}

void setup() {

      Serial.begin(115200);
    while (!Serial) {}

    Serial.println("\n=== MeshLink Test Harness ===");
    Serial.println("Type commands:");
    Serial.println("  RTL");
    Serial.println("  LOITER");
    Serial.println("  AUTO");
    Serial.println("  GUIDED");
    Serial.println("  TELEMETRY");
    Serial.println("================================\n");

  FC_SERIAL.begin(115200);
  MESH_SERIAL.begin(115200);
}


void loop() {

    // -----------------------------
    // USER INPUT (USB Serial)
    // -----------------------------
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleInput(cmd);
    }

    // -----------------------------
    // UPDATE BRIDGE LOGIC
    // -----------------------------
    Bridge.update();

    // -----------------------------
    // DEBUG: mirror FC output
    // -----------------------------
    while (FC_SERIAL.available()) {
        String out = FC_SERIAL.readStringUntil('\n');

        Serial.print("[FC MOCK] ");
        Serial.println(out);
    }

    delay(10);
}


