# MeshLink Ground Station

A native MeshLink Ground Control Station GUI application.

## Features

* Map-based vehicle tracking
* Click-to-add mission waypoints
* Click-to-send guided loiter commands
* Buttons for RTL, LOITER, AUTO, telemetry requests, and bridge reboot
* Deterministic mission upload fragmenting
* Logical Vehicle ID support

## Files

* `meshlink_protocol.py` — MeshLink packet definitions and serialization
* `ground_station.py` — GUI application and command flow

## Running

Use the Python interpreter that has `tkinter` installed:

```bash
python ground_station.py
```

## Heltec V3 Node Configuration

Use the following setup on a Heltec V3 node for the MeshLink channel:

1. Load the official Meshtastic firmware.
2. Configure the node name and channel.
3. Add channel 1 key and name. This will be the MeshLink channel.
4. Enable the Serial module with mode `Proto`. Assign pins 6 to TX and 5 to RX. Set the baud rate to `115200`.
5. Use Meshtastic port `369` for MeshLink traffic. The payload is a binary MeshLink packet, not a hex string in the text-message app.

This config allows the ground station to talk to the air node over the dedicated MeshLink channel while keeping normal radio traffic separate.

## Notes

* This app currently uses a simple grayscale map canvas and synthetic vehicle state.
* Replace `DummyTransport` in `ground_station.py` with a real serial or network transport for live Meshtastic integration.
