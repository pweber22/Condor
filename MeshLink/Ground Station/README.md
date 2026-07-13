# MeshLink Ground Station

A native MeshLink Ground Control Station GUI application.

## Features

* Map-based vehicle tracking
* Click-to-add mission waypoints
* Click-to-send guided loiter commands
* Buttons for RTL, LOITER, AUTO, status/telemetry requests, and bridge reboot
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

## Notes

* This app currently uses a simple grayscale map canvas and synthetic vehicle state.
* Replace `DummyTransport` in `ground_station.py` with a real serial or network transport for live Meshtastic integration.
