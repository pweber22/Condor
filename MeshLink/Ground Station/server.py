import os
import json
import logging
import threading
from datetime import datetime
from typing import Any, cast

"""MeshLink Ground Station backend.

This module starts a Flask app and bridges the UI to a local Meshtastic
radio node. It handles Meshtastic serial transport, event subscriptions,
command/message packet serialization, and incoming message logging.
"""

from flask import Flask, jsonify, render_template, request
from flask_cors import CORS
from pubsub import pub
try:
    from meshtastic.serial_interface import SerialInterface
    from meshtastic.util import findPorts
    from meshtastic.protobuf import portnums_pb2
except ImportError:
    SerialInterface = None
    findPorts = None
    portnums_pb2 = None
try:
    from serial.tools import list_ports as serial_list_ports
except ImportError:
    serial_list_ports = None

from meshlink_protocol import (
    MessageType,
    CommandID,
    VehicleID,
    build_command_payload,
    build_guided_loiter_payload,
    build_mission_upload_fragment,
    parse_telemetry_payload,
    PacketHeader,
    MeshLinkPacket,
    PROTOCOL_VERSION,
    HEARTBEAT_INTERVAL,
)

os.makedirs("MeshLink\\Ground Station\\logs", exist_ok=True)
log_filename = datetime.now().strftime("MeshLink\\Ground Station\\logs/meshtastic_%Y-%m-%d.log")

telemetry_log = logging.getLogger("telemetry")
telemetry_log.setLevel(logging.INFO)


handler = logging.FileHandler(log_filename)

formatter = logging.Formatter(
    "%(asctime)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

handler.setFormatter(formatter)

telemetry_log.addHandler(handler)

app = Flask(__name__, template_folder='templates', static_folder='static')
CORS(app)

# Default serial port and baud rate for the local Meshtastic radio.
# These can be overridden with environment variables if needed.
SERIAL_PORT = os.getenv('MESHTASTIC_PORT', 'COM8')
SERIAL_BAUD = int(os.getenv('MESHTASTIC_BAUD', '115200'))

# Shared runtime state for the Flask app.
transport = None
selected_port = SERIAL_PORT
meshtastic_error = None
incoming_messages = []
MAX_INCOMING_MESSAGES = 200
subscribed = False
sequence = 0
active_vehicle = 1
mission_waypoints = []
heartbeat_stop = None
heartbeat_thread = None

# Current MeshLink version over Meshtastic custom app packets.
MESH_LINK_VERSION = '4'
MESHLINK_PORTNUM: int = 369
MESHLINK_TEXT_PREFIX = 'LINK:'

vehicles = {
}


class MeshtasticTransport:
    """Wrapper for Meshtastic transport.

    This is the ground station's bridge to the local Meshtastic node.
    The Meshtastic Python library opens the serial port and exposes a
    high-level interface for sending text and app-layer binary payloads.
    """
    def __init__(self, port: str):
        if SerialInterface is None:
            raise RuntimeError("meshtastic package is not installed")
        self.port = port
        # SerialInterface creates a pyserial.Serial stream internally on Windows
        # and manages the Meshtastic protocol framing for us.
        self.iface = SerialInterface(devPath=port, connectNow=True)

    def write(self, data: bytes, destination):
        # Send a raw Meshtastic application payload using the custom MeshLink
        # port. This carries the compact binary MeshLink packet directly rather
        # than wrapping it in a text-message hex string.
        self.iface.sendData(
            data,
            destinationId=destination,
            portNum=cast(Any, MESHLINK_PORTNUM),
            channelIndex=0,
            wantAck=True,
        )

    def send_text(self, text: str, destination=None):
        # Send a Meshtastic text message. The library's sendText() method
        # uses the same serial transport and default channelIndex.
        destinationId = '^all' if destination is None else destination
        self.iface.sendText(
            text,
            destinationId='^all',
            wantAck=True,
            channelIndex=0,
        )

    def close(self):
        if self.iface is not None:
            self.iface.close()


def connect_serial(port=None):
    """Open the Meshtastic serial transport and subscribe to event callbacks.

    This function reuses the existing transport when the port is unchanged.
    The `pub` subscriptions for receive and connection events are only
    registered once for the lifetime of the Flask process.
    """
    global transport, meshtastic_error, selected_port, subscribed
    if SerialInterface is None:
        meshtastic_error = 'meshtastic library is not installed'
        telemetry_log.info(meshtastic_error)
        return None

    if port is not None:
        selected_port = port

    # Subscribe once to the meshtastic pubsub events used by this app.
    if not subscribed:
        try:
            pub.subscribe(on_meshtastic_receive, 'meshtastic.receive')
            pub.subscribe(on_connection_established, 'meshtastic.connection.established')
            pub.subscribe(on_connection_lost, 'meshtastic.connection.lost')
            subscribed = True
        except Exception as exc:
            telemetry_log.info(f'Failed to subscribe to Meshtastic events: {exc}')

    port_to_use = selected_port
    if transport is not None:
        if transport.port == port_to_use:
            telemetry_log.info(f"Meshtastic transport already open on {port_to_use}")
            return transport
        try:
            telemetry_log.info(f"Closing existing Meshtastic transport on {transport.port}")
            transport.close()
        except Exception as exc:
            telemetry_log.info(f"Error closing existing transport: {exc}")
        transport = None

    try:
        transport = MeshtasticTransport(port_to_use)
        meshtastic_error = None
        telemetry_log.info(f"Opened Meshtastic node on {port_to_use}")
        return transport
    except Exception as exc:
        meshtastic_error = str(exc)
        telemetry_log.info(f"Unable to open Meshtastic transport on {port_to_use}: {meshtastic_error}")
        return None


def coerce_portnum(portnum):
    """Normalize a Meshtastic port number to an int when possible."""
    if portnum is None:
        return None
    if isinstance(portnum, str):
        portnum = portnum.strip()
        if not portnum:
            return None
        try:
            return int(portnum)
        except (TypeError, ValueError):
            return None
    try:
        return int(portnum)
    except (TypeError, ValueError):
        return None


def extract_raw_payload(decoded, packet=None):
    """Pull the earliest payload-like field out of a Meshtastic decoded packet."""
    if decoded is None:
        return None
    data = decoded.get('data', {}) or {}
    if isinstance(data, dict):
        for key in ('payload', 'text'):
            if key in data and data[key] is not None:
                return data[key]
    for key in ('payload', 'raw', 'data'):
        if key in decoded and decoded[key] is not None:
            return decoded[key]
    if packet is not None and isinstance(packet, dict):
        for key in ('payload', 'raw'):
            if key in packet and packet[key] is not None:
                return packet[key]
    return None


def looks_like_meshlink_payload(raw_payload):
    """Return True when the payload bytes or text can be decoded as a MeshLink packet."""
    if raw_payload is None:
        return False
    if isinstance(raw_payload, str):
        cleaned = raw_payload.strip()
        if cleaned.upper().startswith(('LINK:', 'MESH:')):
            return parse_meshlink_text(cleaned) is not None
        try:
            return parse_meshlink_packet(bytes.fromhex(cleaned)) is not None
        except Exception:
            return False
    return parse_meshlink_packet(raw_payload) is not None


def packet_debug(message, packet=None, **extra):
    """Compatibility stub for packet debug output; intentionally silent."""
    return


def is_app_packet(decoded):
    """Detect whether a decoded Meshtastic packet is a MeshLink app message."""
    if decoded is None:
        return False
    portnum = decoded.get('portnum')
    portnum_value = coerce_portnum(portnum)
    portname = str(portnum).upper() if portnum is not None else ''
    packet_debug('is_app_packet check', decoded=decoded, portnum=portnum, portnum_value=portnum_value, expected_port=MESHLINK_PORTNUM)

    if portnum_value == MESHLINK_PORTNUM:
        return True

    payload = extract_raw_payload(decoded)
    if payload is not None and looks_like_meshlink_payload(payload):
        if portnum_value is None or portname in ('PRIVATE_APP', 'MESHLINK', ''):
            packet_debug('accepted MeshLink payload despite generic/private port label', portnum=portnum, portname=portname)
            return True

    # Only accept a generic PRIVATE_APP packet when the payload actually matches
    # this MeshLink protocol; do not accept all private app traffic indiscriminately.
    if portnums_pb2 is not None and portnum_value == getattr(portnums_pb2.PortNum, 'PRIVATE_APP', -1):
        if payload is not None and looks_like_meshlink_payload(payload):
            packet_debug('PRIVATE_APP payload matches MeshLink packet', portnum=portnum_value)
            return True

    return False


def is_text_message_packet(decoded):
    """Detect whether a decoded Meshtastic packet was received on TEXT_MESSAGE_APP."""
    if decoded is None:
        return False
    portnum = coerce_portnum(decoded.get('portnum'))
    if portnum is None:
        packet_debug('is_text_message_packet: unable to coerce portnum', decoded=decoded)
        return False
    result = (
        portnums_pb2 is not None and
        portnum in (
            portnums_pb2.PortNum.TEXT_MESSAGE_APP,
            portnums_pb2.PortNum.TEXT_MESSAGE_COMPRESSED_APP,
        )
    )
    packet_debug('is_text_message_packet check', decoded=decoded, portnum=portnum, result=result)
    return result


def parse_meshlink_packet(raw_payload):
    """Parse a raw binary MeshLink packet from a Meshtastic payload."""
    if isinstance(raw_payload, memoryview):
        raw_payload = raw_payload.tobytes()
    if not isinstance(raw_payload, (bytes, bytearray)):
        return None
    try:
        return MeshLinkPacket.unpack(bytes(raw_payload))
    except Exception:
        return None


def parse_meshlink_text(text):
    """Backward-compatible parser for legacy hex payloads wrapped in text."""
    if isinstance(text, bytes):
        try:
            text = text.decode('utf-8', errors='replace')
        except Exception:
            return None
    if not isinstance(text, str):
        return None
    stripped = text.strip()
    prefixes = ('LINK:', 'MESH:')
    matched_prefix = None
    for prefix in prefixes:
        if stripped.upper().startswith(prefix):
            matched_prefix = prefix
            break
    if matched_prefix is None:
        return None
    hex_payload = stripped[len(matched_prefix):].strip()
    if not hex_payload:
        return None
    try:
        raw = bytes.fromhex(hex_payload)
    except ValueError:
        return None
    try:
        packet = MeshLinkPacket.unpack(raw)
        return packet
    except Exception:
        return None


def update_vehicle_telemetry(meshlink_packet):
    """Update the local vehicle state from an incoming telemetry packet."""
    try:
        telemetry = parse_telemetry_payload(meshlink_packet.payload)
    except Exception as exc:
        telemetry_log.info(f"Failed to parse telemetry payload: {exc}")
        return

    source = meshlink_packet.header.source
    if source == int(VehicleID.GCS):
        return

    vehicle = vehicles.setdefault(source, {
        'id': source,
        'lat': 0.0,
        'lon': 0.0,
        'battery_cV': 0,
        'altitude_m': 0,
        'flight_path': [],
        'status': 'TELEMETRY',
    })
    latitude = telemetry['latitude']
    longitude = telemetry['longitude']
    new_position = [latitude, longitude]
    flight_path = vehicle.setdefault('flight_path', [])
    if (latitude, longitude) != (0.0, 0.0) and (
        not flight_path or flight_path[-1] != new_position
    ):
        flight_path.append(new_position)
    vehicle.update({
        'id': source,
        'lat': latitude,
        'lon': longitude,
        'altitude_m': telemetry['altitude_m'],
        'battery_cV': telemetry['battery_cV'],
        'groundspeed_cms': telemetry['groundspeed_cms'],
        'flight_mode': telemetry['flight_mode'],
        'current_waypoint': telemetry['current_waypoint'],
        'rssi': telemetry['rssi'],
        'heading_step': telemetry['heading_step'],
        'heading_deg': telemetry['heading_deg'],
        'status': 'TELEMETRY',
        'last_update': datetime.now().timestamp(),
    })


def summarize_meshtastic_packet(packet):
    """Create a one-line summary from a Meshtastic receive packet.

    The Meshtastic packet can carry decoded text, decoded payload bytes, or
    raw fields. This summary is used by the UI log to show incoming traffic.
    """
    if not isinstance(packet, dict):
        packet_debug('summarize_meshtastic_packet: non-dict packet', packet=packet)
        return str(packet)
    decoded = packet.get('decoded')
    packet_debug('summarize_meshtastic_packet start', packet=packet, decoded_type=type(decoded).__name__ if decoded is not None else None)
    if not decoded:
        packet_debug('summarize_meshtastic_packet: no decoded payload', packet=packet)
        return ''

    def format_value(value):
        if isinstance(value, (bytes, bytearray)):
            return value.hex()
        if isinstance(value, dict):
            try:
                return json.dumps(value, default=str)
            except Exception:
                return str(value)
        return str(value)

    data = decoded.get('data', {}) or {}
    text = data.get('text')
    payload = data.get('payload')
    packet_debug('packet decode fields', packet=packet, portnum=decoded.get('portnum'), data_type=type(data).__name__, text_type=type(text).__name__ if text is not None else None, payload_type=type(payload).__name__ if payload is not None else None)

    if isinstance(text, bytes):
        try:
            text = text.decode('utf-8', errors='replace')
        except Exception:
            text = text.hex()

    raw_payload = payload or data.get('payload') or decoded.get('payload') or packet.get('payload')
    packet_debug('raw payload inspection', raw_payload_type=type(raw_payload).__name__ if raw_payload is not None else None, raw_payload_preview=str(raw_payload)[:200] if raw_payload is not None else None)
    meshlink_packet = parse_meshlink_packet(raw_payload) if raw_payload is not None else None
    if meshlink_packet is None and text is not None:
        packet_debug('trying legacy text hex decode', text=text[:200])
        meshlink_packet = parse_meshlink_text(text)
    if meshlink_packet is None:
        packet_debug('MeshLink parse failed: not a valid MeshLink packet', raw_payload_type=type(raw_payload).__name__ if raw_payload is not None else None, text_preview=text[:200] if isinstance(text, str) else None)
    else:
        packet_debug('MeshLink parse succeeded', header=meshlink_packet.header, payload_len=len(meshlink_packet.payload), payload_preview=meshlink_packet.payload.hex()[:200])

    if meshlink_packet is not None:
        if meshlink_packet.header.type == MessageType.TELEMETRY:
            update_vehicle_telemetry(meshlink_packet)

            try:
                telemetry = parse_telemetry_payload(meshlink_packet.payload)
                heading_deg = telemetry['heading_deg']
                return (
                    f"telemetry src={meshlink_packet.header.source} "
                    f"flight_mode={telemetry['flight_mode']} wp={telemetry['current_waypoint']} "
                    f"lat={telemetry['latitude']:.5f} lon={telemetry['longitude']:.5f} "
                    f"alt={telemetry['altitude_m']}m gs={telemetry['groundspeed_cms']/100:.1f}m/s "
                    f"bat={telemetry['battery_cV']/100:.2f}V "
                    f"rssi={telemetry['rssi']}dBm heading={heading_deg:.1f}°"
                )
            except Exception:
                pass
        return (
            f"meshlink type={MessageType(meshlink_packet.header.type).name} "
            f"src={meshlink_packet.header.source} dst={meshlink_packet.header.destination} "
            f"seq={meshlink_packet.header.sequence} part={meshlink_packet.header.part}/{meshlink_packet.header.total_parts} "
            f"len={len(meshlink_packet.payload)} "
            f"payload={meshlink_packet.payload.hex()}"
        )

    # Prefer decoded text/payload from the Meshtastic message.
    if text is None and payload is None:
        payload = decoded.get('payload') or decoded.get('raw') or packet.get('payload')

    if text is None and payload is None:
        return ''

    parts = []
    if packet.get('from') is not None:
        parts.append(f"from={packet['from']}")
    if packet.get('to') is not None:
        parts.append(f"to={packet['to']}")
    if decoded.get('portnum') is not None:
        parts.append(f"port={decoded['portnum']}")
    if decoded.get('type') is not None:
        parts.append(f"type={decoded['type']}")
    if decoded.get('channelIndex') is not None:
        parts.append(f"channel={decoded['channelIndex']}")

    if text is not None:
        parts.append(f"text={text}")
    if payload is not None:
        formatted = format_value(payload)
        if len(formatted) > 200:
            formatted = formatted[:200] + '...'
        parts.append(f"payload={formatted}")

    return ' '.join(parts)


def add_meshtastic_message(packet, topic='meshtastic.receive'):
    """Store Meshtastic events in the incoming message log.

    This function keeps only the most recent messages in memory and ignores
    non-text receive traffic for this UI log.
    """
    if topic != 'meshtastic.receive':
        return

    packet_debug('received mesh packet callback', packet=packet)
    decoded = packet.get('decoded') if isinstance(packet, dict) else None
    if decoded is None:
        packet_debug('dropping packet: no decoded payload', packet=packet)
        return
    packet_debug('incoming decoded packet', decoded=decoded, portnum=decoded.get('portnum'))
    if not is_app_packet(decoded):
        packet_debug('dropping packet: not MeshLink app port', decoded=decoded, portnum=decoded.get('portnum'))
        return

    text = summarize_meshtastic_packet(packet)
    if not text:
        packet_debug('dropping packet: summarize_meshtastic_packet returned empty', packet=packet)
        return

    telemetry_log.info(f"TM Received: {text}")

    incoming_messages.append({
        'id': len(incoming_messages) + 1,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'topic': topic,
        'text': text,
    })
    while len(incoming_messages) > MAX_INCOMING_MESSAGES:
        incoming_messages.pop(0)


def send_heartbeat():
    """Broadcast a MeshLink heartbeat to the mesh."""
    if transport is None:
        return
    try:
        send_packet(int(VehicleID.BROADCAST), MessageType.HEARTBEAT, b'')
        incoming_messages.append({
            'id': len(incoming_messages) + 1,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'topic': 'meshlink.heartbeat',
            'text': f"TX HEARTBEAT interval={HEARTBEAT_INTERVAL}s",
        })
        while len(incoming_messages) > MAX_INCOMING_MESSAGES:
            incoming_messages.pop(0)
        telemetry_log.info(f"TX HEARTBEAT interval={HEARTBEAT_INTERVAL}s")
    except Exception as exc:
        telemetry_log.info(f"Failed to send MeshLink heartbeat: {exc}")


def heartbeat_loop():
    """Background loop that periodically sends a heartbeat packet."""
    while heartbeat_stop is not None and not heartbeat_stop.is_set():
        send_heartbeat()
        heartbeat_stop.wait(HEARTBEAT_INTERVAL)


def start_heartbeat_loop():
    """Start a background heartbeat thread if one is not already running."""
    global heartbeat_stop, heartbeat_thread
    if heartbeat_thread is not None and heartbeat_thread.is_alive():
        return
    heartbeat_stop = threading.Event()
    heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    heartbeat_thread.start()


def stop_heartbeat_loop():
    """Stop the heartbeat thread."""
    global heartbeat_stop, heartbeat_thread
    if heartbeat_stop is not None:
        heartbeat_stop.set()
        heartbeat_stop = None
    heartbeat_thread = None


def on_meshtastic_receive(packet, interface=None):
    """Meshtastic pubsub callback for received packets."""
    add_meshtastic_message(packet, topic='meshtastic.receive')


def on_connection_established(interface=None, topic=pub.AUTO_TOPIC):
    """Meshtastic pubsub callback when the radio connects."""
    global meshtastic_error
    telemetry_log.info('Meshtastic connection established')
    meshtastic_error = None
    start_heartbeat_loop()


def on_connection_lost(interface=None, topic=pub.AUTO_TOPIC):
    """Meshtastic pubsub callback when the radio disconnects."""
    global transport, meshtastic_error
    telemetry_log.info('Meshtastic connection lost')
    meshtastic_error = 'Meshtastic connection lost'
    stop_heartbeat_loop()
    if transport is not None:
        try:
            transport.close()
        except Exception:
            pass
        transport = None


def list_ports():
    """Return available serial ports for Meshtastic connection selection."""
    ports = []
    if serial_list_ports is not None:
        try:
            for port in serial_list_ports.comports():
                ports.append({
                    'device': port.device,
                    'description': port.description or '',
                    'hwid': port.hwid or '',
                })
            return ports
        except Exception as exc:
            telemetry_log.info(f'Error listing COM ports: {exc}')
    if findPorts is not None:
        try:
            for device in findPorts(True):
                ports.append({'device': device, 'description': '', 'hwid': ''})
        except Exception as exc:
            telemetry_log.info(f'Error listing Meshtastic ports: {exc}')
    return ports


def get_connection_state():
    """Return current Meshtastic connection status for the frontend."""
    return {
        'meshtastic_connected': transport is not None,
        'meshtastic_port': selected_port,
        'meshtastic_library': SerialInterface is not None,
        'meshtastic_error': meshtastic_error,
        'selected_port': selected_port,
    }


def next_sequence():
    global sequence
    sequence = (sequence + 1) & 0xFF
    return sequence


def send_packet(destination, type_, payload, total_parts=1, part=1):
    """Build and send a MeshLink packet using Meshtastic text messages.

    The MeshLink packet still uses the same binary header and payload.
    Instead of raw binary transport, it is converted to hex ASCII and sent
    through Meshtastic sendText() so the receiving side can decode it from
    the text channel.
    """
    if transport is None:
        raise RuntimeError('Meshtastic transport is not connected')

    header_destination = int(VehicleID.BROADCAST)
    try:
        header_destination = int(destination)
    except (TypeError, ValueError):
        header_destination = int(VehicleID.BROADCAST)

    # All Meshtastic packets are sent as broadcast text messages.
    transport_destination = '^all'

    header = PacketHeader(
        version=PROTOCOL_VERSION,
        source=int(VehicleID.GCS),
        destination=header_destination,
        type=int(type_),
        sequence=next_sequence(),
        total_parts=total_parts,
        part=part,
        payload_length=len(payload),
        crc16=0,
    )
    packet = MeshLinkPacket(header, payload)
    text_payload = MESHLINK_TEXT_PREFIX + packet.pack().hex()
    meshlink_bytes = packet.pack()
    telemetry_log.info(f"attempting to send MeshLink binary payload on port {MESHLINK_PORTNUM}: {meshlink_bytes.hex()}")
    try:
        transport.write(meshlink_bytes, transport_destination)
    except Exception as exc:
        telemetry_log.info(f"Failed to send MeshLink binary packet via Meshtastic: {exc}")
        raise
    telemetry_log.info(f"TX MESHLINK port={MESHLINK_PORTNUM} dst={transport_destination} seq={header.sequence} part={part}/{total_parts} len={len(meshlink_bytes)}")


def send_text_message(text, destination=None):
    """Send a simple Meshtastic text message to the default channel.

    Text messages use channel index 0 and are always broadcast to all nodes.
    """
    if transport is None:
        raise RuntimeError('Meshtastic transport is not connected')
    try:
        transport.send_text(text, '^all')
    except Exception as exc:
        telemetry_log.info(f"Failed to send text via Meshtastic: {exc}")
        raise
    telemetry_log.info(f"TX TEXT dst=^all len={len(text)}")


@app.route('/')
def index():
    """Serve the main ground station web UI."""
    return render_template('index.html')


@app.route('/api/state')
def state():
    # for vehicle in vehicles:
    #     if (vehicle.last_update-datetime.now().timestamp() > 180):
    #        vehicle.update({'status':'STALE'})

    return jsonify({
        'active_vehicle': int(active_vehicle),
        'vehicles': vehicles,
        'mission_waypoints': mission_waypoints,
        'meshtastic': get_connection_state(),
    })


@app.route('/api/ports')
def api_ports():
    """API endpoint for the available COM ports and selected port."""
    return jsonify({
        'ports': list_ports(),
        'selected_port': selected_port,
    })


@app.route('/api/messages')
def api_messages():
    return jsonify({
        'messages': incoming_messages,
    })


@app.route('/api/reconnect', methods=['POST'])
def api_reconnect():
    """Reconnect the Meshtastic transport to the selected port."""
    global transport
    if transport is not None:
        try:
            transport.close()
        except Exception:
            pass
        transport = None
    data = request.json or {}
    port = data.get('port')
    transport = connect_serial(port)
    return jsonify(get_connection_state())


@app.route('/api/command', methods=['POST'])
def api_command():
    """Send a MeshLink COMMAND packet to the selected vehicle or all vehicles."""
    data = request.json or {}
    command_id = data.get('command_id')
    destination = data.get('destination', int(active_vehicle))
    try:
        payload = build_command_payload(command_id, 0) # type: ignore
        send_packet(destination, MessageType.COMMAND, payload)
        return jsonify({'status': 'ok'})
    except Exception as exc:
        return jsonify({'status': 'error', 'message': str(exc)}), 500


@app.route('/api/send_text', methods=['POST'])
def api_send_text():
    """Send a text message through Meshtastic for debugging purposes."""
    data = request.json or {}
    text = data.get('text', '')
    destination = data.get('destination')
    if not text:
        return jsonify({'status': 'error', 'message': 'Text cannot be empty'}), 400
    try:
        send_text_message(text, destination)
        return jsonify({'status': 'ok'})
    except Exception as exc:
        return jsonify({'status': 'error', 'message': str(exc)}), 500


@app.route('/api/guided_loiter', methods=['POST'])
def api_guided_loiter():
    """Send a GUIDED_LOITER command to the selected vehicle or all vehicles."""
    data = request.json or {}
    destination = data.get('destination', int(active_vehicle))
    try:
        lat = int(data['latitude'] * 1e7)
        lon = int(data['longitude'] * 1e7)
        alt = int(data.get('altitude_m', 100))
        radius = int(data.get('radius_m', 50))
        payload = build_guided_loiter_payload(lat, lon, alt, radius)
        send_packet(destination, MessageType.GUIDED_LOITER, payload)
        return jsonify({'status': 'ok'})
    except Exception as exc:
        return jsonify({'status': 'error', 'message': str(exc)}), 500


@app.route('/api/mission', methods=['POST'])
def api_mission():
    data = request.json or {}
    destination = data.get('destination', int(active_vehicle))
    if isinstance(destination, str) and destination.lower() in {'all', 'broadcast'}:
        return jsonify({'status': 'error', 'message': 'Broadcast mission upload is not allowed'}), 400
    waypoints = [
        (int(round(w['latitude'] * 1e7)), int(round(w['longitude'] * 1e7)), int(w.get('altitude_m', 100)), int(w.get('waypoint_type', 0)))
        for w in data['waypoints']
    ]
    total_waypoints = len(waypoints)
    fragments = []
    for first_idx in range(0, total_waypoints, 4):
        chunk = waypoints[first_idx:first_idx + 4]
        fragments.append(build_mission_upload_fragment(total_waypoints, first_idx, chunk))
    for part, payload in enumerate(fragments, start=1):
        send_packet(destination, MessageType.MISSION_UPLOAD, payload, total_parts=len(fragments), part=part+1)
    mission_waypoints.clear()
    mission_waypoints.extend([
        {
            'latitude': w[0] / 1e7,
            'longitude': w[1] / 1e7,
            'altitude_m': w[2],
            'waypoint_type': w[3],
        }
        for w in waypoints
    ])
    return jsonify({'status': 'ok', 'fragments': len(fragments)})


if __name__ == '__main__':
    transport = connect_serial()
    start_heartbeat_loop()
    try:
        app.run(host='127.0.0.1', port=5000, debug=True, use_reloader=False)
    finally:
        stop_heartbeat_loop()
