import os
import json
from datetime import datetime

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
    PacketHeader,
    MeshLinkPacket,
    PROTOCOL_VERSION,
)

app = Flask(__name__, template_folder='templates', static_folder='static')
CORS(app)

# Default serial port and baud rate for the local Meshtastic radio.
# These can be overridden with environment variables if needed.
SERIAL_PORT = os.getenv('MESHTASTIC_PORT', 'COM3')
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

# Current MeshLink version for this text-over-Meshtastic scheme.
MESH_LINK_VERSION = '3'

# Prefix applied to text messages carrying MeshLink hex packets.
MESHLINK_TEXT_PREFIX = 'LINK:'
vehicles = {
    1: {
        'id': 1,
        'lat': 34.7535,
        'lon': -118.3442,
        'status': 'OK',
        'battery_cV': 1150,
        'altitude_m': 120,
    },
    2: {
        'id': 2,
        'lat': 34.7540,
        'lon': -118.3540,
        'status': 'OK',
        'battery_cV': 1203,
        'altitude_m': 100,
    }
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
        # Send a raw Meshtastic application payload.
        #
        # SerialInterface maintains the underlying pyserial stream at
        # self.iface.stream, and sendData() handles packet framing,
        # packet IDs, acks, and dispatching through the Meshtastic link.
        # Directly writing bytes to the raw serial stream would bypass the
        # Meshtastic protocol, so use sendData() for app-layer payloads.
        self.iface.sendData(
            data,
            destinationId=destination,
            portNum=256, # type: ignore
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
        print(meshtastic_error)
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
            print(f'Failed to subscribe to Meshtastic events: {exc}')

    port_to_use = selected_port
    if transport is not None:
        if transport.port == port_to_use:
            print(f"Meshtastic transport already open on {port_to_use}")
            return transport
        try:
            print(f"Closing existing Meshtastic transport on {transport.port}")
            transport.close()
        except Exception as exc:
            print(f"Error closing existing transport: {exc}")
        transport = None

    try:
        transport = MeshtasticTransport(port_to_use)
        meshtastic_error = None
        print(f"Opened Meshtastic node on {port_to_use}")
        return transport
    except Exception as exc:
        meshtastic_error = str(exc)
        print(f"Unable to open Meshtastic transport on {port_to_use}: {meshtastic_error}")
        return None


def is_app_packet(decoded):
    """Detect whether a decoded Meshtastic packet is application-layer traffic."""
    portnum = decoded.get('portnum')
    if portnum is None:
        return False
    if isinstance(portnum, int):
        return portnum == 256
    portname = str(portnum).upper()
    excluded = ['TELEMETRY', 'POSITION', 'BEACON', 'NODEINFO', 'LINK', 'ACK', 'NAK', 'ROUTE']
    return not any(token in portname for token in excluded)


def is_text_message_packet(decoded):
    """Detect whether a decoded Meshtastic packet was received on TEXT_MESSAGE_APP."""
    if decoded is None:
        return False
    portnum = decoded.get('portnum')
    if portnum is None:
        return False
    if isinstance(portnum, int):
        return (
            portnums_pb2 is not None and
            portnum in (
                portnums_pb2.PortNum.TEXT_MESSAGE_APP,
                portnums_pb2.PortNum.TEXT_MESSAGE_COMPRESSED_APP,
            )
        )
    portname = str(portnum).upper()
    return portname in ('TEXT_MESSAGE_APP', 'TEXT_MESSAGE_COMPRESSED_APP')


def parse_meshlink_text(text):
    """Detect and parse a MeshLink hex packet embedded in a text message.

    When using MeshLink-over-text, the binary packet is encoded as hex ASCII
    and prefixed with a constant string so it can be distinguished from
    ordinary plain-text chat messages.
    """
    if isinstance(text, bytes):
        try:
            text = text.decode('utf-8', errors='replace')
        except Exception:
            return None
    if not isinstance(text, str):
        return None
    stripped = text.strip()
    if not stripped.upper().startswith(MESHLINK_TEXT_PREFIX):
        return None
    hex_payload = stripped[len(MESHLINK_TEXT_PREFIX):].strip()
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


def summarize_meshtastic_packet(packet):
    """Create a one-line summary from a Meshtastic receive packet.

    The Meshtastic packet can carry decoded text, decoded payload bytes, or
    raw fields. This summary is used by the UI log to show incoming traffic.
    """
    if not isinstance(packet, dict):
        return str(packet)
    decoded = packet.get('decoded')
    if not decoded:
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

    if isinstance(text, bytes):
        try:
            text = text.decode('utf-8', errors='replace')
        except Exception:
            text = text.hex()

    if text is None and is_text_message_packet(decoded):
        raw_payload = payload or decoded.get('payload') or packet.get('payload')
        if isinstance(raw_payload, (bytes, bytearray)):
            try:
                text = raw_payload.decode('utf-8', errors='replace')
            except Exception:
                text = raw_payload.hex()
        elif raw_payload is not None:
            text = str(raw_payload)

    meshlink_packet = parse_meshlink_text(text) if text is not None else None
    if meshlink_packet is not None:
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

    decoded = packet.get('decoded') if isinstance(packet, dict) else None
    if not is_text_message_packet(decoded):
        return

    text = summarize_meshtastic_packet(packet)
    if not text:
        return

    incoming_messages.append({
        'id': len(incoming_messages) + 1,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'topic': topic,
        'text': text,
    })
    while len(incoming_messages) > MAX_INCOMING_MESSAGES:
        incoming_messages.pop(0)


def on_meshtastic_receive(packet, interface=None):
    """Meshtastic pubsub callback for received packets."""
    add_meshtastic_message(packet, topic='meshtastic.receive')


def on_connection_established(interface=None, topic=pub.AUTO_TOPIC):
    """Meshtastic pubsub callback when the radio connects."""
    global meshtastic_error
    print('Meshtastic connection established')
    meshtastic_error = None


def on_connection_lost(interface=None, topic=pub.AUTO_TOPIC):
    """Meshtastic pubsub callback when the radio disconnects."""
    global transport, meshtastic_error
    print('Meshtastic connection lost')
    meshtastic_error = 'Meshtastic connection lost'
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
            print(f'Error listing COM ports: {exc}')
    if findPorts is not None:
        try:
            for device in findPorts(True):
                ports.append({'device': device, 'description': '', 'hwid': ''})
        except Exception as exc:
            print(f'Error listing Meshtastic ports: {exc}')
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


def send_packet(destination, type_, payload, total_parts=1, part=0):
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
    print("attempting to send text payload:", text_payload)
    try:
        transport.send_text(text_payload, transport_destination)
    except Exception as exc:
        print(f"Failed to send MeshLink text packet via Meshtastic: {exc}")
        raise
    print(f"TX TEXT-MESHLINK {type_.name} dst={transport_destination} seq={header.sequence} part={part}/{total_parts}")


def send_text_message(text, destination=None):
    """Send a simple Meshtastic text message to the default channel.

    Text messages use channel index 0 and are always broadcast to all nodes.
    """
    if transport is None:
        raise RuntimeError('Meshtastic transport is not connected')
    try:
        transport.send_text(text, '^all')
    except Exception as exc:
        print(f"Failed to send text via Meshtastic: {exc}")
        raise
    print(f"TX TEXT dst=^all len={len(text)}")


@app.route('/')
def index():
    """Serve the main ground station web UI."""
    return render_template('index.html')


@app.route('/api/state')
def state():
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
    """Send a MeshLink COMMAND packet to the selected vehicle."""
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
    """Send a GUIDED_LOITER command to the selected vehicle."""
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
    data = request.json
    destination = data.get('destination', int(active_vehicle))
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
        send_packet(destination, MessageType.MISSION_UPLOAD, payload, total_parts=len(fragments), part=part)
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
    app.run(host='127.0.0.1', port=5000, debug=True, use_reloader=False)
