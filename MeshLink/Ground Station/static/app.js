/*
  MeshLink Ground Station frontend.
  - Displays vehicle positions on a Leaflet map.
  - Shows available Meshtastic COM ports and connection status.
  - Sends commands, guided loiter requests, mission uploads, and text messages.
  - Polls backend state and incoming Meshtastic messages.
*/
let map;
let markers = {};
let vehicleMarkers = {};
let vehiclePaths = {};
let vehicleLines = {};
let waypointMarkers = [];
let waypointLine = null;
let waypoints = [];
let activeMode = 'waypoint';
let activeVehicleId = null;
let lastIncomingMessageId = 0;

function createVehicleIcon(id) {
    const colors = ['#ff4757', '#1e90ff', '#2ed573', '#ffa502', '#eccc68', '#ff6b81'];
    const color = colors[(id - 1) % colors.length];
    return L.divIcon({
        className: 'vehicle-div-icon',
        html: `<div class="vehicle-icon" style="background:${color};">${id}</div>`,
        iconSize: [32, 32],
        iconAnchor: [16, 16],
    });
}

function getVehicleColor(id) {
    const colors = ['#ff4757', '#1e90ff', '#2ed573', '#ffa502', '#eccc68', '#ff6b81'];
    return colors[(parseInt(id, 10) - 1) % colors.length];
}

const waypointIcon = L.icon({
    iconUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
});

function init() {
    // Initialize Leaflet map centered over the default area.
    map = L.map('map').setView([35.0456, -118.1838], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);

    // Use click interactions to add waypoints or send loiter commands.
    map.on('click', onMapClick);

    // Track the selected interaction mode from the radio buttons.
    document.getElementsByName('mode').forEach(r => r.addEventListener('change', () => {
        activeMode = r.value;
        log('Mode set to ' + activeMode);
    }));

    // Load initial UI state from the backend.
    fetchPorts();
    fetchState();
    fetchMessages();

    // Poll state and message updates periodically.
    setInterval(() => {
        fetchState();
        fetchMessages();
    }, 2000);
}

function updateConnectionStatus(status) {
    const el = document.getElementById('connection-status');
    if (!el) return;

    if (status && status.meshtastic_connected) {
        el.textContent = `Meshtastic: connected (${status.meshtastic_port})`;
        el.classList.add('connected');
        el.classList.remove('disconnected', 'warning');
    } else if (status && !status.meshtastic_library) {
        el.textContent = 'Meshtastic: library missing';
        el.title = status.meshtastic_error || '';
        el.classList.add('warning');
        el.classList.remove('connected', 'disconnected');
    } else if (status && status.meshtastic_error) {
        const errorText = status.meshtastic_error;
        el.textContent = `Meshtastic: disconnected (${status.meshtastic_port}) — ${errorText}`;
        el.title = errorText;
        el.classList.add('warning');
        el.classList.remove('connected', 'disconnected');
    } else {
        const port = status ? status.meshtastic_port : 'unknown';
        el.textContent = `Meshtastic: disconnected (${port})`;
        el.title = '';
        el.classList.add('disconnected');
        el.classList.remove('connected', 'warning');
    }
}

function reconnectMeshtastic() {
    // Reconnect the backend to the selected COM port.
    const portSelect = document.getElementById('port-select');
    const port = portSelect ? portSelect.value : null;
    fetch('/api/reconnect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port }),
    })
        .then(r => r.json())
        .then(status => {
            updateConnectionStatus(status);
            if (status.meshtastic_connected) {
                log(`Meshtastic connected on ${status.meshtastic_port}`);
            } else if (status.meshtastic_error) {
                log(`Meshtastic connect failed: ${status.meshtastic_error}`);
            } else {
                log('Meshtastic connect failed');
            }
            fetchPorts();
        })
        .catch(err => {
            log('Meshtastic reconnect request failed: ' + err);
        });
}

function fetchPorts() {
    // Retrieve the list of available serial ports from the backend.
    fetch('/api/ports')
        .then(r => r.json())
        .then(data => {
            updatePortList(data);
        })
        .catch(err => {
            log('Failed to load COM port list: ' + err);
        });
}

function updatePortList(data) {
    const select = document.getElementById('port-select');
    if (!select) return;
    const ports = data.ports || [];
    select.innerHTML = '';
    ports.forEach(port => {
        const option = document.createElement('option');
        option.value = port.device;
        option.textContent = `${port.device}${port.description ? ' — ' + port.description : ''}`;
        select.appendChild(option);
    });
    if (data.selected_port && ports.some(p => p.device === data.selected_port)) {
        select.value = data.selected_port;
    } else if (ports.length > 0) {
        select.value = ports[0].device;
    }
}

function fetchMessages() {
    // Poll backend for new received Meshtastic messages and append them to the log.
    fetch('/api/messages')
        .then(r => r.json())
        .then(data => {
            if (data && Array.isArray(data.messages)) {
                data.messages.forEach(msg => {
                    if (msg.id > lastIncomingMessageId) {
                        log(`[${msg.timestamp}] ${msg.topic}: ${msg.text}`);
                        lastIncomingMessageId = msg.id;
                    }
                });
            }
        })
        .catch(err => {
            log('Failed to fetch incoming messages: ' + err);
        });
}

function fetchState() {
    // Get the current application state from the backend, including vehicle and connection info.
    fetch('/api/state')
        .then(r => r.json())
        .then(data => {
            if (activeVehicleId === null) {
                activeVehicleId = data.active_vehicle || Object.keys(data.vehicles)[0];
            }
            updateVehicles(data.vehicles);
            updateVehicleList(data.vehicles);
            if (waypoints.length === 0 && data.mission_waypoints && data.mission_waypoints.length) {
                updateWaypoints(data.mission_waypoints);
            }
            updateVehicleInfo(data.vehicles, activeVehicleId);
            updateConnectionStatus(data.meshtastic);
        });
}

function updateVehicles(vehicleData) {
    Object.keys(vehicleData).forEach(id => {
        const v = vehicleData[id];
        const groundspeed = v.groundspeed_cms != null ? (v.groundspeed_cms / 100).toFixed(1) : 'N/A';
        const rssi = v.rssi != null ? `${v.rssi} dBm` : 'N/A';
        const linkQuality = v.link_quality != null ? `${v.link_quality}%` : 'N/A';
        const markerText = `Vehicle ${id}\nBattery: ${v.battery_cV / 100} V\nAltitude: ${v.altitude_m} m\nGroundspeed: ${groundspeed} m/s\nRSSI: ${rssi}\nLink: ${linkQuality}\nStatus: ${v.status}`;
            const coords = [v.lat, v.lon];
        if (Array.isArray(v.flight_path) && v.flight_path.length > 0) {
            vehiclePaths[id] = v.flight_path.map(point => [point[0], point[1]]);
        } else if (!vehiclePaths[id]) {
            vehiclePaths[id] = [];
        }

        if (vehicleMarkers[id]) {
            vehicleMarkers[id].setLatLng(coords);
            vehicleMarkers[id].setPopupContent(markerText);
            vehicleMarkers[id].setIcon(createVehicleIcon(parseInt(id, 10)));
        } else {
            vehicleMarkers[id] = L.marker(coords, { icon: createVehicleIcon(parseInt(id, 10)) })
                .addTo(map)
                .bindPopup(markerText);
        }

        const path = vehiclePaths[id];
        if (path.length === 0 || path[path.length - 1][0] !== coords[0] || path[path.length - 1][1] !== coords[1]) {
            path.push(coords);
        }
        if (path.length > 1) {
            if (vehicleLines[id]) {
                vehicleLines[id].setLatLngs(path);
            } else {
                vehicleLines[id] = L.polyline(path, { color: getVehicleColor(id), weight: 3, opacity: 0.8 }).addTo(map);
            }
        }
    });
}

function updateVehicleInfo(vehicleData, activeId) {
    const info = vehicleData[activeId];
    const container = document.getElementById('vehicle-info');
    if (!info) {
        container.textContent = 'No active vehicle data';
        return;
    }
    const groundspeed = info.groundspeed_cms != null ? (info.groundspeed_cms / 100).toFixed(1) : 'N/A';
    const rssi = info.rssi != null ? `${info.rssi} dBm` : 'N/A';
    const linkQuality = info.link_quality != null ? `${info.link_quality}%` : 'N/A';
    container.innerHTML = `
        <div class="vehicle-summary">
            <div class="vehicle-summary-title">Vehicle ${activeId}</div>
            <div>Status: ${info.status}</div>
            <div>Flight Mode: ${info.flight_mode}</div>
            <div>Battery: ${info.battery_cV / 100} V</div>
            <div>Altitude: ${info.altitude_m} m</div>
            <div>Groundspeed: ${groundspeed} m/s</div>
            <div>RSSI: ${rssi}</div>
            <div>Lat: ${info.lat.toFixed(5)}</div>
            <div>Lon: ${info.lon.toFixed(5)}</div>
            <div>Last Update: ${((Date.now()/1000 - info.last_update)).toFixed(0)} s ago</div>
        </div>
    `;
}

function updateVehicleList(vehicleData) {
    const list = document.getElementById('vehicle-list');
    list.innerHTML = '';
    Object.keys(vehicleData).forEach(id => {
        const v = vehicleData[id];
        const item = document.createElement('div');
        item.className = `vehicle-item ${id == activeVehicleId ? 'active' : ''}`;
        item.innerHTML = `
            <div class="vehicle-item-id">V${id}</div>
            <div class="vehicle-item-body">
                <div>Battery: ${(v.battery_cV / 100).toFixed(2)} V</div>
                <div>Alt: ${v.altitude_m} m</div>
                <div>Status: ${v.status}</div>
            </div>
        `;
        item.addEventListener('click', () => {
            activeVehicleId = id;
            updateVehicleInfo(vehicleData, id);
            updateVehicleList(vehicleData);
        });
        list.appendChild(item);
    });
}

function updateWaypoints(list) {
    waypoints = list;
    const container = document.getElementById('waypoint-list');
    container.innerHTML = '';
    clearWaypointMarkers();

    const pathCoords = [];
    list.forEach((wp, idx) => {
        const lat = wp.latitude;
        const lon = wp.longitude;
        const item = document.createElement('div');
        item.textContent = `WP${idx + 1}: ${lat.toFixed(5)}, ${lon.toFixed(5)}`;
        container.appendChild(item);
        const marker = L.marker([lat, lon], { icon: waypointIcon }).addTo(map);
        waypointMarkers.push(marker);
        pathCoords.push([lat, lon]);
    });

    if (waypointLine) {
        map.removeLayer(waypointLine);
        waypointLine = null;
    }
    if (pathCoords.length > 1) {
        waypointLine = L.polyline(pathCoords, { color: '#fbbc04', weight: 3 }).addTo(map);
    }
}

function clearWaypointMarkers() {
    waypointMarkers.forEach(marker => map.removeLayer(marker));
    waypointMarkers = [];
    if (waypointLine) {
        map.removeLayer(waypointLine);
        waypointLine = null;
    }
}

function log(message) {
    const logArea = document.getElementById('log');
    logArea.value += message + '\n';
    logArea.scrollTop = logArea.scrollHeight;
}

function onMapClick(event) {
    // Handle map clicks differently depending on the selected mode.
    const lat = event.latlng.lat;
    const lon = event.latlng.lng;
    if (activeMode === 'waypoint') {
        addWaypoint(lat, lon);
    } else {
        guidedLoiter(lat, lon);
    }
}

function addWaypoint(lat, lon) {
    // Add a mission waypoint locally and update the map.
    waypoints.push({ latitude: lat, longitude: lon, altitude_m: 100, waypoint_type: 0 });
    updateWaypoints(waypoints);
    log(`Waypoint added: ${lat.toFixed(5)}, ${lon.toFixed(5)}`);
}

function guidedLoiter(lat, lon) {
    // Ask the user to confirm and send a guided loiter command via backend API.
    const confirmed = confirm(`Send GUIDED_LOITER to ${lat.toFixed(5)}, ${lon.toFixed(5)}?`);
    if (!confirmed) return;
    fetch('/api/guided_loiter', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ latitude: lat, longitude: lon, altitude_m: 100, radius_m: 50 })
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            if (status >= 200 && status < 300 && data.status === 'ok') {
                log(`Guided loiter sent: ${lat.toFixed(5)}, ${lon.toFixed(5)}`);
            } else {
                log(`Guided loiter failed: ${data.message || JSON.stringify(data)}`);
            }
        })
        .catch(err => log(`Guided loiter request failed: ${err}`));
}

function uploadMission() {
    if (waypoints.length === 0) {
        alert('No waypoints to upload.');
        return;
    }
    const confirmed = confirm(`Upload mission with ${waypoints.length} waypoint(s)?`);
    if (!confirmed) return;
    fetch('/api/mission', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ waypoints })
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            if (status >= 200 && status < 300 && data.status === 'ok') {
                log(`Mission uploaded with ${data.fragments} fragments`);
            } else {
                log(`Mission upload failed: ${data.message || JSON.stringify(data)}`);
            }
        })
        .catch(err => log(`Mission request failed: ${err}`));
}

function clearMission() {
    const confirmed = confirm('Clear all waypoints from the map?');
    if (!confirmed) return;
    waypoints = [];
    updateWaypoints(waypoints);
    log('Mission cleared');
}

function sendCommand(command_id) {
    const labels = {
        1: 'RTL',
        2: 'LOITER',
        3: 'AUTO',
        4: 'REQUEST STATUS',
        5: 'REQUEST TELEMETRY',
        6: 'REBOOT BRIDGE'
    };
    const confirmed = confirm(`Send command ${labels[command_id]} to vehicle ${activeVehicleId}?`);
    if (!confirmed) return;
    const destination = activeVehicleId;
    fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command_id, destination })
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            if (status >= 200 && status < 300 && data.status === 'ok') {
                log(`Command ${labels[command_id]} sent`);
            } else {
                log(`Command failed: ${data.message || JSON.stringify(data)}`);
            }
        })
        .catch(err => log(`Command request failed: ${err}`));
}

function sendTestText() {
    // Prompt the user for text and send it over the Meshtastic link.
    const text = prompt('Enter test text to send over Meshtastic:', 'Hello from GCS');
    if (!text) return;
    fetch('/api/send_text', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text })
})
        .then(r => r.json())
        .then(data => {
            if (data.status === 'ok') {
                log(`Test text sent: ${text}`);
            } else {
                log(`Test text failed: ${data.message || JSON.stringify(data)}`);
            }
        })
        .catch(err => log(`Failed to send test text: ${err}`));
}

window.addEventListener('load', init);
