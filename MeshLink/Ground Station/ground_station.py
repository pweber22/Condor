import tkinter as tk
from tkinter import ttk, messagebox
from meshlink_protocol import (
    VehicleID,
    MessageType,
    CommandID,
    build_command_payload,
    build_guided_loiter_payload,
    build_mission_upload_fragment,
    PacketHeader,
    MeshLinkPacket,
    PROTOCOL_VERSION,
)


class DummyTransport:
    def write(self, data: bytes):
        print("TX", data.hex())

    def read(self, n: int) -> bytes:
        return b""


class GroundStationGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MeshLink Ground Station")
        self.geometry("1000x700")

        self.transport = DummyTransport()
        self.vehicle_id = VehicleID.GCS
        self.sequence = 0
        self.active_vehicle = 1
        self.mission_waypoints = []
        self.vehicles = {1: (37.7749, -122.4194)}

        self.create_widgets()
        self.draw_map()

    def create_widgets(self):
        left_frame = ttk.Frame(self)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=8)

        self.vehicle_label = ttk.Label(left_frame, text="Active Vehicle: 1")
        self.vehicle_label.pack(pady=4)

        commands = [
            ("RTL", CommandID.RTL),
            ("LOITER", CommandID.LOITER),
            ("AUTO", CommandID.AUTO),
            ("REQUEST STATUS", CommandID.REQUEST_STATUS),
            ("REQUEST TELEMETRY", CommandID.REQUEST_TELEMETRY),
            ("REBOOT BRIDGE", CommandID.REBOOT_BRIDGE),
        ]

        for label, cmd in commands:
            button = ttk.Button(left_frame, text=label, command=lambda c=cmd: self.send_command(c))
            button.pack(fill=tk.X, pady=2)

        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

        self.mode_label = ttk.Label(left_frame, text="Map mode: add waypoint")
        self.mode_label.pack(pady=4)

        self.mode = tk.StringVar(value="waypoint")
        ttk.Radiobutton(left_frame, text="Add waypoint", variable=self.mode, value="waypoint").pack(anchor=tk.W)
        ttk.Radiobutton(left_frame, text="Guided loiter", variable=self.mode, value="loiter").pack(anchor=tk.W)

        self.upload_button = ttk.Button(left_frame, text="Upload Mission", command=self.upload_mission)
        self.upload_button.pack(fill=tk.X, pady=6)

        self.clear_button = ttk.Button(left_frame, text="Clear Mission", command=self.clear_mission)
        self.clear_button.pack(fill=tk.X)

        self.waypoint_list = tk.Listbox(left_frame, height=12)
        self.waypoint_list.pack(fill=tk.BOTH, expand=True, pady=8)

        self.log_text = tk.Text(left_frame, height=12, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=4)

        map_frame = ttk.Frame(self)
        map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.canvas = tk.Canvas(map_frame, bg="#e0e0e0")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_map_click)

    def draw_map(self):
        self.canvas.delete("all")
        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 600

        self.canvas.create_rectangle(0, 0, width, height, fill="#1f2937")
        self.canvas.create_text(10, 10, anchor=tk.NW, text="MeshLink Vehicle Map", fill="white")

        for vid, (lat, lon) in self.vehicles.items():
            x = (lon + 180) / 360 * width
            y = (90 - lat) / 180 * height
            self.canvas.create_oval(x - 8, y - 8, x + 8, y + 8, fill="#0f9d58")
            self.canvas.create_text(x + 12, y, anchor=tk.W, text=f"V{vid}", fill="white")

        for idx, (lat, lon, alt, wtype) in enumerate(self.mission_waypoints, start=1):
            x = (lon + 180) / 360 * width
            y = (90 - lat) / 180 * height
            self.canvas.create_rectangle(x - 5, y - 5, x + 5, y + 5, fill="#fbbc04")
            self.canvas.create_text(x + 10, y, anchor=tk.W, text=f"WP{idx}", fill="white")

    def on_map_click(self, event):
        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 600
        lon = event.x / width * 360 - 180
        lat = 90 - event.y / height * 180

        if self.mode.get() == "waypoint":
            self.add_waypoint(lat, lon)
        else:
            self.send_guided_loiter(lat, lon)

    def add_waypoint(self, lat, lon):
        alt = 100  # default altitude
        wtype = 0
        self.mission_waypoints.append((int(lat * 1e7), int(lon * 1e7), alt, wtype))
        self.waypoint_list.insert(tk.END, f"WP{len(self.mission_waypoints)}: {lat:.5f},{lon:.5f}")
        self.log(f"Added waypoint {len(self.mission_waypoints)}")
        self.draw_map()

    def upload_mission(self):
        if not self.mission_waypoints:
            messagebox.showwarning("No mission", "Add waypoints to upload a mission.")
            return
        self.log("Uploading mission...")
        self.send_mission_upload(self.active_vehicle, self.mission_waypoints)

    def clear_mission(self):
        self.mission_waypoints.clear()
        self.waypoint_list.delete(0, tk.END)
        self.log("Mission cleared")
        self.draw_map()

    def send_command(self, command_id):
        self.log(f"Send command {command_id.name} to V{self.active_vehicle}")
        payload = build_command_payload(command_id, 0)
        self.send_packet(self.active_vehicle, MessageType.COMMAND, payload)

    def send_guided_loiter(self, lat, lon):
        self.log(f"Guided loiter at {lat:.5f},{lon:.5f}")
        payload = build_guided_loiter_payload(int(lat * 1e7), int(lon * 1e7), 100, 50)
        self.send_packet(self.active_vehicle, MessageType.GUIDED_LOITER, payload)

    def send_mission_upload(self, destination, waypoints, fragment_size: int = 4):
        total_waypoints = len(waypoints)
        fragments = []
        for first_idx in range(0, total_waypoints, fragment_size):
            chunk = waypoints[first_idx:first_idx + fragment_size]
            payload = build_mission_upload_fragment(total_waypoints, first_idx, chunk)
            fragments.append(payload)

        for part, payload in enumerate(fragments, start=1):
            self.send_packet(destination, MessageType.MISSION_UPLOAD, payload, total_parts=len(fragments), part=part)
        self.log(f"Sent mission upload {len(fragments)} fragments")

    def send_packet(self, destination, type_, payload, total_parts: int = 1, part: int = 0):
        self.sequence = (self.sequence + 1) & 0xFF
        header = PacketHeader(
            version=PROTOCOL_VERSION,
            source=self.vehicle_id,
            destination=destination,
            type=type_,
            sequence=self.sequence,
            total_parts=total_parts,
            part=part,
            payload_length=len(payload),
            crc16=0,
        )
        packet = MeshLinkPacket(header, payload)
        self.transport.write(packet.pack())
        self.log(f"TX {type_.name} seq={self.sequence} part={part}/{total_parts}")

    def log(self, message):
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)


if __name__ == "__main__":
    app = GroundStationGUI()
    app.mainloop()
