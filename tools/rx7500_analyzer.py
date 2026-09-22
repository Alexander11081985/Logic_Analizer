"""Tkinter 8-channel logic analyzer for RX7500, PAL and CVBS failover."""

from __future__ import annotations

import csv
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
    SERIAL_IMPORT_ERROR = None
except ImportError as error:
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = error

from pal_analyzer import analyze_failover, analyze_pal_field, analyze_pal_line
from peak67_analyzer import analyze_peak67, analyze_peak67_power_on
from rx7500_protocol import (
    ACQ_EDGE, ACQ_RAW, FLAG_OVERFLOW, FLAG_TIMEOUT, FLAG_TRUNCATED,
    TRIGGER_CHANNEL_IMMEDIATE, TRIGGER_EITHER, TRIGGER_FALLING,
    TRIGGER_IMMEDIATE, TRIGGER_RISING, Capture, PacketParser, analyze_capture,
    build_config_command, capture_transition_rows, suppress_short_pulses,
    transition_rows,
)

COLORS = ("#00d0ff", "#ffd166", "#ef476f", "#7bed9f",
          "#b388ff", "#ff9f43", "#70a1ff", "#2ed573")

PROFILES = {
    "RX7500 SPI": {
        "acq": ACQ_RAW, "rate": 5_000_000, "count": 8192,
        "duration": 0, "trigger": 0, "edge": TRIGGER_RISING,
        "names": ["CLK", "DATA", "LE", "SPI/M", "AUX4", "AUX5", "AUX6", "AUX7"],
    },
    "PEAK67 3-wire reverse engineering": {
        "acq": ACQ_EDGE, "rate": 0, "count": 0,
        "duration": 20_000, "trigger": 0, "edge": TRIGGER_RISING,
        "timeout": 10_000,
        "names": ["PEAK_LINE0", "PEAK_LINE1", "PEAK_LINE2", "AUX3", "AUX4", "AUX5", "AUX6", "AUX7"],
    },
    "PEAK67 power-on timing": {
        "acq": ACQ_EDGE, "rate": 0, "count": 0,
        "duration": 1_000_000, "trigger": 3, "edge": TRIGGER_RISING,
        "timeout": 60_000,
        "names": ["PEAK_CLK", "PEAK_CS", "PEAK_DATA", "PEAK_3V3",
                  "AUX4", "AUX5", "AUX6", "AUX7"],
    },
    "PAL GPIO DAC / line": {
        "acq": ACQ_RAW, "rate": 5_000_000, "count": 8192,
        "duration": 0, "trigger": 0, "edge": TRIGGER_FALLING,
        "names": ["VIDEO_D0", "VIDEO_D1", "VIDEO_D2", "VIDEO_D3",
                  "VIDEO_D4", "VIDEO_D5", "VIDEO_SEL", "AUX/CSYNC"],
    },
    "PAL field timing": {
        "acq": ACQ_EDGE, "rate": 0, "count": 0,
        "duration": 45_000, "trigger": 0, "edge": TRIGGER_FALLING,
        "names": ["SYNC", "AUX1", "AUX2", "AUX3", "AUX4", "AUX5", "AUX6", "AUX7"],
    },
    "LM1881 / video failover": {
        "acq": ACQ_EDGE, "rate": 0, "count": 0,
        "duration": 100_000, "trigger": 0, "edge": TRIGGER_EITHER,
        "names": ["REAL_CSYNC", "REAL_VSYNC", "VIDEO_SEL", "FAKE_SYNC",
                  "AUX4", "AUX5", "AUX6", "AUX7"],
    },
    "Generic 8-channel": {
        "acq": ACQ_RAW, "rate": 5_000_000, "count": 8192,
        "duration": 0, "trigger": 0, "edge": TRIGGER_RISING,
        "names": [f"CH{index}" for index in range(8)],
    },
}

EDGE_NAMES = {TRIGGER_RISING: "Rising", TRIGGER_FALLING: "Falling",
              TRIGGER_EITHER: "Either", TRIGGER_IMMEDIATE: "Immediate"}
EDGE_VALUES = {value: key for key, value in EDGE_NAMES.items()}


class SerialWorker:
    def __init__(self, event_queue: queue.Queue) -> None:
        self.events = event_queue
        self.port = None
        self.thread = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, port_name: str) -> None:
        self.disconnect()
        self.port = serial.Serial(port_name, 921600, timeout=0.10, write_timeout=1.0)
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        port, self.port = self.port, None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass

    def send(self, data: bytes) -> bool:
        if not self.connected:
            return False
        try:
            with self.write_lock:
                self.port.write(data)
                self.port.flush()
            return True
        except Exception as error:
            self.events.put(("error", f"USB write failed: {error}"))
            return False

    def _reader(self) -> None:
        parser = PacketParser()
        try:
            while not self.stop_event.is_set():
                port = self.port
                if port is None or not port.is_open:
                    break
                data = port.read(16384)
                if not data:
                    continue
                for capture in parser.feed(data):
                    self.events.put(("capture", capture))
                if parser.crc_errors or parser.header_errors:
                    self.events.put(("warning", f"Parser: CRC={parser.crc_errors}, header={parser.header_errors}"))
                    parser.crc_errors = parser.header_errors = 0
        except Exception as error:
            if not self.stop_event.is_set():
                self.events.put(("error", f"Serial reader stopped: {error}"))
        finally:
            self.events.put(("disconnected", None))


class AnalyzerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ESP32-S3 8-channel Logic Analyzer — RX7500 / PAL / Failover")
        self.root.geometry("1550x930")
        self.root.minsize(1050, 700)
        self.events = queue.Queue()
        self.worker = SerialWorker(self.events) if serial is not None else None
        self.capture: Capture | None = None
        self.analysis: dict[str, object] | None = None
        self.request_id = 0
        self.pixels_per_us = 6.0
        self.view_start_us = 0.0
        self.view_end_us = 1.0
        self.wave_margin = 120
        self.cursor_item = None

        self.port_var = tk.StringVar()
        self.profile_var = tk.StringVar(value="RX7500 SPI")
        self.acq_var = tk.StringVar(value="Raw samples")
        self.rate_var = tk.StringVar(value="5000000")
        self.count_var = tk.StringVar(value="8192")
        self.duration_var = tk.StringVar(value="45000")
        self.trigger_var = tk.StringVar(value="CH0")
        self.edge_var = tk.StringVar(value="Rising")
        self.timeout_var = tk.StringVar(value="1000")
        self.filter_var = tk.StringVar(value="0")
        self.status_var = tk.StringVar(value="Disconnected")
        self.cursor_var = tk.StringVar(value="Cursor: —")
        self.auto_var = tk.BooleanVar(value=False)
        self.full_var = tk.BooleanVar(value=False)
        self.real_level_var = tk.IntVar(value=1)
        self.channel_names = [tk.StringVar() for _ in range(8)]
        self.channel_visible = [tk.BooleanVar(value=True) for _ in range(8)]

        self._build_ui()
        self.profile_var.trace_add("write", self._profile_changed)
        self._apply_profile()
        self.refresh_ports()
        self.root.after(50, self._poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        if SERIAL_IMPORT_ERROR is not None:
            self.root.after(100, lambda: messagebox.showerror(
                "Missing pyserial", "Run: py -3 -m pip install -r tools\\requirements.txt"))

    def _build_ui(self) -> None:
        connection = ttk.Frame(self.root, padding=(8, 8, 8, 3))
        connection.pack(fill=tk.X)
        ttk.Label(connection, text="Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=31, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(connection, text="Connect", command=self.connect)
        self.connect_button.pack(side=tk.LEFT, padx=3)
        ttk.Button(connection, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT)
        ttk.Separator(connection, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Button(connection, text="Capture", command=self.capture_once).pack(side=tk.LEFT)
        ttk.Checkbutton(connection, text="Auto", variable=self.auto_var).pack(side=tk.LEFT, padx=5)
        ttk.Button(connection, text="Legacy RX7500 R", command=self.legacy_capture).pack(side=tk.LEFT, padx=3)
        ttk.Label(connection, textvariable=self.cursor_var).pack(side=tk.RIGHT)

        config = ttk.LabelFrame(self.root, text="Acquisition configuration", padding=6)
        config.pack(fill=tk.X, padx=8, pady=3)
        fields = [
            ("Profile", ttk.Combobox(config, textvariable=self.profile_var, values=list(PROFILES), width=24, state="readonly")),
            ("Mode", ttk.Combobox(config, textvariable=self.acq_var, values=("Raw samples", "Edge events"), width=12, state="readonly")),
            ("Rate, Hz", ttk.Combobox(config, textvariable=self.rate_var, values=("100000", "500000", "1000000", "2000000", "5000000"), width=10)),
            ("Samples", ttk.Entry(config, textvariable=self.count_var, width=8)),
            ("Duration, µs", ttk.Entry(config, textvariable=self.duration_var, width=9)),
            ("Trigger", ttk.Combobox(config, textvariable=self.trigger_var, values=tuple([f"CH{i}" for i in range(8)] + ["Immediate"]), width=9, state="readonly")),
            ("Edge", ttk.Combobox(config, textvariable=self.edge_var, values=tuple(EDGE_VALUES), width=9, state="readonly")),
            ("Timeout, ms", ttk.Entry(config, textvariable=self.timeout_var, width=7)),
            ("Glitch, ns", ttk.Entry(config, textvariable=self.filter_var, width=7)),
        ]
        for column, (label, widget) in enumerate(fields):
            ttk.Label(config, text=label).grid(row=0, column=column, padx=3, sticky="w")
            widget.grid(row=1, column=column, padx=3, sticky="ew")
        ttk.Button(config, text="Apply / arm", command=self.capture_once).grid(row=1, column=len(fields), padx=6)

        channels = ttk.LabelFrame(self.root, text="Channels (GPIO4…GPIO11)", padding=5)
        channels.pack(fill=tk.X, padx=8, pady=3)
        for index in range(8):
            frame = ttk.Frame(channels)
            frame.pack(side=tk.LEFT, padx=5)
            ttk.Checkbutton(frame, text=f"CH{index}/G{index + 4}", variable=self.channel_visible[index],
                            command=self.render_waveform).pack(anchor="w")
            ttk.Entry(frame, textvariable=self.channel_names[index], width=13).pack()

        status = ttk.Frame(self.root, padding=(8, 2))
        status.pack(fill=tk.X)
        ttk.Label(status, textvariable=self.status_var).pack(side=tk.LEFT)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 8))
        summary_box = ttk.LabelFrame(body, text="Analysis", padding=5)
        body.add(summary_box, weight=1)
        self.summary = tk.Text(summary_box, width=43, wrap="word", state="disabled", font=("Consolas", 9))
        self.summary.pack(fill=tk.BOTH, expand=True)

        right = ttk.Frame(body)
        body.add(right, weight=4)
        toolbar = ttk.Frame(right)
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="−", width=3, command=lambda: self.zoom(0.7)).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="+", width=3, command=lambda: self.zoom(1.4)).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Fit activity", command=self.fit_activity).pack(side=tk.LEFT)
        ttk.Checkbutton(toolbar, text="Full capture", variable=self.full_var,
                        command=self.fit_waveform).pack(side=tk.LEFT, padx=8)
        ttk.Label(toolbar, text="VIDEO_SEL real level:").pack(side=tk.LEFT, padx=(15, 3))
        ttk.Combobox(toolbar, textvariable=self.real_level_var, values=(0, 1), width=3,
                     state="readonly").pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Save CSV", command=self.save_csv).pack(side=tk.RIGHT)

        notebook = ttk.Notebook(right)
        notebook.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        wave_page = ttk.Frame(notebook)
        notebook.add(wave_page, text="Waveform")
        self.canvas = tk.Canvas(wave_page, background="#101419", highlightthickness=0)
        scroll = ttk.Scrollbar(wave_page, orient=tk.HORIZONTAL, command=self.canvas.xview)
        self.canvas.configure(xscrollcommand=scroll.set)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        scroll.pack(fill=tk.X)
        self.canvas.bind("<Motion>", self._wave_motion)
        self.canvas.bind("<Leave>", self._wave_leave)
        self.canvas.bind("<MouseWheel>", lambda event: self.zoom(1.2 if event.delta > 0 else 1 / 1.2))
        self.canvas.bind("<Configure>", lambda _event: self.render_waveform())
        transitions_page = ttk.Frame(notebook)
        notebook.add(transitions_page, text="Transitions / events")
        columns = ("time", *[f"ch{i}" for i in range(8)], "changed")
        self.tree = ttk.Treeview(transitions_page, columns=columns, show="headings")
        for column in columns:
            self.tree.heading(column, text=column.upper())
            self.tree.column(column, width=85 if column in ("time", "changed") else 48, anchor=tk.CENTER)
        tree_scroll = ttk.Scrollbar(transitions_page, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=tree_scroll.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

    def _profile_changed(self, *_args) -> None:
        if self.profile_var.get() in PROFILES:
            self._apply_profile()

    def _apply_profile(self) -> None:
        profile = PROFILES[self.profile_var.get()]
        self.acq_var.set("Raw samples" if profile["acq"] == ACQ_RAW else "Edge events")
        self.rate_var.set(str(profile["rate"]))
        self.count_var.set(str(profile["count"]))
        self.duration_var.set(str(profile["duration"]))
        self.timeout_var.set(str(profile.get("timeout", 1000)))
        self.trigger_var.set(f"CH{profile['trigger']}")
        self.edge_var.set(EDGE_NAMES[profile["edge"]])
        for variable, name in zip(self.channel_names, profile["names"]):
            variable.set(name)
        for index, variable in enumerate(self.channel_visible):
            if self.profile_var.get() == "RX7500 SPI":
                variable.set(index < 4)
            elif self.profile_var.get().startswith("PEAK67"):
                count = 4 if self.profile_var.get() == "PEAK67 power-on timing" else 3
                variable.set(index < count)
            else:
                variable.set(True)

    def refresh_ports(self) -> None:
        if list_ports is None:
            return
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        values = [f"{item.device} — {item.description}" for item in ports]
        self.port_combo["values"] = values
        if values and self.port_var.get() not in values:
            preferred = next((
                f"{item.device} — {item.description}"
                for item in ports if item.vid == 0x303A
            ), None)
            if preferred is None:
                preferred = next((v for v in values if v.startswith("COM9 ")), None)
            if preferred is None:
                preferred = next((v for v in values if v.startswith("COM14 ")), values[0])
            self.port_var.set(preferred)

    def connect(self) -> None:
        if self.worker is None or not self.port_var.get():
            return
        try:
            self.worker.connect(self.port_var.get().split(" — ", 1)[0])
            self.connect_button.state(["disabled"])
            self.status_var.set("Connected; configure and press Capture")
        except Exception as error:
            messagebox.showerror("Connection failed", str(error))

    def disconnect(self) -> None:
        if self.worker:
            self.worker.disconnect()
        self.connect_button.state(["!disabled"])
        self.status_var.set("Disconnected")

    def _configuration(self) -> bytes:
        acquisition = ACQ_RAW if self.acq_var.get() == "Raw samples" else ACQ_EDGE
        trigger = (TRIGGER_CHANNEL_IMMEDIATE if self.trigger_var.get() == "Immediate"
                   else int(self.trigger_var.get()[2:]))
        edge = EDGE_VALUES[self.edge_var.get()]
        if trigger == TRIGGER_CHANNEL_IMMEDIATE:
            edge = TRIGGER_IMMEDIATE
        self.request_id += 1
        return build_config_command(
            request_id=self.request_id, acquisition=acquisition,
            sample_rate_hz=int(self.rate_var.get()) if acquisition == ACQ_RAW else 0,
            sample_count=int(self.count_var.get()) if acquisition == ACQ_RAW else 0,
            duration_us=int(self.duration_var.get()) if acquisition == ACQ_EDGE else 0,
            trigger_channel=trigger, trigger_edge=edge,
            trigger_timeout_ms=int(self.timeout_var.get()),
        )

    def capture_once(self) -> None:
        if not self.worker or not self.worker.connected:
            messagebox.showwarning("Not connected", "Connect to ESP32-S3 first.")
            return
        try:
            command = self._configuration()
        except (ValueError, KeyError) as error:
            messagebox.showerror("Invalid configuration", str(error))
            return
        if self.worker.send(command):
            self.status_var.set(f"ARMED — {self.profile_var.get()}, request #{self.request_id}")

    def legacy_capture(self) -> None:
        if self.worker and self.worker.connected and self.worker.send(b"R"):
            self.profile_var.set("RX7500 SPI")
            self._apply_profile()
            self.status_var.set("ARMED — legacy RX7500 v1 / CH0 rising")

    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "capture":
                    first_capture = self.capture is None
                    if self._capture_has_display_activity(payload):
                        self.capture = payload
                        self.reanalyze(auto_fit=first_capture)
                    else:
                        kept = f"#{self.capture.number}" if self.capture is not None else "none"
                        self.status_var.set(
                            f"No new PEAK67 transitions on CH0-CH2; keeping capture {kept}"
                        )
                    if self.auto_var.get() and self.worker and self.worker.connected:
                        self.root.after(250, self.capture_once)
                elif kind == "error":
                    self.status_var.set(str(payload))
                    messagebox.showerror("Analyzer", str(payload))
                elif kind == "warning":
                    self.status_var.set(str(payload))
                elif kind == "disconnected" and (not self.worker or not self.worker.connected):
                    self.connect_button.state(["!disabled"])
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _capture_has_display_activity(self, capture: Capture) -> bool:
        if self.profile_var.get() != "PEAK67 3-wire reverse engineering":
            return True
        peak_mask = 0x07
        if capture.acquisition == ACQ_EDGE:
            return any(event.changed_mask & peak_mask for event in capture.events)
        rows = capture_transition_rows(capture)
        # Row zero in raw mode describes the initial sample rather than a real
        # transition, therefore only later rows count as new PEAK67 activity.
        return any(index != 0 and changed & peak_mask for index, _, changed in rows)

    def reanalyze(self, auto_fit: bool = False) -> None:
        if self.capture is None:
            return
        profile = self.profile_var.get()
        try:
            if profile == "RX7500 SPI":
                self.analysis = analyze_capture(self.capture, max(0.0, float(self.filter_var.get())))
            elif profile == "PEAK67 3-wire reverse engineering":
                self.analysis = analyze_peak67(self.capture, max(0.0, float(self.filter_var.get())))
            elif profile == "PEAK67 power-on timing":
                self.analysis = analyze_peak67_power_on(
                    self.capture, max(0.0, float(self.filter_var.get())))
            elif profile == "PAL GPIO DAC / line":
                self.analysis = analyze_pal_line(self._filtered_capture())
            elif profile == "PAL field timing":
                self.analysis = analyze_pal_field(self.capture)
            elif profile == "LM1881 / video failover":
                self.analysis = analyze_failover(self.capture, real_select_level=self.real_level_var.get())
            else:
                self.analysis = {}
        except (ValueError, ZeroDivisionError) as error:
            self.analysis = {"verdict": "FAIL", "error": str(error)}
        self._update_summary()
        self._update_transitions()
        if auto_fit:
            self.fit_waveform()
        else:
            # Preserve the user's horizontal scale between captures.  This is
            # important in Auto mode when several known channel changes must be
            # compared at exactly the same time scale.
            self.render_waveform()

    def _filtered_capture(self) -> Capture:
        capture = self.capture
        if capture is None or capture.acquisition != ACQ_RAW:
            return capture
        minimum = max(1, int(float(self.filter_var.get()) / capture.sample_period_ns + 0.999))
        samples = suppress_short_pulses(capture.samples, minimum, 8)
        return Capture(capture.number, capture.sample_rate_hz, samples=samples,
                       flags=capture.flags, version=capture.version,
                       acquisition=ACQ_RAW, channel_count=capture.channel_count)

    def _summary_lines(self) -> list[str]:
        c, a = self.capture, self.analysis or {}
        if c is None:
            return ["No capture"]
        mode = "raw samples" if c.acquisition == ACQ_RAW else "edge events"
        flags = []
        if c.flags & FLAG_TIMEOUT: flags.append("TIMEOUT")
        if c.flags & FLAG_OVERFLOW: flags.append("OVERFLOW")
        if c.flags & FLAG_TRUNCATED: flags.append("TRUNCATED")
        lines = [f"Capture: #{c.number}  protocol v{c.version}",
                 f"Request: {c.request_id}", f"Actual mode: {mode}",
                 f"Channels: {c.channel_count}", f"Duration: {c.duration_us:.3f} µs",
                 f"Trigger: CH{c.trigger_channel} / {EDGE_NAMES.get(c.trigger_edge, '?')}",
                 f"Flags: {', '.join(flags) if flags else 'none'}"]
        if c.acquisition == ACQ_RAW:
            lines += [f"Actual rate: {c.sample_rate_hz} Hz",
                      f"Resolution: {c.sample_period_ns:.1f} ns", f"Samples: {len(c.samples)}"]
        else:
            lines += [f"Timestamp: {c.timestamp_hz} Hz", f"Events: {len(c.events)}",
                      f"Lost events: {c.lost_events}"]
        profile = self.profile_var.get()
        lines += ["", f"Profile: {profile}"]
        if "error" in a:
            lines += [f"ERROR: {a['error']}"]
        elif profile == "RX7500 SPI":
            bits = "".join(map(str, a["bits"]))
            frame_text = f"0x{a['frame']:06X}" if a["frame"] is not None else "invalid"
            lines += [f"CLK rises/falls: {len(a['clk_rising'])}/{len(a['clk_falling'])}",
                      f"Bits: {' '.join(bits[i:i+8] for i in range(0, len(bits), 8))}",
                      f"Frame: {frame_text}",
                      f"Frequency: {a['direct_frequency'] or '—'} MHz",
                      f"SPI/M HIGH: {a['mode_high_percent']:.2f}%",
                      f"Verdict: {'VALID' if a['direct_frequency'] and len(a['clk_rising']) == 24 and a['selected_le'] else 'INVALID'}"]
        elif profile.startswith("PEAK67"):
            def timing_text(stats):
                if not stats or stats[0] is None:
                    return "—"
                return f"{stats[0]:.3f}/{stats[1]:.3f}/{stats[2]:.3f} µs"

            if profile == "PEAK67 power-on timing":
                power_time = a.get("power_rise_us")
                frame = a.get("first_valid_frame")
                lines += [
                    f"3V3 rising: {power_time:.3f} µs" if power_time is not None else "3V3 rising: —",
                    f"Power timing: {a.get('power_timing_verdict', '—')}",
                ]
                if frame:
                    lines += [
                        f"First valid frame: {frame['frame_hex']}",
                        f"3V3 rising → CS falling: {frame['power_to_cs_fall_us']:.3f} µs",
                        f"3V3 rising → first CLK rising: {frame['power_to_first_clk_rise_us']:.3f} µs",
                        f"3V3 rising → frame committed (CS rising): {frame['power_to_frame_complete_us']:.3f} µs",
                    ]
                lines.append("")

            lines += [
                f"Transitions CH0/CH1/CH2: {a.get('transition_counts', '—')}",
                f"Detected candidate mapping: CLK=CH{a.get('detected_clock_channel', '?')}, DATA=CH{a.get('detected_data_channel', '?')}, LE/CS=CH{a.get('detected_control_channel', '?')}",
                f"Mapping confidence: {a.get('mapping_basis', '—')}",
                f"Initial CH0/CH1/CH2: {a.get('initial_levels', '—')}",
                f"Final   CH0/CH1/CH2: {a.get('final_levels', '—')}",
                f"CLK rises/falls: {a.get('clk_rising_count', 0)}/{a.get('clk_falling_count', 0)}",
                f"CLK period min/avg/max: {timing_text(a.get('clock_period_stats_us'))}",
                f"CLK HIGH min/avg/max: {timing_text(a.get('clock_high_stats_us'))}",
                f"CLK LOW  min/avg/max: {timing_text(a.get('clock_low_stats_us'))}",
                f"DATA setup@rise: {timing_text(a.get('rising_setup_stats_us'))}",
                f"DATA hold @rise: {timing_text(a.get('rising_hold_stats_us'))}",
                f"DATA setup@fall: {timing_text(a.get('falling_setup_stats_us'))}",
                f"DATA hold @fall: {timing_text(a.get('falling_hold_stats_us'))}",
                f"LE/CS rises/falls: {a.get('control_rising_count', 0)}/{a.get('control_falling_count', 0)}",
                f"LE/CS HIGH min/avg/max: {timing_text(a.get('control_high_stats_us'))}",
                f"LE/CS LOW  min/avg/max: {timing_text(a.get('control_low_stats_us'))}",
                f"Line-3 interpretation: {a.get('control_role_candidate', '—')}",
                f"Verdict: {a.get('verdict', '—')}",
            ]
            for frame in a.get("frames", [])[:8]:
                lines.append("")
                lines.append(f"Frame candidate #{frame['index']}  LE/CS during clocks={frame['control_level_during_clocks']}")
                for edge_name, edge_key in (("CLK rising", "rising"), ("CLK falling", "falling")):
                    decoded = frame.get(edge_key)
                    if not decoded:
                        continue
                    bits = decoded["bit_string"]
                    grouped = " ".join(bits[offset:offset + 8] for offset in range(0, len(bits), 8))
                    msb_bytes = " ".join(f"{value:02X}" for value in decoded["msb_bytes"]) or "—"
                    lsb_bytes = " ".join(f"{value:02X}" for value in decoded["lsb_bytes"]) or "—"
                    lines += [
                        f"  {edge_name}: {decoded['bit_count']} bits: {grouped}",
                        f"    MSB-first value {decoded['msb_hex']}; bytes {msb_bytes}",
                        f"    LSB-first value {decoded['lsb_hex']}; bytes {lsb_bytes}",
                        f"    simultaneous DATA+CLK changes: {decoded['simultaneous_data_clock']}",
                    ]
            lines += ["", a.get("pretrigger_warning", "")]
        elif profile == "PAL GPIO DAC / line":
            lines += [f"Resolution: {a.get('resolution_us', 0):.3f} µs (cannot resolve 100 ns STM samples)"]
            for key in ("line", "hsync", "back_porch", "active", "front_porch"):
                value = a.get("measured_us", {}).get(key)
                if value is not None:
                    lines.append(f"{key}: {value:.3f} µs  err {a['errors_us'][key]:+.3f}  {a['ratings'][key]}")
            lines += ["DAC: " + ", ".join(f"{code}={name}" for code, name in a.get("symbolic_codes", [])),
                      f"Verdict: {a.get('verdict', '—')}"]
        elif profile == "PAL field timing":
            avg = lambda values: sum(values) / len(values) if values else None
            for label, key in (("Line", "normal_line_periods_us"), ("Half-line", "half_line_periods_us"),
                               ("Field", "field_periods_us"), ("Frame", "frame_periods_us")):
                value = avg(a.get(key, []))
                lines.append(f"{label}: {value:.3f} µs" if value is not None else f"{label}: —")
            lines += [f"Second field phase: {a.get('second_field_phase_us', '—')} µs",
                      f"Vertical sequences: {a.get('sequences', [])}",
                      f"Anomalous pulses: {a.get('anomalous_count', 0)}",
                      f"Verdict: {a.get('verdict', '—')}"]
        elif profile == "LM1881 / video failover":
            for label, key in (("CSYNC avg", "csync_avg_us"), ("Last real sync", "last_valid_real_sync_us"),
                               ("SEL→fake", "video_sel_to_fake_us"), ("First restore", "first_restored_sync_us"),
                               ("Stable lock", "stable_lock_us"), ("SEL→real", "video_sel_to_real_us"),
                               ("Loss latency", "loss_to_switch_us"), ("Restore latency", "restore_to_switch_us")):
                value = a.get(key)
                lines.append(f"{label}: {value:.3f} µs" if value is not None else f"{label}: —")
            lines += [f"Valid/invalid lines: {a.get('valid_lines', 0)}/{a.get('invalid_lines', 0)}",
                      f"Fake generator active: {a.get('fake_sync_active', False)}"]
        else:
            lines.append("No protocol-specific verdict.")
        return lines

    def _update_summary(self) -> None:
        self.summary.configure(state="normal")
        self.summary.delete("1.0", tk.END)
        self.summary.insert("1.0", "\n".join(self._summary_lines()))
        self.summary.configure(state="disabled")
        self.status_var.set(f"Capture #{self.capture.number} received — {self.profile_var.get()}")

    def _rows_with_time(self) -> list[tuple[int, float, int, int]]:
        if self.capture is None:
            return []
        rows = capture_transition_rows(self.capture)
        if self.capture.acquisition == ACQ_RAW:
            return [(index, index * 1_000_000.0 / self.capture.sample_rate_hz, state, changed)
                    for index, state, changed in rows]
        return [(ticks, ticks * 1_000_000.0 / self.capture.timestamp_hz, state, changed)
                for ticks, state, changed in rows]

    def _update_transitions(self) -> None:
        self.tree.delete(*self.tree.get_children())
        names = [variable.get() for variable in self.channel_names]
        for _, time_us, state, changed in self._rows_with_time()[:10000]:
            changed_text = "+".join(names[i] for i in range(8) if changed & (1 << i))
            self.tree.insert("", tk.END, values=(f"{time_us:.3f}",
                             *[1 if state & (1 << i) else 0 for i in range(8)], changed_text))

    def _display_window(self) -> tuple[float, float]:
        if self.capture is None:
            return 0.0, 1.0
        if self.full_var.get():
            return 0.0, max(self.capture.duration_us, 0.1)
        rows = self._rows_with_time()
        if not rows:
            return 0.0, max(self.capture.duration_us, 0.1)

        # Default to the real activity envelope instead of wasting most of the
        # plot on trigger wait / idle capture tail.
        times = [time_us for _, time_us, _, changed in rows if changed]
        if not times:
            return 0.0, max(self.capture.duration_us, 0.1)
        first, last = min(times), max(times)
        span = max(last - first, 0.2)
        padding = max(span * 0.06, 0.5)
        start = max(0.0, first - padding)
        end = last + padding
        if end - start < 1.0:
            end = start + 1.0
        return start, end

    def _display_duration(self) -> float:
        start, end = self._display_window()
        return max(end - start, 0.1)

    def render_waveform(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        if self.capture is None:
            canvas.create_text(30, 30, anchor="nw", fill="#b8c0cc", text="Connect and press Capture.")
            return
        visible = [i for i in range(8) if self.channel_visible[i].get()]
        dac_lane = self.profile_var.get() == "PAL GPIO DAC / line" and self.capture.acquisition == ACQ_RAW
        lane_count = len(visible) + (1 if dac_lane else 0)
        if not lane_count:
            return
        start_us, end_us = self._display_window()
        self.view_start_us, self.view_end_us = start_us, end_us
        duration_us = max(end_us - start_us, 0.1)
        height = max(460, canvas.winfo_height())
        width = max(canvas.winfo_width(), int(self.wave_margin + duration_us * self.pixels_per_us + 60))
        canvas.configure(scrollregion=(0, 0, width, height))
        tick_candidates = (0.1, .2, .5, 1, 2, 5, 10, 20, 50, 100, 500, 1000, 5000, 10000)
        tick = next((v for v in tick_candidates if v * self.pixels_per_us >= 70), 20000)
        first_tick = int(start_us / tick) * tick
        if first_tick < start_us:
            first_tick += tick
        t = first_tick
        while t <= end_us:
            x = self.wave_margin + (t - start_us) * self.pixels_per_us
            canvas.create_line(x, 25, x, height - 20, fill="#26313b")
            canvas.create_text(x + 3, 10, anchor="nw", fill="#7f8c98", text=f"{t:g} µs")
            t += tick
        lane_height = (height - 60) / lane_count
        rows = self._rows_with_time()
        initial = self.capture.samples[0] if self.capture.samples else self.capture.initial_state
        for _, time_us, state, _ in rows:
            if time_us >= start_us:
                break
            initial = state
        mapping_names: dict[int, str] = {}
        if self.profile_var.get().startswith("PEAK67") and self.analysis:
            for key, role in (("detected_clock_channel", "CLK?"),
                              ("detected_data_channel", "DATA?"),
                              ("detected_control_channel", "LE/CS?")):
                channel = self.analysis.get(key)
                if isinstance(channel, int):
                    mapping_names[channel] = role
        for lane, channel in enumerate(visible):
            center = 40 + lane * lane_height + lane_height / 2
            high, low = center - min(16, lane_height / 4), center + min(16, lane_height / 4)
            name, color, mask = self.channel_names[channel].get(), COLORS[channel], 1 << channel
            role = mapping_names.get(channel)
            lane_label = f"{name}  [{role}]" if role else name
            canvas.create_text(8, center, anchor="w", fill=color, text=lane_label,
                               font=("Segoe UI", 9, "bold"))
            level = bool(initial & mask)
            points = [self.wave_margin, high if level else low]
            for _, time_us, state, changed in rows:
                if time_us < start_us: continue
                if time_us > end_us: break
                if not changed & mask: continue
                x = self.wave_margin + (time_us - start_us) * self.pixels_per_us
                points.extend((x, high if level else low))
                level = bool(state & mask)
                points.extend((x, high if level else low))
            points.extend((self.wave_margin + duration_us * self.pixels_per_us, high if level else low))
            canvas.create_line(*points, fill=color, width=2)
        if dac_lane:
            lane = len(visible)
            center = 40 + lane * lane_height + lane_height / 2
            amplitude = min(25, lane_height / 2 - 5)
            canvas.create_text(8, center, anchor="w", fill="#ffffff", text="DAC code", font=("Segoe UI", 9, "bold"))
            samples = self.capture.samples
            points = []
            previous = samples[0] & 0x3F
            points.extend((self.wave_margin, center + amplitude - 2 * amplitude * previous / 63))
            for index, state, _ in transition_rows(samples):
                if index == 0: continue
                time_us = index * 1_000_000.0 / self.capture.sample_rate_hz
                if time_us > duration_us: break
                code = state & 0x3F
                if code == previous: continue
                x = self.wave_margin + time_us * self.pixels_per_us
                points.extend((x, center + amplitude - 2 * amplitude * previous / 63,
                               x, center + amplitude - 2 * amplitude * code / 63))
                previous = code
            points.extend((self.wave_margin + duration_us * self.pixels_per_us,
                           center + amplitude - 2 * amplitude * previous / 63))
            canvas.create_line(*points, fill="#ffffff", width=2)

    def fit_waveform(self) -> None:
        start, end = self._display_window()
        self.view_start_us, self.view_end_us = start, end
        duration = max(end - start, 0.1)
        available = max(200, self.canvas.winfo_width() - self.wave_margin - 40)
        self.pixels_per_us = max(0.005, min(500.0, available / max(duration, 0.1)))
        self.render_waveform()
        self.canvas.xview_moveto(0)

    def fit_activity(self) -> None:
        # The button is an explicit request for the cropped activity view even
        # when "Full capture" was previously enabled.
        self.full_var.set(False)
        self.fit_waveform()

    def zoom(self, factor: float) -> None:
        self.pixels_per_us = max(0.002, min(500.0, self.pixels_per_us * factor))
        self.render_waveform()

    def _wave_motion(self, event) -> None:
        x = self.canvas.canvasx(event.x)
        time_us = max(self.view_start_us,
                      self.view_start_us + (x - self.wave_margin) / self.pixels_per_us)
        self.cursor_var.set(f"Cursor: {time_us:.3f} µs")
        if self.cursor_item is not None:
            self.canvas.delete(self.cursor_item)
        self.cursor_item = self.canvas.create_line(x, 20, x, max(450, self.canvas.winfo_height()) - 15,
                                                   fill="#ffffff", dash=(3, 3), tags="cursor")

    def _wave_leave(self, _event) -> None:
        self.cursor_var.set("Cursor: —")
        if self.cursor_item is not None:
            self.canvas.delete(self.cursor_item)
            self.cursor_item = None

    def save_csv(self) -> None:
        if self.capture is None:
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
            initialfile=f"logic_capture_{self.capture.number}.csv", filetypes=(("CSV", "*.csv"),))
        if not path:
            return
        names = [variable.get() for variable in self.channel_names]
        mode = "raw" if self.capture.acquisition == ACQ_RAW else "edge"
        with open(path, "w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(("profile", "acquisition", "sample_or_timestamp", "time_us",
                             *[f"CH{i}" for i in range(8)], *[f"name{i}" for i in range(8)], "changed_channels"))
            for index, time_us, state, changed in self._rows_with_time():
                changed_text = "+".join(names[i] for i in range(8) if changed & (1 << i))
                writer.writerow((self.profile_var.get(), mode, index, f"{time_us:.6f}",
                                 *[1 if state & (1 << i) else 0 for i in range(8)], *names, changed_text))
        self.status_var.set(f"Saved {path}")

    def _on_close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    try:
        if "vista" in ttk.Style(root).theme_names():
            ttk.Style(root).theme_use("vista")
    except tk.TclError:
        pass
    AnalyzerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
