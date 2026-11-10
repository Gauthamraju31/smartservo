#!/usr/bin/env python3
"""
ElectronBot SmartServo Visual Control & Configuration Studio
Uses Tkinter for a native, responsive desktop UI with live visuals.
"""

import sys
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Optional

from smartservo import SmartServoClient, SmartServo

class SmartServoApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ElectronBot SmartServo Studio (Seeed ESP32-S3)")
        self.root.geometry("820x860")
        self.root.minsize(760, 800)
        self.root.configure(bg="#0b0f19")

        self.client = SmartServoClient()
        self.servo: Optional[SmartServo] = None
        self.current_node_id = 6
        self.running = True

        # State
        self.max_angle = 180.0
        self.target_angle = 90.0
        self.actual_angle = 0.0
        self.actual_velocity = 0.0
        self.raw_adc = 0.0
        self.is_motor_enabled = False
        self.is_sweeping = False
        self.sweep_direction = 1
        self._updating_slider = False
        self._pending_target_angle: Optional[float] = None
        self._last_slider_send = 0.0
        self.invert_var = tk.BooleanVar(value=True)

        # Waveform History (100 points)
        self.hist_len = 100
        self.target_hist = [90.0] * self.hist_len
        self.actual_hist = [0.0] * self.hist_len

        self._init_theme()
        self._build_ui()
        self._start_telemetry_thread()

    def _init_theme(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background="#0b0f19", foreground="#e0e7ff")
        style.configure("TLabel", background="#0b0f19", foreground="#e0e7ff", font=("Segoe UI", 9))
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"), foreground="#00f0ff")
        style.configure("Sub.TLabel", font=("Segoe UI", 8), foreground="#8494b8")
        style.configure("TButton", font=("Segoe UI", 9, "bold"))
        style.configure("TCombobox", fieldbackground="#151d30", background="#233152", foreground="#fff")
        style.configure("TEntry", fieldbackground="#151d30", foreground="#fff")

    def _build_ui(self):
        # 1. Connection Toolbar
        top = tk.Frame(self.root, bg="#151d30", bd=1, relief="ridge", padx=12, pady=8)
        top.pack(fill="x", padx=12, pady=6)

        ttk.Label(top, text="Port:").pack(side="left")
        self.cb_ports = ttk.Combobox(top, width=16)
        self.cb_ports.pack(side="left", padx=6)
        self._refresh_ports()

        tk.Button(top, text="⟳", bg="#1c2740", fg="#fff", width=3, command=self._refresh_ports).pack(side="left", padx=2)
        
        self.btn_connect = tk.Button(top, text="Connect", bg="#00f0ff", fg="#000", font=("Segoe UI", 9, "bold"), command=self._toggle_connection)
        self.btn_connect.pack(side="left", padx=8)

        ttk.Label(top, text="Servo Node ID:").pack(side="left", padx=(12, 4))
        self.ent_node_id = ttk.Entry(top, width=5)
        self.ent_node_id.insert(0, "6")
        self.ent_node_id.pack(side="left")

        tk.Button(top, text="Scan Bus", bg="#1c2740", fg="#00ff88", font=("Segoe UI", 9), command=self._scan_bus).pack(side="left", padx=6)

        self.lbl_status = tk.Label(top, text="● Disconnected", fg="#ff0055", bg="#151d30", font=("Segoe UI", 9, "bold"))
        self.lbl_status.pack(side="right", padx=6)

        # 2. Main Visual Panel (Gauge + Scope side-by-side)
        visual_frame = tk.Frame(self.root, bg="#0b0f19")
        visual_frame.pack(fill="x", padx=12, pady=4)

        # Gauge Card
        gauge_card = tk.Frame(visual_frame, bg="#151d30", bd=1, relief="ridge", padx=12, pady=10)
        gauge_card.pack(side="left", fill="both", expand=True, padx=(0, 6))

        ttk.Label(gauge_card, text="Joint Position Gauge", style="Header.TLabel").pack(anchor="w")
        self.gauge_canvas = tk.Canvas(gauge_card, width=280, height=210, bg="#151d30", highlightthickness=0)
        self.gauge_canvas.pack(pady=4)

        self.lbl_gauge_text = tk.Label(gauge_card, text="0.0°", fg="#00f0ff", bg="#151d30", font=("Segoe UI", 20, "bold"))
        self.lbl_gauge_text.pack()

        self.lbl_gauge_sub = tk.Label(gauge_card, text="Target: 90.0° | Err: 0.0°", fg="#8494b8", bg="#151d30", font=("Segoe UI", 9))
        self.lbl_gauge_sub.pack(pady=(0, 4))

        # Scope Card
        scope_card = tk.Frame(visual_frame, bg="#151d30", bd=1, relief="ridge", padx=12, pady=10)
        scope_card.pack(side="right", fill="both", expand=True, padx=(6, 0))

        scope_hdr = tk.Frame(scope_card, bg="#151d30")
        scope_hdr.pack(fill="x")
        ttk.Label(scope_hdr, text="Real-Time Step Response", style="Header.TLabel").pack(side="left")
        tk.Label(scope_hdr, text="Gold: Target  |  Cyan: Actual", fg="#ffb700", bg="#151d30", font=("Segoe UI", 8)).pack(side="right")

        self.scope_canvas = tk.Canvas(scope_card, width=380, height=210, bg="#0c1220", highlightthickness=0)
        self.scope_canvas.pack(fill="both", expand=True, pady=6)

        self.lbl_vel = tk.Label(scope_card, text="Estimated Velocity: 0.0 deg/s", fg="#e0e7ff", bg="#151d30", font=("Segoe UI", 9))
        self.lbl_vel.pack()

        # 3. Motion Control Panel
        ctrl_card = tk.Frame(self.root, bg="#151d30", bd=1, relief="ridge", padx=14, pady=10)
        ctrl_card.pack(fill="x", padx=12, pady=6)

        ttk.Label(ctrl_card, text="Servo Motion Controls", style="Header.TLabel").pack(anchor="w")

        # Angle Slider
        slider_row = tk.Frame(ctrl_card, bg="#151d30")
        slider_row.pack(fill="x", pady=6)
        ttk.Label(slider_row, text="Target Angle:").pack(side="left")
        self.lbl_slider_val = tk.Label(slider_row, text=f"{self.target_angle:.1f}°", fg="#00f0ff", bg="#151d30", font=("Segoe UI", 11, "bold"), width=8)
        self.lbl_slider_val.pack(side="left", padx=4)

        self.angle_slider = ttk.Scale(slider_row, from_=0, to=self.max_angle, orient="horizontal", command=self._on_slider_drag)
        self.angle_slider.set(self.target_angle)
        self.angle_slider.bind("<ButtonRelease-1>", lambda e: self._on_slider_release())
        self.angle_slider.pack(side="left", fill="x", expand=True, padx=8)

        # Preset & Nudge Buttons
        self.preset_frame = tk.Frame(ctrl_card, bg="#151d30")
        self.preset_frame.pack(fill="x", pady=4)
        self._update_preset_buttons()

        # Dynamic Limits (Torque & Velocity)
        limits_row = tk.Frame(ctrl_card, bg="#151d30")
        limits_row.pack(fill="x", pady=6)

        ttk.Label(limits_row, text="Torque Limit (0-100%):").pack(side="left")
        self.t_slider = ttk.Scale(limits_row, from_=0.0, to=1.0, orient="horizontal", value=0.90, command=self._on_torque_slider)
        self.t_slider.pack(side="left", fill="x", expand=True, padx=6)
        self.lbl_torque_val = tk.Label(limits_row, text="90%", fg="#fff", bg="#151d30", width=5)
        self.lbl_torque_val.pack(side="left")

        ttk.Label(limits_row, text="Speed Limit:").pack(side="left", padx=(10, 0))
        self.v_slider = ttk.Scale(limits_row, from_=0, to=500, orient="horizontal", value=0, command=self._on_vel_slider)
        self.v_slider.pack(side="left", fill="x", expand=True, padx=6)

        # Action Buttons (Enable, E-Stop, Sweep)
        act_row = tk.Frame(ctrl_card, bg="#151d30")
        act_row.pack(fill="x", pady=(8, 2))

        self.btn_enable = tk.Button(act_row, text="Enable Motor", bg="#00ff88", fg="#000", font=("Segoe UI", 10, "bold"), command=self._toggle_enable)
        self.btn_enable.pack(side="left", fill="x", expand=True, padx=4)

        self.btn_sweep = tk.Button(act_row, text="Start Auto Sweep", bg="#1c2740", fg="#00f0ff", font=("Segoe UI", 10), command=self._toggle_sweep)
        self.btn_sweep.pack(side="left", fill="x", expand=True, padx=4)

        tk.Button(act_row, text="E-Stop", bg="#ff0055", fg="#fff", font=("Segoe UI", 10, "bold"), command=self._estop).pack(side="left", fill="x", expand=True, padx=4)

        # 4. Configuration & Flash Tuning Card
        cfg_card = tk.Frame(self.root, bg="#151d30", bd=1, relief="ridge", padx=14, pady=10)
        cfg_card.pack(fill="both", expand=True, padx=12, pady=6)

        ttk.Label(cfg_card, text="EEPROM Closed-Loop Gain Tuning & Setup", style="Header.TLabel").pack(anchor="w")

        # PID entries
        pid_grid = tk.Frame(cfg_card, bg="#151d30")
        pid_grid.pack(fill="x", pady=6)

        ttk.Label(pid_grid, text="Kp (Position):").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        self.e_kp = ttk.Entry(pid_grid, width=8); self.e_kp.insert(0, "35.0"); self.e_kp.grid(row=0, column=1, padx=4)

        ttk.Label(pid_grid, text="Ki (Integral):").grid(row=0, column=2, padx=4, pady=2, sticky="w")
        self.e_ki = ttk.Entry(pid_grid, width=8); self.e_ki.insert(0, "0.20"); self.e_ki.grid(row=0, column=3, padx=4)

        ttk.Label(pid_grid, text="Kv (Velocity):").grid(row=0, column=4, padx=4, pady=2, sticky="w")
        self.e_kv = ttk.Entry(pid_grid, width=8); self.e_kv.insert(0, "0.0"); self.e_kv.grid(row=0, column=5, padx=4)

        ttk.Label(pid_grid, text="Kd (Damping):").grid(row=0, column=6, padx=4, pady=2, sticky="w")
        self.e_kd = ttk.Entry(pid_grid, width=8); self.e_kd.insert(0, "40.0"); self.e_kd.grid(row=0, column=7, padx=4)

        # Setup entries
        opt_grid = tk.Frame(cfg_card, bg="#151d30")
        opt_grid.pack(fill="x", pady=4)

        ttk.Label(opt_grid, text="Startup Boot Angle (°):").grid(row=0, column=0, padx=4, sticky="w")
        self.e_init_pos = ttk.Entry(opt_grid, width=8); self.e_init_pos.insert(0, "90.0"); self.e_init_pos.grid(row=0, column=1, padx=4)

        ttk.Label(opt_grid, text="Reassign I2C Address:").grid(row=0, column=2, padx=(16, 4), sticky="w")
        self.e_new_id = ttk.Entry(opt_grid, width=8); self.e_new_id.insert(0, "6"); self.e_new_id.grid(row=0, column=3, padx=4)

        self.chk_invert = tk.Checkbutton(
            opt_grid,
            text="Invert Motor Polarity (Reverse)",
            variable=self.invert_var,
            command=self._on_invert_toggle,
            bg="#151d30",
            fg="#00f0ff",
            selectcolor="#0c1220",
            activebackground="#151d30",
            activeforeground="#00f0ff",
            font=("Segoe UI", 9, "bold")
        )
        self.chk_invert.grid(row=0, column=4, padx=(20, 4), sticky="w")

        # Row 1: Max Angle Configuration (180 vs 360) & Live ADC Readout
        ttk.Label(opt_grid, text="Max Angle Range (°):").grid(row=1, column=0, padx=4, pady=(6, 2), sticky="w")
        self.e_max_angle = ttk.Entry(opt_grid, width=8)
        self.e_max_angle.insert(0, f"{self.max_angle:.0f}")
        self.e_max_angle.grid(row=1, column=1, padx=4, pady=(6, 2))

        mode_box = tk.Frame(opt_grid, bg="#151d30")
        mode_box.grid(row=1, column=2, columnspan=2, padx=(16, 4), pady=(6, 2), sticky="w")
        tk.Button(mode_box, text="180° Mode", bg="#233152", fg="#00f0ff", font=("Segoe UI", 8), command=lambda: self._set_max_angle_ui(180.0)).pack(side="left", padx=2)
        tk.Button(mode_box, text="360° Mode", bg="#233152", fg="#00f0ff", font=("Segoe UI", 8), command=lambda: self._set_max_angle_ui(360.0)).pack(side="left", padx=2)

        self.lbl_raw_adc = tk.Label(opt_grid, text="Raw ADC: ----", fg="#ffb700", bg="#151d30", font=("Segoe UI", 9, "bold"))
        self.lbl_raw_adc.grid(row=1, column=4, padx=(20, 4), pady=(6, 2), sticky="w")

        # Config buttons
        cfg_btns = tk.Frame(cfg_card, bg="#151d30")
        cfg_btns.pack(fill="x", pady=8)

        tk.Button(cfg_btns, text="Commit PID to Flash", bg="#7000ff", fg="#fff", font=("Segoe UI", 9, "bold"), command=self._save_pid).pack(side="left", padx=3)
        tk.Button(cfg_btns, text="Save Startup Angle", bg="#1c2740", fg="#fff", command=self._save_init_pos).pack(side="left", padx=3)
        tk.Button(cfg_btns, text="Save Max Angle", bg="#1c2740", fg="#00f0ff", font=("Segoe UI", 9, "bold"), command=self._save_max_angle).pack(side="left", padx=3)
        tk.Button(cfg_btns, text="Calib 0° Min ADC", bg="#1c2740", fg="#8494b8", command=self._calib_min_adc).pack(side="left", padx=3)
        tk.Button(cfg_btns, text="Calib Max° ADC", bg="#1c2740", fg="#8494b8", command=self._calib_max_adc).pack(side="left", padx=3)
        tk.Button(cfg_btns, text="Apply New Node ID", bg="#1c2740", fg="#ffb700", command=self._reassign_id).pack(side="left", padx=3)

    # UI Drawing: Analog Circular Gauge (Supports 180° Semicircle & 360° Full Dial)
    def _draw_gauge(self, target: float, actual: float):
        c = self.gauge_canvas
        c.delete("all")

        if self.max_angle <= 180.0:
            # 180-degree Semicircle dial
            cx, cy, r = 140, 155, 115
            c.create_arc(cx-r, cy-r, cx+r, cy+r, start=0, extent=180, style="arc", outline="#1e2942", width=14)

            for deg in range(0, int(self.max_angle) + 1, 30):
                frac = deg / self.max_angle
                rad = math.radians(180 - frac * 180)
                x1 = cx + (r - 18) * math.cos(rad)
                y1 = cy - (r - 18) * math.sin(rad)
                x2 = cx + (r - 28) * math.cos(rad)
                y2 = cy - (r - 28) * math.sin(rad)
                c.create_line(x1, y1, x2, y2, fill="#3b4d75", width=2)
                xt = cx + (r - 40) * math.cos(rad)
                yt = cy - (r - 40) * math.sin(rad)
                c.create_text(xt, yt, text=f"{deg}°", fill="#8494b8", font=("Segoe UI", 7))

            # Target Needle (Gold Dashed)
            t_frac = min(1.0, max(0.0, target / self.max_angle))
            t_rad = math.radians(180 - t_frac * 180)
            tx = cx + (r - 16) * math.cos(t_rad)
            ty = cy - (r - 16) * math.sin(t_rad)
            c.create_line(cx, cy, tx, ty, fill="#ffb700", width=3, dash=(4, 2))

            # Actual Needle (Cyan Glow)
            a_frac = min(1.0, max(0.0, actual / self.max_angle))
            a_rad = math.radians(180 - a_frac * 180)
            ax = cx + (r - 10) * math.cos(a_rad)
            ay = cy - (r - 10) * math.sin(a_rad)
            c.create_line(cx, cy, ax, ay, fill="#00f0ff", width=4)
            c.create_oval(cx-10, cy-10, cx+10, cy+10, fill="#00f0ff", outline="")
        else:
            # 360-degree Full Compass Dial
            cx, cy, r = 140, 105, 88
            c.create_oval(cx-r, cy-r, cx+r, cy+r, outline="#1e2942", width=14)

            # Ticks every 45 degrees
            for deg in range(0, 360, 45):
                rad = math.radians(deg - 90)  # 0 deg at 12 o'clock
                x1 = cx + (r - 12) * math.cos(rad)
                y1 = cy + (r - 12) * math.sin(rad)
                x2 = cx + (r - 22) * math.cos(rad)
                y2 = cy + (r - 22) * math.sin(rad)
                c.create_line(x1, y1, x2, y2, fill="#3b4d75", width=2)
                xt = cx + (r - 34) * math.cos(rad)
                yt = cy + (r - 34) * math.sin(rad)
                c.create_text(xt, yt, text=f"{deg}°", fill="#8494b8", font=("Segoe UI", 7))

            # Target Needle (Gold Dashed)
            t_frac = min(1.0, max(0.0, target / self.max_angle))
            t_rad = math.radians(t_frac * 360.0 - 90)
            tx = cx + (r - 14) * math.cos(t_rad)
            ty = cy + (r - 14) * math.sin(t_rad)
            c.create_line(cx, cy, tx, ty, fill="#ffb700", width=3, dash=(4, 2))

            # Actual Needle (Cyan Glow)
            a_frac = min(1.0, max(0.0, actual / self.max_angle))
            a_rad = math.radians(a_frac * 360.0 - 90)
            ax = cx + (r - 10) * math.cos(a_rad)
            ay = cy + (r - 10) * math.sin(a_rad)
            c.create_line(cx, cy, ax, ay, fill="#00f0ff", width=4)
            c.create_oval(cx-8, cy-8, cx+8, cy+8, fill="#00f0ff", outline="")

    # UI Drawing: Waveform Scope (Dynamic Vertical Scaling)
    def _draw_scope(self):
        c = self.scope_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50: return

        # Grid lines based on max_angle
        grid_degs = [0, 45, 90, 135, 180] if self.max_angle <= 180.0 else [0, 90, 180, 270, 360]
        for y_deg in grid_degs:
            y_px = h - (y_deg / self.max_angle) * (h - 20) - 10
            c.create_line(0, y_px, w, y_px, fill="#162035", width=1)
            c.create_text(20, y_px - 6, text=f"{y_deg}°", fill="#3b4d75", font=("Segoe UI", 7))

        # Target Waveform (Gold)
        target_pts = []
        for i, val in enumerate(self.target_hist):
            x = (i / (self.hist_len - 1)) * w
            y = h - (val / self.max_angle) * (h - 20) - 10
            target_pts.extend([x, y])
        if len(target_pts) >= 4:
            c.create_line(target_pts, fill="#ffb700", width=1.5, dash=(4, 2))

        # Actual Waveform (Cyan)
        actual_pts = []
        for i, val in enumerate(self.actual_hist):
            x = (i / (self.hist_len - 1)) * w
            y = h - (val / self.max_angle) * (h - 20) - 10
            actual_pts.extend([x, y])
        if len(actual_pts) >= 4:
            c.create_line(actual_pts, fill="#00f0ff", width=2)

    # Serial & Connections
    def _refresh_ports(self):
        ports = SmartServoClient.list_ports()
        self.cb_ports['values'] = ports
        if ports and not self.cb_ports.get():
            for p in ports:
                if "ACM" in p or "USB" in p:
                    self.cb_ports.set(p)
                    break
            else:
                self.cb_ports.set(ports[0])

    def _toggle_connection(self):
        if self.client.is_connected():
            self.client.disconnect()
            self.servo = None
            self.btn_connect.config(text="Connect", bg="#00f0ff", fg="#000")
            self.lbl_status.config(text="● Disconnected", fg="#ff0055")
        else:
            port = self.cb_ports.get()
            if not port:
                messagebox.showwarning("Warning", "Please select a serial port!")
                return
            try:
                self.client.connect(port)
                self.current_node_id = int(self.ent_node_id.get())
                self.servo = self.client.get_servo(self.current_node_id)
                self.btn_connect.config(text="Disconnect", bg="#ff0055", fg="#fff")
                self.lbl_status.config(text=f"● Online (ID {self.current_node_id})", fg="#00ff88")
                # Query current polarity, max angle, and PID gains from servo Flash
                try:
                    inv = self.servo.get_invert()
                    if inv is not None:
                        self.invert_var.set(inv)
                    ma = self.servo.get_max_angle()
                    if ma is not None and ma >= 90.0:
                        self._apply_max_angle_ui(ma)
                    pid = self.servo.get_pid()
                    if pid is not None:
                        kp, ki, kv, kd = pid
                        self.e_kp.delete(0, tk.END); self.e_kp.insert(0, f"{kp:.2f}")
                        self.e_ki.delete(0, tk.END); self.e_ki.insert(0, f"{ki:.2f}")
                        self.e_kv.delete(0, tk.END); self.e_kv.insert(0, f"{kv:.2f}")
                        self.e_kd.delete(0, tk.END); self.e_kd.insert(0, f"{kd:.2f}")
                except Exception:
                    pass
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def _get_active_servo(self) -> Optional[SmartServo]:
        if not self.client.is_connected():
            return None
        if not self.servo or self.servo.node_id != self.current_node_id:
            self.servo = self.client.get_servo(self.current_node_id)
        return self.servo

    def _sync_slider_ui(self, angle: float):
        self._updating_slider = True
        try:
            self.angle_slider.set(angle)
        finally:
            self._updating_slider = False
        self.lbl_slider_val.config(text=f"{angle:.1f}°")

    # Motion Handlers
    def _on_slider_drag(self, val):
        if self._updating_slider:
            return
        try:
            angle = float(val)
        except ValueError:
            return
        self.target_angle = max(0.0, min(self.max_angle, angle))
        self.lbl_slider_val.config(text=f"{self.target_angle:.1f}°")
        if self.is_motor_enabled:
            self._pending_target_angle = self.target_angle

    def _on_slider_release(self):
        if self.is_motor_enabled:
            self._pending_target_angle = self.target_angle

    def _set_target(self, angle: float):
        self.target_angle = max(0.0, min(self.max_angle, angle))
        self._sync_slider_ui(self.target_angle)
        if self.is_motor_enabled:
            self._pending_target_angle = self.target_angle

    def _nudge(self, delta: float):
        self._set_target(self.target_angle + delta)

    def _on_torque_slider(self, val):
        t = float(val)
        self.lbl_torque_val.config(text=f"{int(t*100)}%")
        s = self._get_active_servo()
        if s:
            s.set_torque(t)

    def _on_vel_slider(self, val):
        s = self._get_active_servo()
        if s:
            s.set_velocity(float(val))

    def _toggle_enable(self):
        s = self._get_active_servo()
        if not s: return
        new_state = not self.is_motor_enabled
        actual = s.set_enable(new_state)
        if actual is None:
            messagebox.showerror("Communication Error", "Failed to communicate with servo motor.\nPlease check I2C wiring and power.")
            return
        self.is_motor_enabled = new_state
        if 0.0 <= actual <= 360.0:
            self.actual_angle = actual
        if self.is_motor_enabled:
            self.btn_enable.config(text="Disable Motor", bg="#ff0055", fg="#fff")
            # Sync target angle to current actual angle on enable so slider and motor are in sync
            if self.actual_angle > 0.0:
                self.target_angle = self.actual_angle
                self._pending_target_angle = None
                self._sync_slider_ui(self.target_angle)
        else:
            self.btn_enable.config(text="Enable Motor", bg="#00ff88", fg="#000")
            self._pending_target_angle = None

    def _estop(self):
        self._pending_target_angle = None
        s = self._get_active_servo()
        if s:
            s.set_enable(False)
        self.is_motor_enabled = False
        self.is_sweeping = False
        self.btn_enable.config(text="Enable Motor", bg="#00ff88", fg="#000")
        self.btn_sweep.config(text="Start Auto Sweep", bg="#1c2740")

    def _toggle_sweep(self):
        self.is_sweeping = not self.is_sweeping
        if self.is_sweeping:
            self.btn_sweep.config(text="Stop Auto Sweep", bg="#ffb700", fg="#000")
            if not self.is_motor_enabled:
                self._toggle_enable()
        else:
            self.btn_sweep.config(text="Start Auto Sweep", bg="#1c2740", fg="#00f0ff")

    # Configuration Handlers
    def _scan_bus(self):
        if not self.client.is_connected():
            messagebox.showwarning("Notice", "Connect serial port first!")
            return
        devs = self.client.scan_bus()
        if not devs:
            messagebox.showinfo("Scan Results", "No I2C devices responded on the bus.\nCheck wiring, power, and pullup resistors.")
        else:
            res_str = ", ".join([f"0x{d:02X} ({d})" for d in devs])
            messagebox.showinfo("Scan Results", f"Detected {len(devs)} device(s):\n{res_str}")
            if devs:
                self.ent_node_id.delete(0, tk.END)
                self.ent_node_id.insert(0, str(devs[0]))

    def _save_pid(self):
        s = self._get_active_servo()
        if not s: return
        try:
            kp = float(self.e_kp.get())
            ki = float(self.e_ki.get())
            kv = float(self.e_kv.get())
            kd = float(self.e_kd.get())
            if s.set_pid(kp, ki, kv, kd):
                messagebox.showinfo("Success", "PID gains written & saved to servo Flash!")
            else:
                messagebox.showerror("Error", "Servo did not acknowledge PID command.")
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric values in PID fields.")

    def _save_init_pos(self):
        s = self._get_active_servo()
        if not s: return
        try:
            pos = float(self.e_init_pos.get())
            if s.set_init_pos(pos):
                messagebox.showinfo("Success", f"Default startup position ({pos}°) saved!")
            else:
                messagebox.showerror("Error", "Command failed.")
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric value.")

    def _reassign_id(self):
        s = self._get_active_servo()
        if not s: return
        try:
            new_id = int(self.e_new_id.get())
            if messagebox.askyesno("Confirm", f"Change servo node ID from {s.node_id} to {new_id}?"):
                if s.change_node_id(new_id):
                    self.ent_node_id.delete(0, tk.END)
                    self.ent_node_id.insert(0, str(new_id))
                    messagebox.showinfo("Success", f"Node ID updated to {new_id}!")
                else:
                    messagebox.showerror("Error", "Failed to change node ID.")
        except ValueError:
            messagebox.showerror("Error", "Invalid Node ID.")

    def _on_invert_toggle(self):
        s = self._get_active_servo()
        if not s:
            messagebox.showwarning("Notice", "Connect to serial and servo before changing polarity.")
            return
        inv = self.invert_var.get()
        if s.set_invert(inv):
            state_str = "Inverted (Reversed)" if inv else "Normal"
            messagebox.showinfo("Polarity Updated", f"Motor direction set to {state_str} and saved to Flash!")
        else:
            messagebox.showerror("Error", "Servo did not acknowledge polarity inversion command.")
            self.invert_var.set(not inv)

    def _update_preset_buttons(self):
        for widget in self.preset_frame.winfo_children():
            widget.destroy()

        for nudge in [-5, -1]:
            tk.Button(self.preset_frame, text=f"{nudge}°", bg="#1c2740", fg="#fff", width=5,
                      command=lambda n=nudge: self._nudge(n)).pack(side="left", padx=2)

        presets = [0, 45, 90, 135, 180] if self.max_angle <= 180.0 else [0, 90, 180, 270, 360]
        for preset in presets:
            tk.Button(self.preset_frame, text=f"{preset}°", bg="#233152", fg="#00f0ff", width=6,
                      command=lambda p=preset: self._set_target(p)).pack(side="left", expand=True, padx=2)

        for nudge in [1, 5]:
            tk.Button(self.preset_frame, text=f"+{nudge}°", bg="#1c2740", fg="#fff", width=5,
                      command=lambda n=nudge: self._nudge(n)).pack(side="left", padx=2)

    def _set_max_angle_ui(self, max_angle: float):
        self._apply_max_angle_ui(max_angle)

    def _apply_max_angle_ui(self, max_angle: float):
        self.max_angle = float(max_angle)
        self.e_max_angle.delete(0, tk.END)
        self.e_max_angle.insert(0, f"{self.max_angle:.0f}")
        self.angle_slider.configure(to=self.max_angle)
        if self.target_angle > self.max_angle:
            self._set_target(self.max_angle)
        self._update_preset_buttons()
        self._update_visuals()

    def _save_max_angle(self):
        s = self._get_active_servo()
        if not s:
            messagebox.showwarning("Notice", "Connect to servo before saving.")
            return
        try:
            val = float(self.e_max_angle.get())
            if val < 90.0 or val > 360.0:
                messagebox.showerror("Invalid Range", "Max angle must be between 90° and 360°.")
                return
            if s.set_max_angle(val):
                self._apply_max_angle_ui(val)
                messagebox.showinfo("Success", f"Max mechanical angle set to {val:.1f}° and committed to Flash!")
            else:
                messagebox.showerror("Error", "Servo did not acknowledge set_max_angle command.")
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric value in Max Angle field.")

    def _calib_min_adc(self):
        s = self._get_active_servo()
        if not s:
            messagebox.showwarning("Notice", "Connect to servo before calibrating.")
            return
        raw = s.get_raw_adc()
        if raw is not None:
            if messagebox.askyesno("Confirm 0° Calibration", f"Current Raw ADC is {raw:.0f}.\nSave this as 0° (Minimum) ADC calibration to Flash?"):
                if s.set_adc_min(raw):
                    messagebox.showinfo("Success", f"0° Min ADC set to {raw:.0f} and saved to Flash!")
                else:
                    messagebox.showerror("Error", "Command failed.")

    def _calib_max_adc(self):
        s = self._get_active_servo()
        if not s:
            messagebox.showwarning("Notice", "Connect to servo before calibrating.")
            return
        raw = s.get_raw_adc()
        if raw is not None:
            if messagebox.askyesno("Confirm Max° Calibration", f"Current Raw ADC is {raw:.0f}.\nSave this as {self.max_angle:.0f}° (Maximum) ADC calibration to Flash?"):
                if s.set_adc_max(raw):
                    messagebox.showinfo("Success", f"Max ADC set to {raw:.0f} and saved to Flash!")
                else:
                    messagebox.showerror("Error", "Command failed.")

    # Background Telemetry & Visual Update Thread
    def _start_telemetry_thread(self):
        def worker():
            last_sweep_time = time.time()
            poll_cnt = 0
            consecutive_drops = 0
            while self.running:
                try:
                    s = self._get_active_servo()
                    if s and self.client.is_connected():
                        # Priority 1: Pending Motion Target (Fast, non-blocking navigation)
                        if self._pending_target_angle is not None and self.is_motor_enabled:
                            cmd_ang = self._pending_target_angle
                            self._pending_target_angle = None
                            ang_resp = s.set_angle(cmd_ang)
                            if ang_resp is not None and 0.0 <= ang_resp <= 360.0:
                                self.actual_angle = ang_resp
                                consecutive_drops = 0
                            else:
                                consecutive_drops += 1
                            self.target_hist.append(self.target_angle)
                            self.target_hist.pop(0)
                            self.actual_hist.append(self.actual_angle)
                            self.actual_hist.pop(0)
                            self.root.after(0, self._update_visuals)
                            if consecutive_drops >= 3:
                                self.client.scan_bus()
                                consecutive_drops = 0
                            time.sleep(0.02)
                            continue

                        # Priority 2: Auto Sweep Mode (Direct step execution)
                        if self.is_sweeping and (time.time() - last_sweep_time >= 0.06):
                            last_sweep_time = time.time()
                            next_ang = self.target_angle + self.sweep_direction * 3.0
                            if next_ang >= (self.max_angle - 5.0):
                                next_ang = self.max_angle - 5.0
                                self.sweep_direction = -1
                            elif next_ang <= 5.0:
                                next_ang = 5.0
                                self.sweep_direction = 1
                            self.target_angle = next_ang
                            ang_resp = s.set_angle(next_ang)
                            if ang_resp is not None and 0.0 <= ang_resp <= 360.0:
                                self.actual_angle = ang_resp
                                consecutive_drops = 0
                            else:
                                consecutive_drops += 1
                            self.target_hist.append(self.target_angle)
                            self.target_hist.pop(0)
                            self.actual_hist.append(self.actual_angle)
                            self.actual_hist.pop(0)
                            self.root.after(0, lambda a=next_ang: self._sync_slider_ui(a))
                            self.root.after(0, self._update_visuals)
                            if consecutive_drops >= 3:
                                self.client.scan_bus()
                                consecutive_drops = 0
                            time.sleep(0.02)
                            continue

                        # Priority 3: Telemetry polling (Angle, Velocity, Raw ADC)
                        ang = s.get_angle()
                        if ang is not None and 0.0 <= ang <= 360.0:
                            self.actual_angle = ang
                            consecutive_drops = 0
                            poll_cnt += 1
                            # Poll velocity every 5 cycles
                            if poll_cnt % 5 == 0:
                                vel = s.get_velocity()
                                if vel is not None and abs(vel) < 1000.0:
                                    self.actual_velocity = vel

                            # Poll raw ADC every 10 cycles
                            if poll_cnt % 10 == 0:
                                raw = s.get_raw_adc()
                                if raw is not None and 0.0 <= raw <= 4095.0:
                                    self.raw_adc = raw
                                    self.root.after(0, lambda r=raw: self.lbl_raw_adc.config(text=f"Raw ADC: {r:.0f}"))
                        else:
                            consecutive_drops += 1
                            if consecutive_drops >= 3:
                                self.client.scan_bus()
                                consecutive_drops = 0

                        # Keep UI visuals updating smoothly even if individual polls drop
                        self.target_hist.append(self.target_angle)
                        self.target_hist.pop(0)
                        self.actual_hist.append(self.actual_angle)
                        self.actual_hist.pop(0)
                        self.root.after(0, self._update_visuals)
                except Exception:
                    pass
                time.sleep(0.035) # ~28 Hz update loop

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def _update_visuals(self):
        try:
            self._draw_gauge(self.target_angle, self.actual_angle)
            self._draw_scope()
            self.lbl_gauge_text.config(text=f"{self.actual_angle:.1f}°")
            err = self.actual_angle - self.target_angle
            self.lbl_gauge_sub.config(text=f"Target: {self.target_angle:.1f}° | Err: {err:+.1f}° | 0-{self.max_angle:.0f}°")
            self.lbl_vel.config(text=f"Estimated Velocity: {self.actual_velocity:.1f} deg/s")
        except Exception:
            pass

    def on_closing(self):
        self.running = False
        if self.client.is_connected():
            self.client.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SmartServoApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

