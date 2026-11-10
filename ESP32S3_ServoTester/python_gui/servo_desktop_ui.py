#!/usr/bin/env python3
"""
ElectronBot SmartServo Desktop GUI Tester
Communicates with the Seeed Studio XIAO ESP32-S3 over USB Serial.
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

class ServoDesktopUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ElectronBot SmartServo GUI Studio")
        self.root.geometry("640x720")
        self.root.configure(bg="#0b0f19")

        self.ser = None
        self.running = True
        self.target_angle = 90.0
        self.actual_angle = 0.0
        self.is_connected = False

        self._init_styles()
        self._build_ui()
        self._start_serial_thread()

    def _init_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background="#0b0f19", foreground="#e0e7ff")
        style.configure("TLabel", background="#0b0f19", foreground="#e0e7ff", font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI", 14, "bold"), foreground="#00f0ff")
        style.configure("TButton", font=("Segoe UI", 9, "bold"), padding=5)

    def _build_ui(self):
        # Header / Port selector
        top_frame = tk.Frame(self.root, bg="#151d30", bd=1, relief="ridge", padx=10, pady=8)
        top_frame.pack(fill="x", padx=12, pady=8)

        ttk.Label(top_frame, text="Serial Port:").pack(side="left")
        self.port_cb = ttk.Combobox(top_frame, width=15)
        self.port_cb.pack(side="left", padx=6)
        self._refresh_ports()

        ttk.Button(top_frame, text="⟳", width=3, command=self._refresh_ports).pack(side="left", padx=2)
        self.btn_connect = ttk.Button(top_frame, text="Connect", command=self._toggle_connection)
        self.btn_connect.pack(side="left", padx=8)

        self.lbl_status = tk.Label(top_frame, text="● Offline", fg="#ff0055", bg="#151d30", font=("Segoe UI", 10, "bold"))
        self.lbl_status.pack(side="right", padx=6)

        # Visual Gauge (Canvas)
        gauge_frame = tk.Frame(self.root, bg="#151d30", bd=1, relief="ridge", padx=10, pady=10)
        gauge_frame.pack(fill="x", padx=12, pady=6)

        self.canvas = tk.Canvas(gauge_frame, width=280, height=160, bg="#151d30", highlightthickness=0)
        self.canvas.pack()
        self._draw_gauge(90, 0)

        self.lbl_angle_readout = tk.Label(gauge_frame, text="Actual: 0.0° | Target: 90.0°", fg="#00f0ff", bg="#151d30", font=("Segoe UI", 12, "bold"))
        self.lbl_angle_readout.pack(pady=4)

        # Slider & Presets
        ctrl_frame = tk.Frame(self.root, bg="#151d30", bd=1, relief="ridge", padx=12, pady=10)
        ctrl_frame.pack(fill="x", padx=12, pady=6)

        ttk.Label(ctrl_frame, text="Target Angle Control (0° - 180°):", style="Header.TLabel").pack(anchor="w")
        
        self.angle_slider = ttk.Scale(ctrl_frame, from_=0, to=180, orient="horizontal", command=self._on_slider)
        self.angle_slider.set(90)
        self.angle_slider.pack(fill="x", pady=8)

        btn_grid = tk.Frame(ctrl_frame, bg="#151d30")
        btn_grid.pack(fill="x", pady=4)
        for val in [0, 45, 90, 135, 180]:
            b = tk.Button(btn_grid, text=f"{val}°", bg="#1c2740", fg="#fff", width=5, command=lambda v=val: self._set_target(v))
            b.pack(side="left", expand=True, padx=2)

        # Actions (Enable / E-Stop)
        act_frame = tk.Frame(ctrl_frame, bg="#151d30")
        act_frame.pack(fill="x", pady=8)
        self.btn_en = tk.Button(act_frame, text="Enable Motor", bg="#00ff88", fg="#000", font=("Segoe UI", 10, "bold"), command=self._toggle_enable)
        self.btn_en.pack(side="left", fill="x", expand=True, padx=4)

        b_stop = tk.Button(act_frame, text="E-Stop", bg="#ff0055", fg="#fff", font=("Segoe UI", 10, "bold"), command=self._estop)
        b_stop.pack(side="right", fill="x", expand=True, padx=4)

        # Configuration Panel
        cfg_frame = tk.Frame(self.root, bg="#151d30", bd=1, relief="ridge", padx=12, pady=10)
        cfg_frame.pack(fill="both", expand=True, padx=12, pady=6)

        ttk.Label(cfg_frame, text="I2C Configuration & Tuning", style="Header.TLabel").pack(anchor="w", pady=4)

        f_id = tk.Frame(cfg_frame, bg="#151d30")
        f_id.pack(fill="x", pady=4)
        ttk.Label(f_id, text="Node ID:").pack(side="left")
        self.ent_id = ttk.Entry(f_id, width=6)
        self.ent_id.insert(0, "12")
        self.ent_id.pack(side="left", padx=6)
        tk.Button(f_id, text="Reassign ID", bg="#1c2740", fg="#fff", command=self._reassign_id).pack(side="left", padx=4)
        tk.Button(f_id, text="Scan I2C Bus", bg="#1c2740", fg="#00f0ff", command=self._scan_bus).pack(side="right")

        f_pid = tk.Frame(cfg_frame, bg="#151d30")
        f_pid.pack(fill="x", pady=6)
        ttk.Label(f_pid, text="Kp:").grid(row=0, column=0, padx=2)
        self.e_kp = ttk.Entry(f_pid, width=6); self.e_kp.insert(0, "10.0"); self.e_kp.grid(row=0, column=1, padx=4)
        ttk.Label(f_pid, text="Ki:").grid(row=0, column=2, padx=2)
        self.e_ki = ttk.Entry(f_pid, width=6); self.e_ki.insert(0, "0.0"); self.e_ki.grid(row=0, column=3, padx=4)
        ttk.Label(f_pid, text="Kv:").grid(row=0, column=4, padx=2)
        self.e_kv = ttk.Entry(f_pid, width=6); self.e_kv.insert(0, "0.0"); self.e_kv.grid(row=0, column=5, padx=4)
        ttk.Label(f_pid, text="Kd:").grid(row=0, column=6, padx=2)
        self.e_kd = ttk.Entry(f_pid, width=6); self.e_kd.insert(0, "50.0"); self.e_kd.grid(row=0, column=7, padx=4)

        tk.Button(cfg_frame, text="Save PID to Servo Flash", bg="#7000ff", fg="#fff", font=("Segoe UI", 9, "bold"), command=self._save_pid).pack(fill="x", pady=6)

    def _draw_gauge(self, target, actual):
        self.canvas.delete("all")
        cx, cy, r = 140, 140, 110
        import math
        # Background arc
        self.canvas.create_arc(cx-r, cy-r, cx+r, cy+r, start=0, extent=180, style="arc", outline="#233152", width=12)
        
        # Target needle (orange)
        trad = math.radians(180 - target)
        tx = cx + (r - 20) * math.cos(trad)
        ty = cy - (r - 20) * math.sin(trad)
        self.canvas.create_line(cx, cy, tx, ty, fill="#ffb700", width=3, dash=(4, 2))

        # Actual needle (cyan)
        arad = math.radians(180 - actual)
        ax = cx + (r - 10) * math.cos(arad)
        ay = cy - (r - 10) * math.sin(arad)
        self.canvas.create_line(cx, cy, ax, ay, fill="#00f0ff", width=4)
        self.canvas.create_oval(cx-8, cy-8, cx+8, cy+8, fill="#00f0ff", outline="")

    def _refresh_ports(self):
        if not serial: return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports and not self.port_cb.get():
            self.port_cb.set(ports[0])

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="● Offline", fg="#ff0055")
        else:
            if not serial:
                messagebox.showerror("Error", "pyserial not installed. Run: pip install pyserial")
                return
            port = self.port_cb.get()
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text="● Online", fg="#00ff88")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def _send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd_str + "\n").encode())
            except Exception:
                pass

    def _on_slider(self, val):
        self._set_target(float(val))

    def _set_target(self, angle):
        self.target_angle = angle
        self.angle_slider.set(angle)
        self._send_cmd(f"pos {angle:.1f}")

    def _toggle_enable(self):
        if self.btn_en.cget("text") == "Enable Motor":
            self._send_cmd("en 1")
            self.btn_en.config(text="Disable Motor", bg="#ff0055", fg="#fff")
        else:
            self._send_cmd("en 0")
            self.btn_en.config(text="Enable Motor", bg="#00ff88", fg="#000")

    def _estop(self):
        self._send_cmd("en 0")
        self.btn_en.config(text="Enable Motor", bg="#00ff88", fg="#000")

    def _reassign_id(self):
        new_id = self.ent_id.get().strip()
        if new_id:
            self._send_cmd(f"id {new_id}")
            messagebox.showinfo("Node ID", f"Sent command to set Node ID = {new_id}")

    def _scan_bus(self):
        self._send_cmd("scan")

    def _save_pid(self):
        kp = self.e_kp.get()
        ki = self.e_ki.get()
        kv = self.e_kv.get()
        kd = self.e_kd.get()
        # In ESP32-S3 serial command:
        self._send_cmd(f"pid {kp} {ki} {kv} {kd}")
        messagebox.showinfo("PID", "PID parameters sent!")

    def _start_serial_thread(self):
        def reader():
            while self.running:
                if self.ser and self.ser.is_open:
                    self._send_cmd("read")
                    try:
                        line = self.ser.readline().decode(errors="ignore").strip()
                        if "Angle:" in line:
                            # format: Angle: 90.00 deg | Velocity: 0.00 deg/s
                            parts = line.split("|")
                            ang_str = parts[0].replace("Angle:", "").replace("deg", "").strip()
                            self.actual_angle = float(ang_str)
                            self.root.after(0, self._update_gui)
                    except Exception:
                        pass
                time.sleep(0.08)

        t = threading.Thread(target=reader, daemon=True)
        t.start()

    def _update_gui(self):
        self._draw_gauge(self.target_angle, self.actual_angle)
        self.lbl_angle_readout.config(text=f"Actual: {self.actual_angle:.1f}° | Target: {self.target_angle:.1f}° | Err: {self.actual_angle - self.target_angle:.1f}°")

if __name__ == "__main__":
    root = tk.Tk()
    app = ServoDesktopUI(root)
    root.mainloop()

