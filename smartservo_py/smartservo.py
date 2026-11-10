"""
SmartServo Python Module
Client driver for ElectronBot SmartServo via Seeed Studio XIAO ESP32-S3 USB Bridge.
"""

import time
import threading
import serial
import serial.tools.list_ports
from typing import List, Optional, Tuple

class SmartServoClient:
    """Manages the serial connection to the Seeed Studio XIAO ESP32-S3 USB Bridge."""
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        if port:
            self.connect(port)

    @staticmethod
    def list_ports() -> List[str]:
        """Returns list of available serial port names, prioritizing USB/ACM devices."""
        all_ports = [p.device for p in serial.tools.list_ports.comports()]
        usb_ports = [p for p in all_ports if "ACM" in p or "USB" in p]
        other_ports = [p for p in all_ports if p not in usb_ports]
        return usb_ports + other_ports

    def connect(self, port: str):
        """Connects to the specified serial port in a thread-safe manner."""
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.port = port
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = self.baudrate
            self.ser.timeout = self.timeout
            self.ser.dtr = None
            self.ser.rts = None
            self.ser.open()
            time.sleep(0.15)
            self.ser.reset_input_buffer()

    def disconnect(self):
        """Disconnects the serial connection."""
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send_raw(self, cmd_str: str) -> str:
        """Sends a text command and reads the response line with thread lock protection."""
        with self._lock:
            if not self.is_connected():
                return ""
            try:
                if self.ser.in_waiting > 0:
                    self.ser.reset_input_buffer()
                self.ser.write((cmd_str.strip() + "\n").encode('utf-8'))
                deadline = time.time() + self.timeout
                while time.time() < deadline:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    if line.startswith("[") and "]" in line:
                        continue  # Skip internal ESP-IDF log messages
                    # Minimum 5ms inter-command spacing to let I2C bus and driver settle
                    time.sleep(0.005)
                    return line
                return ""
            except Exception:
                return ""

    def ping(self) -> bool:
        """Pings the ESP32-S3 bridge."""
        try:
            return self.send_raw("PING") == "PONG"
        except Exception:
            return False

    def scan_bus(self) -> List[int]:
        """Scans the I2C bus and returns a list of detected device 7-bit addresses."""
        resp = self.send_raw("SCAN")
        if resp.startswith("DEVICES"):
            parts = resp.replace("DEVICES", "").strip()
            if not parts:
                return []
            return [int(p) for p in parts.split(",") if p.isdigit()]
        return []

    def get_servo(self, node_id: int = 6) -> 'SmartServo':
        """Returns a SmartServo controller object for a given node ID."""
        return SmartServo(self, node_id)


class SmartServo:
    """Interface to a specific SmartServo node on the I2C bus."""
    def __init__(self, client: SmartServoClient, node_id: int = 6):
        self.client = client
        self.node_id = node_id

    def set_angle(self, angle: float) -> Optional[float]:
        """
        Sets target mechanical angle in degrees (Opcode 0x01).
        Returns actual measured angle, or None on I2C failure.
        """
        resp = self.client.send_raw(f"ANGLE {self.node_id} {angle:.2f}")
        if resp.startswith("OK"):
            return float(resp.split()[1])
        return None

    def set_velocity(self, vel: float) -> Optional[float]:
        """
        Sets velocity limit (Opcode 0x02).
        Returns actual measured velocity, or None on failure.
        """
        resp = self.client.send_raw(f"VEL {self.node_id} {vel:.2f}")
        if resp.startswith("OK"):
            return float(resp.split()[1])
        return None

    def set_torque(self, torque: float) -> Optional[float]:
        """
        Sets dynamic torque limit 0.0 - 1.0 (Opcode 0x03).
        Returns actual measured angle, or None on failure.
        """
        resp = self.client.send_raw(f"TORQUE {self.node_id} {torque:.2f}")
        if resp.startswith("OK"):
            return float(resp.split()[1])
        return None

    def get_angle(self) -> Optional[float]:
        """Reads current joint angle in degrees (Opcode 0x11)."""
        resp = self.client.send_raw(f"GET_ANGLE {self.node_id}")
        if resp.startswith("OK"):
            return float(resp.split()[1])
        return None

    def get_velocity(self) -> Optional[float]:
        """Reads estimated velocity in deg/s (Opcode 0x12)."""
        resp = self.client.send_raw(f"GET_VEL {self.node_id}")
        if resp.startswith("OK"):
            return float(resp.split()[1])
        return None

    def set_enable(self, enable: bool) -> Optional[float]:
        """
        Enables or disables motor H-bridge PWM output (Opcode 0xFF).
        Returns actual measured angle on success, or None on failure.
        """
        arg = 1 if enable else 0
        resp = self.client.send_raw(f"ENABLE {self.node_id} {arg}")
        if resp.startswith("OK"):
            parts = resp.split()
            if len(parts) > 1:
                return float(parts[1])
            return 0.0
        return None

    def set_pid(self, kp: float, ki: float, kv: float, kd: float) -> bool:
        """
        Configures DCE / PID loop gains and commits them to non-volatile flash (Opcodes 0x22-0x25).
        """
        ok = True
        ok &= self.client.send_raw(f"SET_KP {self.node_id} {kp:.2f}").startswith("OK")
        time.sleep(0.04)
        ok &= self.client.send_raw(f"SET_KI {self.node_id} {ki:.2f}").startswith("OK")
        time.sleep(0.04)
        ok &= self.client.send_raw(f"SET_KV {self.node_id} {kv:.2f}").startswith("OK")
        time.sleep(0.04)
        ok &= self.client.send_raw(f"SET_KD {self.node_id} {kd:.2f}").startswith("OK")
        time.sleep(0.04)
        return ok

    def get_pid(self) -> Optional[Tuple[float, float, float, float]]:
        """
        Reads current PID loop gains (Kp, Ki, Kv, Kd) from servo Flash/RAM.
        Returns (kp, ki, kv, kd) or None on failure.
        """
        r_kp = self.client.send_raw(f"GET_KP {self.node_id}")
        r_ki = self.client.send_raw(f"GET_KI {self.node_id}")
        r_kv = self.client.send_raw(f"GET_KV {self.node_id}")
        r_kd = self.client.send_raw(f"GET_KD {self.node_id}")
        if r_kp.startswith("OK") and r_ki.startswith("OK") and r_kv.startswith("OK") and r_kd.startswith("OK"):
            try:
                kp = float(r_kp.split()[1])
                ki = float(r_ki.split()[1])
                kv = float(r_kv.split()[1])
                kd = float(r_kd.split()[1])
                return (kp, ki, kv, kd)
            except Exception:
                return None
        return None

    def set_torque_limit(self, limit: float) -> bool:
        """Sets max torque cap and commits to flash (Opcode 0x26)."""
        resp = self.client.send_raw(f"SET_TORQUE_LIMIT {self.node_id} {limit:.2f}")
        return resp.startswith("OK")

    def set_init_pos(self, pos: float) -> bool:
        """Sets default power-on startup angle and commits to flash (Opcode 0x27)."""
        resp = self.client.send_raw(f"SET_INIT_POS {self.node_id} {pos:.2f}")
        return resp.startswith("OK")

    def change_node_id(self, new_id: int) -> bool:
        """
        Reassigns the servo's 7-bit I2C address and commits to flash (Opcode 0x21).
        Updates this object's node_id on success.
        """
        resp = self.client.send_raw(f"SET_ID {self.node_id} {new_id}")
        if resp.startswith("OK"):
            self.node_id = new_id
            return True
        return False

    def set_invert(self, invert: bool) -> bool:
        """
        Sets motor polarity direction (Opcode 0x30).
        0 = Normal, 1 = Inverted (Reverse). Commits to Flash.
        """
        arg = 1 if invert else 0
        resp = self.client.send_raw(f"SET_INVERT {self.node_id} {arg}")
        return resp.startswith("OK")

    def get_invert(self) -> Optional[bool]:
        """
        Reads motor polarity direction (Opcode 0x14).
        Returns True if inverted, False if normal, or None on failure.
        """
        resp = self.client.send_raw(f"GET_INVERT {self.node_id}")
        if resp.startswith("OK"):
            parts = resp.split()
            if len(parts) > 1:
                return (int(parts[1]) != 0)
        return None

    def set_max_angle(self, max_angle: float) -> bool:
        """
        Sets mechanical maximum angle range (e.g. 180.0 or 360.0 degrees)
        and commits to Flash (Opcode 0x2A).
        """
        resp = self.client.send_raw(f"SET_MAX_ANGLE {self.node_id} {max_angle:.1f}")
        return resp.startswith("OK")

    def get_max_angle(self) -> Optional[float]:
        """
        Reads mechanical maximum angle range (Opcode 0x15).
        Returns float (e.g. 180.0 or 360.0), or None on failure.
        """
        resp = self.client.send_raw(f"GET_MAX_ANGLE {self.node_id}")
        if resp.startswith("OK"):
            parts = resp.split()
            if len(parts) > 1:
                return float(parts[1])
        return None

    def get_raw_adc(self) -> Optional[float]:
        """Reads raw 12-bit ADC value (0-4095) from potentiometer (Opcode 0x13)."""
        resp = self.client.send_raw(f"GET_RAW_ADC {self.node_id}")
        if resp.startswith("OK"):
            parts = resp.split()
            if len(parts) > 1:
                return float(parts[1])
        return None

    def set_adc_min(self, min_val: float) -> bool:
        """Sets potentiometer ADC calibration value at 0 degrees (Opcode 0x28)."""
        resp = self.client.send_raw(f"SET_ADC_MIN {self.node_id} {min_val:.0f}")
        return resp.startswith("OK")

    def set_adc_max(self, max_val: float) -> bool:
        """Sets potentiometer ADC calibration value at max degrees (Opcode 0x29)."""
        resp = self.client.send_raw(f"SET_ADC_MAX {self.node_id} {max_val:.0f}")
        return resp.startswith("OK")



