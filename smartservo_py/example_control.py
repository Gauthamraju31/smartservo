#!/usr/bin/env python3
"""
Simple programmatic example using the smartservo Python module.
Connects to the Seeed Studio XIAO ESP32-S3 over USB and controls the servo.
"""

import time
from smartservo import SmartServoClient

def main():
    ports = SmartServoClient.list_ports()
    print(f"Available serial ports: {ports}")
    
    if not ports:
        print("No serial ports detected. Connect the Seeed Studio XIAO ESP32-S3 via USB.")
        return

    # Use the first USB port, or specify your port (e.g. '/dev/ttyACM0')
    port = ports[0]
    print(f"Connecting to {port}...")
    
    client = SmartServoClient(port=port)
    if not client.ping():
        print("[WARN] ESP32-S3 did not respond to PING. Check firmware.")
    
    # 1. Scan for active servo devices on the I2C bus
    print("Scanning I2C bus for servos...")
    devices = client.scan_bus()
    print(f"Found devices at addresses: {[hex(d) for d in devices]}")

    target_id = 12 if 12 in devices else (devices[0] if devices else 12)
    print(f"Using Servo Node ID: {target_id} (0x{target_id:02X})")
    
    servo = client.get_servo(node_id=target_id)

    # 2. Enable Motor
    print("Enabling motor output...")
    servo.set_enable(True)

    # 3. Read Current Position
    cur_angle = servo.get_angle()
    print(f"Current Joint Angle: {cur_angle:.2f} deg")

    # 4. Move through a sequence of angles
    test_positions = [45.0, 90.0, 135.0, 90.0]
    for target in test_positions:
        print(f"Moving to {target}°...")
        measured = servo.set_angle(target)
        time.sleep(0.8)
        actual = servo.get_angle()
        print(f"  Reached: {actual:.2f}° (Error: {actual - target:+.2f}°)")

    # 5. Disable Motor
    print("Disabling motor...")
    servo.set_enable(False)
    client.disconnect()
    print("Done!")

if __name__ == "__main__":
    main()

