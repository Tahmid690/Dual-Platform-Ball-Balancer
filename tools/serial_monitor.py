#!/usr/bin/env python3
"""
Serial Monitor for ATmega32 HC-SR04 Ultrasonic Sensor
Reads UART output at 9600 baud and displays it with timestamps.

Usage:
    python3 serial_monitor.py              # Auto-detect serial port
    python3 serial_monitor.py /dev/ttyUSB0 # Specify port manually
"""

import sys
import time
import glob
import serial

BAUD_RATE = 9600
TIMEOUT = 1  # seconds


def find_serial_port():
    """Auto-detect common USB-serial adapter ports on Linux."""
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*"]
    ports = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    if not ports:
        return None
    ports.sort()
    return ports[0]


def main():
    # Determine port
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_serial_port()
        if port is None:
            print("❌ No serial port found. Connect your USB-serial adapter")
            print("   or specify the port manually:")
            print(f"   python3 {sys.argv[0]} /dev/ttyUSB0")
            sys.exit(1)

    print(f"🔌 Connecting to {port} at {BAUD_RATE} baud...")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"❌ Could not open {port}: {e}")
        print("   Try: sudo chmod 666", port)
        sys.exit(1)

    # Flush any stale data
    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"✅ Connected! Listening for data... (Ctrl+C to quit)\n")
    print(f"{'TIME':<12} {'DATA'}")
    print("─" * 50)

    try:
        while True:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                timestamp = time.strftime("%H:%M:%S")

                # Color-code errors in red
                if "Error" in line or "Timeout" in line:
                    print(f"{timestamp:<12} \033[91m{line}\033[0m")
                else:
                    print(f"{timestamp:<12} {line}")

    except KeyboardInterrupt:
        print("\n\n🛑 Stopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
