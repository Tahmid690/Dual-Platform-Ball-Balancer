#!/usr/bin/env python3
"""
Sensor Data Collection & Crossfade Analysis Script

Usage:
    python3 collect_data.py               # Auto-detect serial port & record
    python3 collect_data.py /dev/ttyUSB0  # Specify port manually
"""

import sys
import time
import glob
import csv
import serial

BAUD_RATE = 9600
TIMEOUT = 1
OUTPUT_CSV = "sensor_analysis.csv"


def find_serial_port():
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*"]
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
    if not ports:
        return None
    ports.sort()
    return ports[0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_serial_port()
    if not port:
        print("❌ No serial port found. Connect your USB-serial adapter.")
        sys.exit(1)

    print(f"🔌 Connecting to {port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"❌ Could not open {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"✅ Recording sensor sweep data to {OUTPUT_CSV}... (Press Ctrl+C to finish & analyze)\n")

    records = []
    headers = ["Angle_deg", "US1_mm", "VL1_raw", "VL1_pos", "Diff1", "US2_mm", "VL2_raw", "VL2_pos", "Diff2"]

    with open(OUTPUT_CSV, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)

        try:
            while True:
                line_raw = ser.readline()
                if not line_raw:
                    continue
                line = line_raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                timestamp = time.strftime("%H:%M:%S")

                if line.startswith("CSV,"):
                    parts = line.split(",")[1:]
                    if len(parts) == len(headers):
                        try:
                            vals = [int(p) for p in parts]
                            ang, us1, vl1_raw, vl1_pos, diff1, us2, vl2_raw, vl2_pos, diff2 = vals

                            # Filter out timeout/invalid US readings (> 1000mm)
                            if us1 > 1000 or us1 <= 0:
                                us1 = 0
                                diff1 = 0
                            elif vl1_pos > 0:
                                diff1 = us1 - vl1_pos

                            if us2 > 1000 or us2 <= 0:
                                us2 = 0
                                diff2 = 0
                            elif vl2_pos > 0:
                                diff2 = us2 - vl2_pos

                            vals = [ang, us1, vl1_raw, vl1_pos, diff1, us2, vl2_raw, vl2_pos, diff2]
                            writer.writerow(vals)
                            csvfile.flush()
                            records.append(vals)

                            print(f"{timestamp} | Ang:{ang:3d}° | US1:{us1:3d} VL1_pos:{vl1_pos:3d} (Diff:{diff1:+3d}mm) | US2:{us2:3d} VL2_pos:{vl2_pos:3d} (Diff:{diff2:+3d}mm)")
                        except ValueError:
                            pass
                else:
                    print(f"{timestamp} | {line}")

        except KeyboardInterrupt:
            print("\n\n🛑 Collection stopped by user.")
        finally:
            ser.close()

    # Perform automated analysis
    if records:
        print("\n" + "=" * 60)
        print("📊 SENSOR DISCREPANCY & JUMP ANALYSIS SUMMARY")
        print("=" * 60)

        diff1_list = [r[4] for r in records if r[1] > 0 and r[3] > 0]
        diff2_list = [r[8] for r in records if r[5] > 0 and r[7] > 0]

        if diff1_list:
            avg_diff1 = sum(diff1_list) / len(diff1_list)
            min_diff1 = min(diff1_list)
            max_diff1 = max(diff1_list)
            print(f"\n[Ball Sensors: US1 vs VL1_pos]")
            print(f"  Total Valid Points : {len(diff1_list)}")
            print(f"  Average Discrepancy: {avg_diff1:+.2f} mm")
            print(f"  Min Jump           : {min_diff1:+d} mm")
            print(f"  Max Jump           : {max_diff1:+d} mm")

        if diff2_list:
            avg_diff2 = sum(diff2_list) / len(diff2_list)
            min_diff2 = min(diff2_list)
            max_diff2 = max(diff2_list)
            print(f"\n[Setpoint Sensors: US2 vs VL2_pos]")
            print(f"  Total Valid Points : {len(diff2_list)}")
            print(f"  Average Discrepancy: {avg_diff2:+.2f} mm")
            print(f"  Min Jump           : {min_diff2:+d} mm")
            print(f"  Max Jump           : {max_diff2:+d} mm")

        print(f"\n✅ Full raw dataset saved to: {OUTPUT_CSV}")
        print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
