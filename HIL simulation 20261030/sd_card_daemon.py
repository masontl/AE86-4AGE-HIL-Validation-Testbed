# ==============================================================================
# LESSON 10 (PART 1): ALWAYS-ON VIRTUAL 512MB TF/SD CARD DAEMON (sd_card_daemon.py)
# Listens on COM3 @ 115200 baud and writes 12-channel 20Hz telemetry (including
# OX_mV, VF_mV, and Fuel_Trim_Pct) to VIRTUAL_SD_512MB.CSV.
# ==============================================================================

import serial
import time
import csv
import os

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
SD_CARD_FILENAME = "VIRTUAL_SD_512MB.CSV"
MAX_SD_CAPACITY_BYTES = 512 * 1024 * 1024  # 512 MB Virtual SD Card Limit

CSV_HEADER = [
    "Host_Timestamp", "SimTime_ms", "RPM", "TPS_Pct", "Raw_THW",
    "Coolant_F", "TVIS", "Fan", "Live_DTC", "Latched_DTC",
    "OX_mV", "VF_mV", "Fuel_Trim_Pct"
]


def main():
    print("=" * 80)
    print("  [VIRTUAL 512MB TF/SD CARD CONTROLLER — LESSON 10] — MOUNTED & LISTENING")
    print("=" * 80)

    file_exists = os.path.exists(SD_CARD_FILENAME) and os.path.getsize(SD_CARD_FILENAME) > 0
    current_size = os.path.getsize(SD_CARD_FILENAME) if file_exists else 0

    print(f"  - SD Card Image File : {SD_CARD_FILENAME}")
    print(f"  - Initial Card Usage : {current_size:,} / {MAX_SD_CAPACITY_BYTES:,} Bytes "
          f"({(current_size / MAX_SD_CAPACITY_BYTES) * 100:.6f}%)")
    print("  - Action             : Streaming 20Hz Flight Data (12 Channels)... (Ctrl+C to Eject)")
    print("-" * 80)

    frames_logged = 0
    hard_fault_frames = 0
    soft_trim_frames = 0
    t_start = time.perf_counter()

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0) as ser, \
             open(SD_CARD_FILENAME, mode="a", newline="") as sd_file:

            writer = csv.writer(sd_file)
            if not file_exists:
                writer.writerow(CSV_HEADER)
                sd_file.flush()

            if ser.in_waiting > 0:
                ser.read(ser.in_waiting)

            while True:
                if current_size >= MAX_SD_CAPACITY_BYTES:
                    print("\n[!] SD CARD FULL (512 MB Limit Reached)! Halting writes.")
                    break

                raw = ser.readline().decode('utf-8', errors='ignore').strip()
                if not (raw.startswith("$TELEM,") and raw.endswith("*")):
                    continue

                parts = raw[7:-1].split(",")
                if len(parts) != 12:
                    continue

                try:
                    vals = [int(x) for x in parts]
                except ValueError:
                    continue

                host_ts = f"{time.perf_counter() - t_start:.3f}"
                row = [host_ts] + vals
                writer.writerow(row)
                frames_logged += 1

                live_dtc = vals[7]
                ox_mv = vals[9]
                vf_mv = vals[10]
                trim_pct = vals[11]

                if live_dtc != 0:
                    hard_fault_frames += 1
                if trim_pct >= 15 and ox_mv < 250:
                    soft_trim_frames += 1

                # Flush to disk and print live status every 20 frames (1.0 second @ 20Hz)
                if frames_logged % 20 == 0:
                    sd_file.flush()
                    current_size = os.path.getsize(SD_CARD_FILENAME)
                    if live_dtc != 0:
                        status_tag = " [!] HARD OBD-I DTC!"
                    elif trim_pct >= 15 and ox_mv < 250:
                        status_tag = " [~] SOFT FAULT: LEAN OX + MAX RICH TRIM (NO CEL)"
                    else:
                        status_tag = " [OK]"

                    print(f"  [SD] #{frames_logged:4d} ({frames_logged * 0.05:5.1f}s) | "
                          f"RPM:{vals[1]:4d} | Temp:{vals[4]:3d}F | "
                          f"OX:{ox_mv:4d}mV | VF:{vf_mv:4d}mV ({trim_pct:+3d}%) | "
                          f"{current_size / 1024:.1f}KB{status_tag}")

    except KeyboardInterrupt:
        print("\n" + "=" * 80)
        print("  [SD CARD SAFELY EJECTED] Ready for Offline Post-Processing!")
        print(f"  - Total Frames Written This Session      : {frames_logged} ({frames_logged * 0.05:.1f}s)")
        print(f"  - Hard OBD-I Fault Frames Captured       : {hard_fault_frames}")
        print(f"  - Soft Fuel-Trim Anomaly Frames Captured : {soft_trim_frames}")
        print("=" * 80)
    except Exception as e:
        print(f"\n[SD CONTROLLER ERROR] {e}")


if __name__ == "__main__":
    main()