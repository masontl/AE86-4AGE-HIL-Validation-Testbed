# ==============================================================================
# LESSON 9 (PART 1): ALWAYS-ON VIRTUAL 512MB TF/SD CARD DAEMON (sd_card_daemon.py)
# Listens on COM3 @ 115200 baud and writes all 20Hz telemetry to VIRTUAL_SD_512MB.CSV
# Press Ctrl+C (or Stop in Thonny) at any time to "Eject" the SD card for analysis!
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
    "Coolant_F", "TVIS", "Fan", "Live_DTC", "Latched_DTC"
]


def main():
    print("=" * 72)
    print("  [VIRTUAL 512MB TF/SD CARD CONTROLLER] — MOUNTED & LISTENING")
    print("=" * 72)

    file_exists = os.path.exists(SD_CARD_FILENAME) and os.path.getsize(SD_CARD_FILENAME) > 0
    current_size = os.path.getsize(SD_CARD_FILENAME) if file_exists else 0

    print(f"  - SD Card Image File : {SD_CARD_FILENAME}")
    print(f"  - Initial Card Usage : {current_size:,} / {MAX_SD_CAPACITY_BYTES:,} Bytes "
          f"({(current_size / MAX_SD_CAPACITY_BYTES) * 100:.6f}%)")
    print("  - Action             : Streaming 20Hz Flight Data... (Press STOP / Ctrl+C to Eject Card)")
    print("-" * 72)

    frames_logged = 0
    fault_frames_seen = 0
    t_start = time.perf_counter()

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0) as ser, \
             open(SD_CARD_FILENAME, mode="a", newline="") as sd_file:

            writer = csv.writer(sd_file)
            if not file_exists:
                writer.writerow(CSV_HEADER)
                sd_file.flush()

            # Flush stale OS buffer so we start logging cleanly on live frames
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
                if len(parts) != 9:
                    continue

                try:
                    vals = [int(x) for x in parts]
                except ValueError:
                    continue

                host_ts = f"{time.perf_counter() - t_start:.3f}"
                row = [host_ts] + vals
                writer.writerow(row)
                frames_logged += 1

                if vals[7] != 0:  # Live_DTC != 0
                    fault_frames_seen += 1

                # Flush to disk and print live status every 20 frames (1.0 second @ 20Hz)
                if frames_logged % 20 == 0:
                    sd_file.flush()
                    current_size = os.path.getsize(SD_CARD_FILENAME)
                    status_tag = " [!] ACTIVE FAULT LOGGING!" if vals[7] != 0 else " [OK]"
                    print(f"  [SD WRITE] Frames: {frames_logged:5d} ({frames_logged * 0.05:6.1f}s) | "
                          f"RPM: {vals[1]:4d} | Temp: {vals[4]:3d}F | RawTHW: {vals[3]:4d} | "
                          f"SD Size: {current_size / 1024:.1f} KB{status_tag}")

    except KeyboardInterrupt:
        print("\n" + "=" * 72)
        print("  [SD CARD SAFELY EJECTED] Ready for Offline Post-Processing!")
        print(f"  - Total Frames Written This Session : {frames_logged} ({frames_logged * 0.05:.1f}s)")
        print(f"  - Fault Frames Captured on Card     : {fault_frames_seen}")
        print("=" * 72)
    except Exception as e:
        print(f"\n[SD CONTROLLER ERROR] {e}")


if __name__ == "__main__":
    main()