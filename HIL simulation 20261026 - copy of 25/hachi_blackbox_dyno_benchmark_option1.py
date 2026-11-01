import serial
import time
import csv
import os
import re

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
BASE_NAME = "hachi_fifo_blackbox_benchmark"


def get_next_csv_filename(base_name):
    max_num = -1
    pattern = re.compile(rf"^{re.escape(base_name)}_(\d+)\.csv$", re.IGNORECASE)
    for fname in os.listdir("."):
        match = pattern.match(fname)
        if match:
            num = int(match.group(1))
            if num > max_num:
                max_num = num
    return f"{base_name}_{max_num + 1:02d}.csv"


terminal_log = []


def log_print(message):
    print(message)
    terminal_log.append(message)


csv_filename = get_next_csv_filename(BASE_NAME)
log_print(f"[HACHI FIFO] Initializing 30s FIFO Pre-Trigger Benchmark on {SERIAL_PORT}...")
log_print(f"[HACHI FIFO] Target Artifact: {csv_filename}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(0.2)

    # 1. Clear EEPROM and restore normal state
    ser.write(b'NNN\n')
    ser.write(b'RRR\n')
    ser.write(b'CCC\n')
    time.sleep(0.4)
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)
        time.sleep(0.05)
    ser.readline()

    fault1_injected = False
    fault1_latency_ms = None
    fault2_injected = False
    fault2_latency_ms = None

    t_fault1_inject = 0.0
    t_fault2_inject = 0.0

    dump_history_records = []

    with open(csv_filename, "w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(["SimTime_ms", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC", "Phase"])

        # Run 160 frames (~8 seconds live observation)
        for frame in range(160):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not (line.startswith("$TELEM") and line.endswith("*")):
                continue

            parts = line[7:-1].split(",")
            if len(parts) != 7:
                continue

            vals = [int(x) for x in parts]
            t_ms = vals[0]
            rpm  = vals[1]
            tps  = vals[2]
            thw  = vals[3]
            tvis = vals[4]
            fan  = vals[5]
            dtc  = vals[6]

            phase = "1_BASELINE"

            # Inject Fault #1 at Frame 20
            if frame == 20 and not fault1_injected:
                ser.write(b'FFF\n')
                fault1_injected = True
                t_fault1_inject = time.perf_counter()
                log_print(f"[Frame {frame}] >>> Injected Fault #1 (THW Short to GND)")

            if fault1_injected and not fault2_injected:
                if dtc == 4 and fault1_latency_ms is None:
                    fault1_latency_ms = (time.perf_counter() - t_fault1_inject) * 1000.0
                    log_print(f"[Frame {frame}] >>> Fault #1 Latched! Reaction: {fault1_latency_ms:.1f} ms")

            # Heal and Reset Latch at Frame 50
            if frame == 50:
                ser.write(b'NNN\n')
                ser.write(b'RRR\n')
                log_print(f"[Frame {frame}] >>> Healed Wire & Reset Latch")

            # Inject Split-Lap Fault #2 at Frame 80
            if frame == 80 and not fault2_injected:
                ser.write(b'FFF\n')
                fault2_injected = True
                t_fault2_inject = time.perf_counter()
                log_print(f"[Frame {frame}] >>> Injected Fault #2 (Secondary Event)")

            if fault2_injected:
                if dtc == 4 and fault2_latency_ms is None:
                    fault2_latency_ms = (time.perf_counter() - t_fault2_inject) * 1000.0
                    log_print(f"[Frame {frame}] >>> Fault #2 Latched! Reaction: {fault2_latency_ms:.1f} ms")

            # Heal and Reset at Frame 110
            if frame == 110:
                ser.write(b'NNN\n')
                ser.write(b'RRR\n')
                log_print(f"[Frame {frame}] >>> Final Wire Restoration & Clear")

            writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc, phase])

        # Step 2: Interrogate Multi-Bank 30s History Dump ('D')
        log_print("\n[HIL BENCHMARK] Interrogating Non-Volatile Multi-Bank FIFO Dump...")
        time.sleep(0.5)  # Wait for non-blocking EEPROM write to finish
        ser.write(b'D\n')
        time.sleep(0.2)

        total_banks_reported = 0
        while ser.in_waiting > 0:
            resp = ser.readline().decode('utf-8', errors='ignore').strip()
            if resp.startswith("$DUMP_START"):
                parts = resp[12:-1].split(",")
                total_banks_reported = int(parts[0])
            elif resp.startswith("$HIST"):
                h_parts = resp[6:-1].split(",")
                if len(h_parts) == 8:
                    dump_history_records.append([int(x) for x in h_parts])
            elif resp.startswith("$DUMP_END"):
                break

        # Append History Dump Table to CSV
        writer.writerow([])
        writer.writerow(["--- 30-SECOND PRE-TRIGGER FIFO EEPROM DUMP ---"])
        writer.writerow(["Bank_ID", "RelTime_s", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC"])
        for rec in dump_history_records:
            writer.writerow(rec)

        log_print("========================================================")
        log_print(f"   HACHI 30S FIFO BLACK-BOX SCORECARD ({csv_filename})")
        log_print("========================================================")
        log_print(f" 1. Primary Fault Latch & Capture  : {'PASSED [OK]' if fault1_latency_ms else 'FAILED'}")
        log_print(f" 2. Split-Lap Secondary Latch      : {'PASSED [OK]' if fault2_latency_ms else 'FAILED'}")
        log_print(f" 3. Total Recorded EEPROM Banks    : {total_banks_reported} (Expected: 2)")
        log_print(f" 4. Total Pre-Trigger Frames Saved : {len(dump_history_records)} (30 per Bank)")
        log_print("========================================================")

        writer.writerow([])
        writer.writerow(["--- TERMINAL SCORECARD ---"])
        for line in terminal_log:
            writer.writerow([line])

    ser.close()
    print(f"\n[COMPLETE] Report saved to: {csv_filename}")

except Exception as e:
    print(f"[ERROR] Serial issue on {SERIAL_PORT}: {e}")