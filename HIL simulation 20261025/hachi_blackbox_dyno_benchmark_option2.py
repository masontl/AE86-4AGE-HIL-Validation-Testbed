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
log_print(f"[HACHI FIFO RECORDER] Starting HIL Pre-Trigger Benchmark on {SERIAL_PORT}...")
log_print(f"[HACHI FIFO RECORDER] Output Report: {csv_filename}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(0.2)

    # 1. Startup Synchronization & Buffer Flush
    ser.write(b'NNN\n')
    ser.write(b'RRR\n')
    time.sleep(0.3)
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)
        time.sleep(0.05)
    ser.readline()

    fault_injected = False
    fault_inject_wall_time = 0.0
    fault_inject_sim_ms = 0
    wall_latency_ms = None
    sim_latency_ms = None
    recovery_verified = False
    fault_cleared_sent = False
    freeze_frame_data = None
    history_records = []

    with open(csv_filename, "w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(["SimTime_ms", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC", "Test_Phase"])

        # Capture 140 frames to allow pre-trigger buffer to populate
        for frame in range(140):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not (line.startswith("$TELEM") and line.endswith("*")):
                continue

            parts = line[7:-1].split(",")
            if len(parts) != 7:
                continue

            vals = [int(x) for x in parts]
            t_ms, rpm, tps, thw, tvis, fan, dtc = vals
            phase = "1_BASELINE_RECORDING"

            # Inject fault at Frame 50
            if frame == 50 and not fault_injected:
                ser.write(b'FFF\n')
                fault_injected = True
                fault_inject_wall_time = time.perf_counter()
                fault_inject_sim_ms = t_ms
                log_print(f"[Frame {frame}] >>> Sent 'FFF': THW Sensor Ground Fault Injected (SimTime={t_ms}ms)!")

            if fault_injected and not fault_cleared_sent and frame < 85 and thw > 400:
                ser.write(b'F\n')

            if fault_injected and wall_latency_ms is None:
                phase = "2_FAULT_DECAYING"
                if dtc == 4 and fan == 1:
                    wall_latency_ms = (time.perf_counter() - fault_inject_wall_time) * 1000.0
                    sim_latency_ms = t_ms - fault_inject_sim_ms
                    log_print(f"[Frame {frame}] >>> ECU Latched Fault & Locked FIFO Buffer! (THW={thw}, Fan=ON, DTC=4)")
                    log_print(f"            >>> Reaction Latency: {wall_latency_ms:.1f} ms (Sim: {sim_latency_ms} ms)")
            elif fault_injected and wall_latency_ms is not None and not fault_cleared_sent:
                phase = "2_FAULT_LATCHED"

            # Restore sensor wire at Frame 95
            if frame == 95:
                ser.write(b'NNN\n')
                log_print(f"[Frame {frame}] >>> Sent 'NNN': Restored THW Signal...")

            # Reset live fault latch at Frame 115
            if frame == 115 and not fault_cleared_sent:
                ser.write(b'RRR\n')
                fault_cleared_sent = True
                log_print(f"[Frame {frame}] >>> Sent 'RRR': Resetting Live Fault Latch...")

            if fault_cleared_sent:
                phase = "3_POST_RECOVERY"
                if dtc == 0 and thw > 200:
                    recovery_verified = True

            writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc, phase])

        # Step 6: Post-Test Interrogation ('D')
        log_print("\n[HIL BENCHMARK] Interrogating Non-Volatile EEPROM Pre-Trigger Black Box...")
        time.sleep(0.5)  # Allow background non-blocking EEPROM writes to finalize
        ser.write(b'D\n')
        time.sleep(0.1)

        # Read $FREEZE summary and all 30 $HIST entries
        for _ in range(45):
            resp = ser.readline().decode('utf-8', errors='ignore').strip()
            if resp.startswith("$FREEZE") and resp.endswith("*"):
                f_parts = resp[8:-1].split(",")
                if len(f_parts) == 4:
                    freeze_frame_data = [int(x) for x in f_parts]
            elif resp.startswith("$HIST") and resp.endswith("*"):
                h_parts = resp[6:-1].split(",")
                if len(h_parts) == 7:
                    history_records.append([int(x) for x in h_parts])

        log_print("======================================================================")
        log_print(f"       HACHI 30-SECOND PRE-TRIGGER FLIGHT RECORDER SCORECARD")
        log_print("======================================================================")
        log_print(f" 1. Fault Trigger & Failsafe Active  : {'PASSED [OK]' if wall_latency_ms else 'FAILED'}")
        if wall_latency_ms:
            log_print(f"    - Hardware Reaction Latency      : {wall_latency_ms:.1f} ms")
        log_print(f" 2. Self-Healing & Latch Reset       : {'PASSED [OK]' if recovery_verified else 'FAILED'}")

        if freeze_frame_data and freeze_frame_data[0] != 0:
            dtc_f, rpm_f, tps_f, peak_f = freeze_frame_data
            log_print(" 3. Freeze-Frame Summary Snapshot    : CAPTURED [OK]")
            log_print(f"    - Latched Diagnostic Trouble Code: Code {dtc_f}")
            log_print(f"    - Engine Speed at Fault Trigger  : {rpm_f} RPM")
            log_print(f"    - Throttle Position at Fault     : {tps_f}%")
            log_print(f"    - All-Time Peak Engine Speed     : {peak_f} RPM")
        else:
            log_print(" 3. Freeze-Frame Summary Snapshot    : FAILED")

        log_print(f" 4. 30-Second Pre-Fault FIFO History : {len(history_records)}/30 SAMPLES RETRIEVED")
        if history_records:
            log_print("    [Offset] | Engine RPM | Throttle | Raw THW | TVIS | Fan | DTC")
            log_print("    ----------------------------------------------------------")
            for rec in history_records[-10:]:  # Print final 10 seconds leading to trigger
                offset, r_rpm, r_tps, r_thw, r_tvis, r_fan, r_dtc = rec
                log_print(f"    {offset:+3d}s    |  {r_rpm:4d} RPM |   {r_tps:3d}%   |   {r_thw:4d}  |   {r_tvis}  |  {r_fan}  |  {r_dtc}")

        log_print("======================================================================")

        writer.writerow([])
        writer.writerow(["--- 30-SECOND EEPROM PRE-TRIGGER HISTORY TABLE ---"])
        writer.writerow(["Offset_Sec", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC"])
        for rec in history_records:
            writer.writerow(rec)

        writer.writerow([])
        writer.writerow(["--- TERMINAL EXECUTION LOG & SCORECARD ---"])
        for log_line in terminal_log:
            writer.writerow([log_line])

    ser.close()
    print(f"\n[COMPLETE] Telemetry report saved to: {csv_filename}")

except Exception as e:
    print(f"[ERROR] Serial connection issue on {SERIAL_PORT}: {e}")