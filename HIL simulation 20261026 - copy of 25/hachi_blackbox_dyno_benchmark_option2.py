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


def drain_to_live_edge(ser):
    """Actively drains all backed-up com0com OS buffers until we hit live 50ms real-time frames."""
    while True:
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        t0 = time.perf_counter()
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        dt_ms = (time.perf_counter() - t0) * 1000.0
        # If readline() had to wait >10ms for MCU#2 to transmit, we are at the real-time live edge!
        if dt_ms > 10.0 and line.startswith("$TELEM") and line.endswith("*"):
            return line


csv_filename = get_next_csv_filename(BASE_NAME)
log_print(f"[HACHI FIFO RECORDER] Starting HIL Pre-Trigger Benchmark on {SERIAL_PORT}...")
log_print(f"[HACHI FIFO RECORDER] Output Report: {csv_filename}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(0.1)

    # 1. Heal wire, reset latch, and drain ALL stale com0com buffered lines!
    ser.write(b'NNN\n')
    ser.write(b'RRR\n')
    log_print("[SYNC] Draining stale com0com buffer to lock onto live real-time telemetry...")
    first_live_line = drain_to_live_edge(ser)
    log_print(f"[SYNC] Locked onto live stream: {first_live_line}")

    fault_injected = False
    fault_inject_wall_time = time.perf_counter()
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

        for frame in range(120):
            # If com0com ever falls behind, discard intermediate backlog so every frame is live
            if ser.in_waiting > 120:
                ser.read(ser.in_waiting)
                ser.readline()

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

            phase = "1_BASELINE_RECORDING"

            # Step 2: Inject Fault at Frame 30
            if frame == 30 and not fault_injected:
                ser.write(b'FFF\n')
                fault_injected = True
                fault_inject_wall_time = time.perf_counter()
                fault_inject_sim_ms = t_ms
                log_print(f"[Frame {frame}] >>> Sent 'FFF': Injected THW Sensor Ground Fault (SimTime={t_ms}ms)!")

            # Keep re-asserting 'F' during the active fault window (Frames 30..75)
            if fault_injected and not fault_cleared_sent and frame < 75 and thw > 100:
                ser.write(b'FFF\n')

            # Catch EITHER Python 'FFF' fault OR manual SPST switch flip!
            if dtc == 4 and fan == 1 and wall_latency_ms is None:
                if not fault_injected:
                    fault_injected = True
                    fault_inject_sim_ms = t_ms
                wall_latency_ms = (time.perf_counter() - fault_inject_wall_time) * 1000.0
                sim_latency_ms = t_ms - fault_inject_sim_ms
                log_print(f"[Frame {frame}] >>> ECU Latched Fault & Locked 30-Slot FIFO! (THW={thw}, Fan=ON, DTC=4)")
                log_print(f"            >>> Reaction Latency: {wall_latency_ms:.1f} ms (Sim: {sim_latency_ms} ms)")

            if fault_injected and wall_latency_ms is None:
                phase = "2_FAULT_DECAYING"
            elif wall_latency_ms is not None and not fault_cleared_sent:
                phase = "2_FAULT_LATCHED"

            # Step 4: Heal Wire at Frame 80
            if frame == 80:
                ser.write(b'NNN\n')
                log_print(f"[Frame {frame}] >>> Sent 'NNN': Restored THW Sensor Wire (SimTime={t_ms}ms)...")

            # Step 5: Reset Live Latch at Frame 95
            if frame == 95 and not fault_cleared_sent:
                ser.write(b'RRR\n')
                fault_cleared_sent = True
                log_print(f"[Frame {frame}] >>> Sent 'RRR': Resetting Live Fault Latch (SimTime={t_ms}ms)...")

            if fault_cleared_sent:
                phase = "3_POST_RECOVERY"
                if dtc == 0 and thw > 200:
                    recovery_verified = True

            writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc, phase])

        # Step 6: Post-Test Interrogation — Request Non-Volatile EEPROM Dump ('D')
        log_print("\n[HIL BENCHMARK] Interrogating Non-Volatile EEPROM 30-Slot FIFO Black Box...")
        drain_to_live_edge(ser)
        ser.write(b'DDD\n')

        # Read lines until we collect $FREEZE and all 30 $HIST records
        t_dump_start = time.perf_counter()
        while (time.perf_counter() - t_dump_start) < 3.0:
            resp = ser.readline().decode('utf-8', errors='ignore').strip()
            if resp.startswith("$FREEZE") and resp.endswith("*"):
                f_parts = resp[8:-1].split(",")
                if len(f_parts) == 4:
                    freeze_frame_data = [int(x) for x in f_parts]
            elif resp.startswith("$HIST") and resp.endswith("*"):
                h_parts = resp[6:-1].split(",")
                if len(h_parts) == 7:
                    rec = [int(x) for x in h_parts]
                    if len(history_records) < 30:
                        history_records.append(rec)
            if freeze_frame_data and len(history_records) >= 30:
                break

        log_print("======================================================================")
        log_print(f"       HACHI 30-SLOT FIFO FLIGHT RECORDER SCORECARD ({csv_filename})")
        log_print("======================================================================")
        log_print(f" 1. Fault Trigger & Failsafe Active  : {'PASSED [OK]' if wall_latency_ms is not None else 'FAILED'}")
        if wall_latency_ms is not None:
            log_print(f"    - Hardware Reaction Latency      : {wall_latency_ms:.1f} ms (Sim: {sim_latency_ms} ms)")
        log_print(f" 2. Self-Healing & Latch Reset       : {'PASSED [OK]' if recovery_verified else 'FAILED'}")

        if freeze_frame_data and freeze_frame_data[0] != 0:
            dtc_f = freeze_frame_data[0]
            rpm_f = freeze_frame_data[1]
            tps_f = freeze_frame_data[2]
            peak_f = freeze_frame_data[3]
            log_print(" 3. Freeze-Frame Summary Snapshot    : CAPTURED [OK]")
            log_print(f"    - Latched Diagnostic Trouble Code: Code {dtc_f}")
            log_print(f"    - Engine Speed at Fault Trigger  : {rpm_f} RPM")
            log_print(f"    - Throttle Position at Fault     : {tps_f}%")
            log_print(f"    - All-Time Peak Engine Speed     : {peak_f} RPM")
        else:
            log_print(" 3. Freeze-Frame Summary Snapshot    : FAILED (No snapshot found)")

        log_print(f" 4. 30-Slot Pre-Fault FIFO History   : {len(history_records)}/30 SAMPLES RETRIEVED")
        if history_records:
            log_print("    [Sample] | Rel Time | Engine RPM | Throttle | Raw THW | TVIS | Fan | DTC")
            log_print("    ------------------------------------------------------------------------")
            for rec in history_records[-12:]:
                idx     = rec[0]
                r_rpm   = rec[1]
                r_tps   = rec[2]
                r_thw   = rec[3]
                r_tvis  = rec[4]
                r_fan   = rec[5]
                r_dtc   = rec[6]
                rel_sec = idx * 0.2
                log_print(f"     {idx:+3d}     |  {rel_sec:+5.1f}s  |  {r_rpm:4d} RPM |   {r_tps:3d}%   |   {r_thw:4d}  |   {r_tvis}  |  {r_fan}  |  {r_dtc}")

        log_print("======================================================================")

        writer.writerow([])
        writer.writerow(["--- 30-SLOT EEPROM PRE-TRIGGER FIFO HISTORY (200ms/step = 6.0s Window) ---"])
        writer.writerow(["Sample_Idx", "Rel_Time_Sec", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC"])
        for rec in history_records:
            writer.writerow([rec[0], f"{rec[0]*0.2:.1f}", rec[1], rec[2], rec[3], rec[4], rec[5], rec[6]])

        writer.writerow([])
        writer.writerow(["--- PYTHON TERMINAL EXECUTION LOG & SCORECARD ---"])
        for log_line in terminal_log:
            writer.writerow([log_line])

    ser.close()
    print(f"\n[COMPLETE] Telemetry + 30-Slot FIFO Report saved to: {csv_filename}")

except Exception as e:
    print(f"[ERROR] Serial connection issue on {SERIAL_PORT}: {e}")