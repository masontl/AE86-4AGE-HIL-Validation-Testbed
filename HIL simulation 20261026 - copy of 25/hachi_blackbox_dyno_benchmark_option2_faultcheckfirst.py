import serial
import time
import csv
import os
import re

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
BASE_NAME = "hachi_diagnostic_report"


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
    """Drains backed-up com0com OS buffers until we hit live 50ms real-time frames."""
    while True:
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        t0 = time.perf_counter()
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if dt_ms > 10.0 and line.startswith("$TELEM") and line.endswith("*"):
            return line


def interrogate_eeprom(ser):
    """Queries MCU#2 EEPROM ('DDD') and returns (freeze_frame_data, history_records)."""
    drain_to_live_edge(ser)
    ser.write(b'DDD\n')
    freeze_data = None
    hist_recs = []
    t_start = time.perf_counter()
    while (time.perf_counter() - t_start) < 3.0:
        resp = ser.readline().decode('utf-8', errors='ignore').strip()
        if resp.startswith("$FREEZE") and resp.endswith("*"):
            f_parts = resp[8:-1].split(",")
            if len(f_parts) == 4:
                freeze_data = [int(x) for x in f_parts]
        elif resp.startswith("$HIST") and resp.endswith("*"):
            h_parts = resp[6:-1].split(",")
            if len(h_parts) == 7:
                rec = [int(x) for x in h_parts]
                if len(hist_recs) < 30:
                    hist_recs.append(rec)
        if freeze_data is not None and (freeze_data[0] == 0 or len(hist_recs) >= 30):
            break
    return freeze_data, hist_recs


def diagnose_fault_root_cause(history_records):
    """Analyzes the 30-slot FIFO curve to determine physical failure mode."""
    if not history_records:
        return "UNKNOWN (No FIFO history available)"
    trigger_thw = history_records[-1][3]
    prev_thw = history_records[-2][3] if len(history_records) >= 2 else trigger_thw

    if trigger_thw < 41:
        if prev_thw > 200:
            return f"INSTANT SHORT-TO-GROUND (Raw THW dropped {prev_thw} -> {trigger_thw}: Manual SPST Switch / Hard Ground Short)"
        return f"DECAYING SHORT-TO-GROUND (Raw THW={trigger_thw} < 41 ADC)"
    elif trigger_thw > 982:
        return f"OPEN-CIRCUIT / 5V PULL-UP FAULT (Raw THW climbed {prev_thw} -> {trigger_thw} > 982 ADC: Broken Wire or 'FFF' Test)"
    return f"INTERMITTENT / OUT-OF-RANGE SIGNAL (Trigger Raw THW={trigger_thw})"


csv_filename = get_next_csv_filename(BASE_NAME)
log_print(f"[HACHI DIAGNOSTICS] Connecting to ECU on {SERIAL_PORT}...")
log_print(f"[HACHI DIAGNOSTICS] Output Report: {csv_filename}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(0.1)

    # =========================================================================
    # STEP 1: PRIORITY CHECK — INTERROGATE EEPROM BEFORE TOUCHING ANY LATCHES!
    # =========================================================================
    log_print("\n[STEP 1] Interrogating EEPROM for Pre-Existing Field / Real-World Faults...")
    freeze_frame_data, history_records = interrogate_eeprom(ser)

    with open(csv_filename, "w", newline="") as log_file:
        writer = csv.writer(log_file)

        # ---------------------------------------------------------------------
        # MODE A: PRE-EXISTING REAL-WORLD FAULT FOUND IN EEPROM!
        # ---------------------------------------------------------------------
        if freeze_frame_data and freeze_frame_data[0] != 0:
            dtc_f  = freeze_frame_data[0]
            rpm_f  = freeze_frame_data[1]
            tps_f  = freeze_frame_data[2]
            peak_f = freeze_frame_data[3]
            diagnosis = diagnose_fault_root_cause(history_records)

            log_print("======================================================================")
            log_print(" [!] PRIORITY ALERT: REAL-WORLD FAULT RETRIEVED FROM EEPROM BLACK-BOX!")
            log_print("======================================================================")
            log_print(f" - Diagnostic Mode               : FIELD FAULT EXTRACTION (No 'FFF' Injected)")
            log_print(f" - Stored Diagnostic Trouble Code: Code {dtc_f}")
            log_print(f" - Engine Speed at Fault Moment  : {rpm_f} RPM")
            log_print(f" - Throttle Position at Fault    : {tps_f}%")
            log_print(f" - All-Time Peak Engine Speed    : {peak_f} RPM")
            log_print(f" - Automated Root-Cause Analysis : {diagnosis}")
            log_print(f" - 30-Slot Pre-Fault FIFO History: {len(history_records)}/30 SAMPLES RETRIEVED")

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

            # Record 30 frames of post-retrieval live telemetry & clear the fault latch
            log_print("\n[STEP 2] Clearing Latched Code ('RRR') & Resetting EEPROM ('CCC') for Next Drive...")
            ser.write(b'NNN\n')
            ser.write(b'RRR\n')
            ser.write(b'CCC\n')

            writer.writerow(["SimTime_ms", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC", "Test_Phase"])
            drain_to_live_edge(ser)
            recovery_verified = False

            for frame in range(30):
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
                if dtc == 0 and 41 <= thw <= 982:
                    recovery_verified = True
                writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc, "POST_EXTRACTION_RECOVERY"])

            log_print(f" - Live ECU Latch Reset & Armed  : {'VERIFIED [OK]' if recovery_verified else 'STILL FAULTED (Check SPST Switch!)'}")
            log_print("======================================================================")

        # ---------------------------------------------------------------------
        # MODE B: EEPROM IS CLEAN -> RUN PRELIMINARY 'FFF' BENCH SELF-TEST
        # ---------------------------------------------------------------------
        else:
            log_print("[STATUS CLEAN] No stored fault in EEPROM. Running Preliminary 'FFF' Bench Self-Test...")
            ser.write(b'NNN\n')
            ser.write(b'RRR\n')
            drain_to_live_edge(ser)

            fault_injected = False
            fault_inject_wall_time = time.perf_counter()
            fault_inject_sim_ms = 0
            wall_latency_ms = None
            sim_latency_ms = None
            recovery_verified = False
            fault_cleared_sent = False

            writer.writerow(["SimTime_ms", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC", "Test_Phase"])

            for frame in range(110):
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

                phase = "1_BASELINE_NORMAL"

                if frame == 30 and not fault_injected:
                    ser.write(b'FFF\n')
                    fault_injected = True
                    fault_inject_wall_time = time.perf_counter()
                    fault_inject_sim_ms = t_ms
                    log_print(f"[Frame {frame}] >>> Sent 'FFF': Injected Self-Test Fault (SimTime={t_ms}ms)!")

                if fault_injected and not fault_cleared_sent and frame < 70 and thw > 100:
                    ser.write(b'FFF\n')

                if dtc == 4 and fan == 1 and wall_latency_ms is None:
                    wall_latency_ms = (time.perf_counter() - fault_inject_wall_time) * 1000.0
                    sim_latency_ms = t_ms - fault_inject_sim_ms
                    log_print(f"[Frame {frame}] >>> Self-Test Fault Latched! (THW={thw}, Latency={wall_latency_ms:.1f}ms)")

                if fault_injected and wall_latency_ms is None:
                    phase = "2_FAULT_DECAYING"
                elif wall_latency_ms is not None and not fault_cleared_sent:
                    phase = "2_FAULT_LATCHED"

                if frame == 75:
                    ser.write(b'NNN\n')
                    log_print(f"[Frame {frame}] >>> Sent 'NNN': Restored THW Wire...")

                if frame == 90 and not fault_cleared_sent:
                    ser.write(b'RRR\n')
                    fault_cleared_sent = True
                    log_print(f"[Frame {frame}] >>> Sent 'RRR': Reset Live Fault Latch...")

                if fault_cleared_sent:
                    phase = "3_POST_RECOVERY"
                    if dtc == 0 and thw > 200:
                        recovery_verified = True

                writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc, phase])

            # Extract the Self-Test EEPROM log, then wipe EEPROM ('CCC') so it's clean for real-world faults!
            freeze_frame_data, history_records = interrogate_eeprom(ser)
            ser.write(b'CCC\n')
            diagnosis = diagnose_fault_root_cause(history_records)

            log_print("======================================================================")
            log_print(f"       HACHI AUTOMATED SELF-TEST SCORECARD ({csv_filename})")
            log_print("======================================================================")
            log_print(f" 1. Self-Test Fault Triggered    : {'PASSED [OK]' if wall_latency_ms is not None else 'FAILED'}")
            log_print(f" 2. Self-Healing & Latch Reset   : {'PASSED [OK]' if recovery_verified else 'FAILED'}")
            log_print(f" 3. EEPROM Self-Test Verified    : {'PASSED [OK]' if (freeze_frame_data and freeze_frame_data[0] != 0) else 'FAILED'}")
            log_print(f" 4. Root-Cause Signature         : {diagnosis}")
            log_print(" 5. Post-Test EEPROM Cleanup     : CLEARED ('CCC') - Ready for Field Use!")
            log_print("======================================================================")

        # Write EEPROM FIFO Table and Terminal Log to CSV
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
    print(f"\n[COMPLETE] Diagnostic Report saved to: {csv_filename}")

except Exception as e:
    print(f"[ERROR] Serial connection issue on {SERIAL_PORT}: {e}")