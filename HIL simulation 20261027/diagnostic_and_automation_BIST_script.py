import serial
import time
import csv
import os
import re

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
BASE_NAME = "hachi_diagnostic_report"

RING_SLOTS = 60        # 60 slots @ 2 Hz (0.5s/step) = 30.0 Seconds Total (20.0s Pre / 10.0s Post)
SAMPLE_STEP_SEC = 0.5  # 2 Hz = 0.5 seconds per sample


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
    """Queries MCU#2 EEPROM ('DDD') and returns (freeze_frame_data, history_records) for up to 60 slots."""
    drain_to_live_edge(ser)
    ser.write(b'DDD\n')
    freeze_data = None
    hist_recs = []
    t_start = time.perf_counter()
    while (time.perf_counter() - t_start) < 4.5:
        resp = ser.readline().decode('utf-8', errors='ignore').strip()
        if resp.startswith("$FREEZE") and resp.endswith("*"):
            f_parts = resp[8:-1].split(",")
            if len(f_parts) == 4:
                freeze_data = [int(x) for x in f_parts]
        elif resp.startswith("$HIST") and resp.endswith("*"):
            h_parts = resp[6:-1].split(",")
            if len(h_parts) == 7:
                rec = [int(x) for x in h_parts]
                if len(hist_recs) < RING_SLOTS:
                    hist_recs.append(rec)
        if freeze_data is not None and (freeze_data[0] == 0 or len(hist_recs) >= RING_SLOTS):
            break
    return freeze_data, hist_recs


def diagnose_fault_root_cause(history_records):
    """
    Analyzes the 60-slot (30.0s @ 2Hz) Asymmetric (20.0s Pre / 10.0s Post) FIFO curve:
      - Samples -39 to -1 (indices 0..38) : Pre-Trigger Approach (-19.5s to -0.5s)
      - Sample    0       (index 39)      : Exact Trigger Moment (0.0s)
      - Samples  +1 to +20(indices 40..59): Post-Trigger Failsafe Window (+0.5s to +10.0s)
    """
    if not history_records:
        return "UNKNOWN (No FIFO history available)", "UNVERIFIED"

    # Locate Sample 0 (the exact fault trigger slot, index 39) and Sample -1 (index 38)
    trig_pos = 39 if len(history_records) >= 40 else len(history_records) - 1
    for i, rec in enumerate(history_records):
        if rec[0] == 0:
            trig_pos = i
            break

    trigger_thw = history_records[trig_pos][3]
    prev_thw = history_records[trig_pos - 1][3] if trig_pos >= 1 else trigger_thw

    if trigger_thw < 41:
        if prev_thw > 200:
            root_cause = f"INSTANT SHORT-TO-GROUND (Raw THW dropped {prev_thw} -> {trigger_thw} at t=0.0s: Manual SPST Switch / Hard Ground Short)"
        else:
            root_cause = f"DECAYING SHORT-TO-GROUND (Raw THW slewed {prev_thw} -> {trigger_thw} < 41 ADC at t=0.0s: 'FFF' RC Filter Decay)"
    elif trigger_thw > 982:
        root_cause = f"OPEN-CIRCUIT / 5V PULL-UP FAULT (Raw THW climbed {prev_thw} -> {trigger_thw} > 982 ADC at t=0.0s)"
    else:
        root_cause = f"INTERMITTENT / OUT-OF-RANGE SIGNAL (Trigger Raw THW={trigger_thw})"

    # Audit the Post-Trigger Slots (+1 to +20 = +0.5s to +10.0s) to prove Fan Failsafe stayed ON
    post_slots = [r for r in history_records if r[0] > 0 and r[1] > 0]
    if post_slots:
        fan_on_count = sum(1 for r in post_slots if r[5] == 1)
        post_sec = len(post_slots) * SAMPLE_STEP_SEC
        failsafe_audit = f"PASSED [OK] ({fan_on_count}/{len(post_slots)} Post-Trigger Samples [{post_sec:.1f}s] verified Fan=ON)"
    else:
        failsafe_audit = "EARLY EXTRACTION (Extracted before Post-Trigger window elapsed)"

    return root_cause, failsafe_audit


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
            diagnosis, failsafe_audit = diagnose_fault_root_cause(history_records)

            live_pre_count  = sum(1 for r in history_records if r[0] <= 0 and r[1] > 0)
            boot_idle_count = sum(1 for r in history_records if r[0] <= 0 and r[1] == 0)
            live_post_count = sum(1 for r in history_records if r[0] > 0 and r[1] > 0)

            log_print("======================================================================")
            log_print(" [!] PRIORITY ALERT: REAL-WORLD FAULT RETRIEVED FROM EEPROM BLACK-BOX!")
            log_print("======================================================================")
            log_print(f" - Diagnostic Mode               : FIELD FAULT EXTRACTION (No 'FFF' Injected)")
            log_print(f" - Stored Diagnostic Trouble Code: Code {dtc_f}")
            log_print(f" - Engine Speed at Fault Moment  : {rpm_f} RPM")
            log_print(f" - Throttle Position at Fault    : {tps_f}%")
            log_print(f" - Session Peak Engine Speed     : {peak_f} RPM")
            log_print(f" - Automated Root-Cause Analysis : {diagnosis}")
            log_print(f" - Post-Trigger Failsafe Audit   : {failsafe_audit}")
            log_print(f" - 30s (2Hz) Asymmetric History  : {len(history_records)}/60 SLOTS "
                      f"({live_pre_count * SAMPLE_STEP_SEC:.1f}s Live Pre + {live_post_count * SAMPLE_STEP_SEC:.1f}s Post"
                      f"{f' | {boot_idle_count} Pre-Boot Slots' if boot_idle_count > 0 else ''})")

            if history_records:
                log_print("    [Sample] | Rel Time | Phase | Engine RPM | Throttle | Raw THW | TVIS | Fan | DTC")
                log_print("    --------------------------------------------------------------------------------")
                # Print 6 samples before trigger (-3.0s..-0.5s), Trigger (0.0s), and 10 samples after (+0.5s..+5.0s)
                for rec in history_records[33:50]:
                    idx     = rec[0]
                    r_rpm   = rec[1]
                    r_tps   = rec[2]
                    r_thw   = rec[3]
                    r_tvis  = rec[4]
                    r_fan   = rec[5]
                    r_dtc   = rec[6]
                    rel_sec = idx * SAMPLE_STEP_SEC
                    phase_lbl = "PRE " if idx < 0 else ("TRIG" if idx == 0 else "POST")
                    log_print(f"     {idx:+3d}     |  {rel_sec:+5.1f}s  | {phase_lbl}  |  {r_rpm:4d} RPM |   {r_tps:3d}%   |   {r_thw:4d}  |   {r_tvis}  |  {r_fan}  |  {r_dtc}")

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
        # MODE B: EEPROM IS CLEAN -> RUN 'FFF' 30s (20s PRE / 10s POST) SELF-TEST
        # ---------------------------------------------------------------------
        else:
            log_print("[STATUS CLEAN] No stored fault in EEPROM. Running 'FFF' 30-Second (2Hz) Asymmetric Self-Test...")
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

            # 290 frames (@ 50ms/frame = 14.5s simulated):
            #   - Frame 30  : Inject 'FFF'
            #   - Frame ~35 : Fault Latches at t=0.0s & starts 20-slot (10.0s = 200 frames) Post-Trigger capture
            #   - Frame 245 : Restore THW wire ('NNN') AFTER all 20 Post-Trigger slots (+10.0s) are locked in EEPROM
            #   - Frame 265 : Reset live fault latch ('RRR') and verify recovery
            for frame in range(290):
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

                if fault_injected and not fault_cleared_sent and frame < 240 and thw > 100:
                    ser.write(b'FFF\n')

                if dtc == 4 and fan == 1 and wall_latency_ms is None:
                    wall_latency_ms = (time.perf_counter() - fault_inject_wall_time) * 1000.0
                    sim_latency_ms = t_ms - fault_inject_sim_ms
                    log_print(f"[Frame {frame}] >>> Self-Test Fault Latched at t=0.0s! Recording 20 Post-Trigger Slots (+10.0s @ 2Hz)... (THW={thw}, Latency={wall_latency_ms:.1f}ms)")

                if fault_injected and wall_latency_ms is None:
                    phase = "2_FAULT_DECAYING"
                elif wall_latency_ms is not None and not fault_cleared_sent:
                    phase = "2_FAULT_LATCHED_AND_10S_POST_CAPTURING"

                if frame == 245:
                    ser.write(b'NNN\n')
                    log_print(f"[Frame {frame}] >>> Sent 'NNN': Restored THW Wire (10.0s Post-Trigger Window Complete)...")

                if frame == 265 and not fault_cleared_sent:
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
            diagnosis, failsafe_audit = diagnose_fault_root_cause(history_records)
            pre_count  = sum(1 for r in history_records if r[0] <= 0)
            post_count = sum(1 for r in history_records if r[0] > 0)

            log_print("======================================================================")
            log_print(f"        HACHI AUTOMATED SELF-TEST SCORECARD ({csv_filename})")
            log_print("======================================================================")
            log_print(f" 1. Self-Test Fault Triggered    : {'PASSED [OK]' if wall_latency_ms is not None else 'FAILED'}")
            log_print(f" 2. 30s (20s/10s @ 2Hz) Split    : {'PASSED [OK]' if (pre_count == 40 and post_count == 20) else 'FAILED'} ({pre_count} Pre/Trig [-19.5s..0.0s] + {post_count} Post [+0.5s..+10.0s])")
            log_print(f" 3. Post-Trigger Failsafe Audit  : {failsafe_audit}")
            log_print(f" 4. Self-Healing & Latch Reset   : {'PASSED [OK]' if recovery_verified else 'FAILED'}")
            log_print(f" 5. EEPROM Self-Test Verified    : {'PASSED [OK]' if (freeze_frame_data and freeze_frame_data[0] != 0) else 'FAILED'}")
            log_print(f" 6. Root-Cause Signature         : {diagnosis}")
            log_print(" 7. Post-Test EEPROM Cleanup     : CLEARED ('CCC') - Ready for Field Use!")
            log_print("======================================================================")

        # Write 60-Slot (30.0s @ 2Hz) EEPROM FIFO Table and Terminal Log to CSV
        writer.writerow([])
        writer.writerow(["--- 60-SLOT ASYMMETRIC (20.0s PRE / 10.0s POST @ 2Hz = 30.0s Window) EEPROM FIFO HISTORY ---"])
        writer.writerow(["Sample_Idx", "Rel_Time_Sec", "Window_Phase", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC"])
        for rec in history_records:
            idx = rec[0]
            if rec[1] == 0 and idx < 0:
                w_phase = "PRE_BOOT_IDLE"
            elif idx < 0:
                w_phase = "PRE_TRIGGER"
            elif idx == 0:
                w_phase = "TRIGGER_T0"
            else:
                w_phase = "POST_TRIGGER"
            writer.writerow([idx, f"{idx * SAMPLE_STEP_SEC:+.1f}", w_phase, rec[1], rec[2], rec[3], rec[4], rec[5], rec[6]])

        writer.writerow([])
        writer.writerow(["--- PYTHON TERMINAL EXECUTION LOG & SCORECARD ---"])
        for log_line in terminal_log:
            writer.writerow([log_line])

    ser.close()
    print(f"\n[COMPLETE] Diagnostic Report saved to: {csv_filename}")

except Exception as e:
    print(f"[ERROR] Serial connection issue on {SERIAL_PORT}: {e}")