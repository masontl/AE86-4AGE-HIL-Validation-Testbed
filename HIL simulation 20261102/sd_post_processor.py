# ==============================================================================
# LESSON 12: 4-LAYER OFFLINE SD CARD LOG DISSECTOR (sd_post_processor.py)
# Performs Quad-Layer Flight Data Analysis:
#   1. Layer 1 — Hard OBD-I Sensor Faults (Code 4 THW / Code 7 TPS)
#   2. Layer 2 — Thermal Warmup, Bi-Metal AAV & Open-to-Closed-Loop Entry Audit
#   3. Layer 3 — Closed-Loop-Gated Soft Fuel Trim Dissection (Vacuum Leak Audit)
#   4. Layer 4 — VAST Igniter IGf Handshake & Code 14 Fuel-Cut Failsafe Audit
# ==============================================================================

import csv
import os
import re

SD_CARD_FILENAME = "VIRTUAL_SD_512MB.CSV"
ARCHIVE_BASE_NAME = "hachi_mission_archive"
WIPE_SD_AFTER_ARCHIVE = True


def get_next_archive_filename(base_name):
    max_num = -1
    pattern = re.compile(rf"^{re.escape(base_name)}_(\d+)\.csv$", re.IGNORECASE)
    for fname in os.listdir("."):
        match = pattern.match(fname)
        if match:
            num = int(match.group(1))
            if num > max_num:
                max_num = num
    return f"{base_name}_{max_num + 1:02d}.csv"


def main():
    if not os.path.exists(SD_CARD_FILENAME) or os.path.getsize(SD_CARD_FILENAME) == 0:
        print(f"[!] No data found on '{SD_CARD_FILENAME}'. Run hachi_read_SDimg_create_csv.py first!")
        return

    rows = []
    with open(SD_CARD_FILENAME, mode="r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for r in reader:
            if len(r) in (13, 14, 15):
                try:
                    loop_cl = int(r[13]) if len(r) >= 14 else (1 if int(r[5]) >= 140 else 0)
                    igf_ok  = int(r[14]) if len(r) == 15 else (0 if int(r[8]) == 14 else 1)
                    rows.append({
                        "host_ts":       float(r[0]),
                        "sim_ms":        int(r[1]),
                        "rpm":           int(r[2]),
                        "tps":           int(r[3]),
                        "raw_thw":       int(r[4]),
                        "coolant_f":     int(r[5]),
                        "tvis":          int(r[6]),
                        "fan":           int(r[7]),
                        "live_dtc":      int(r[8]),
                        "latched_dtc":   int(r[9]),
                        "ox_mv":         int(r[10]),
                        "vf_mv":         int(r[11]),
                        "fuel_trim_pct": int(r[12]),
                        "loop_cl":       loop_cl,
                        "igf_ok":        igf_ok,
                    })
                except ValueError:
                    continue

    if not rows:
        print("[!] SD card file contained header only (0 valid telemetry frames).")
        return

    report_lines = []
    def log_p(msg):
        print(msg)
        report_lines.append(msg)

    archive_file = get_next_archive_filename(ARCHIVE_BASE_NAME)
    total_duration_s = (rows[-1]["sim_ms"] - rows[0]["sim_ms"]) / 1000.0
    valid_temps = [r["coolant_f"] for r in rows if r["coolant_f"] > 0]
    start_temp = valid_temps[0] if valid_temps else -99
    peak_temp  = max(valid_temps) if valid_temps else -99

    # --------------------------------------------------------------------------
    # LAYER 1: Scan for Hard OBD-I Sensor Fault Onset Edges (Code 4 / Code 7)
    # --------------------------------------------------------------------------
    hard_sensor_events = []
    for idx, r in enumerate(rows):
        prev_dtc = rows[idx - 1]["live_dtc"] if idx > 0 else 0
        if r["live_dtc"] in (4, 7) and prev_dtc != r["live_dtc"]:
            hard_sensor_events.append(idx)

    # --------------------------------------------------------------------------
    # LAYER 2: Thermal Warmup & Open-to-Closed-Loop Transition Audit
    # --------------------------------------------------------------------------
    ol_frames = [r for r in rows if r["loop_cl"] == 0 and r["live_dtc"] == 0]
    cl_frames = [r for r in rows if r["loop_cl"] == 1 and r["live_dtc"] == 0]
    ol_pct = (len(ol_frames) / len(rows)) * 100.0
    cl_pct = (len(cl_frames) / len(rows)) * 100.0

    cl_entry_idx = next(
        (i for i in range(1, len(rows)) if rows[i]["loop_cl"] == 1 and rows[i - 1]["loop_cl"] == 0),
        0 if rows[0]["loop_cl"] == 1 else None
    )

    cl_dropouts = [
        i for i in range(1, len(rows))
        if rows[i]["loop_cl"] == 0 and rows[i - 1]["loop_cl"] == 1 and rows[i]["live_dtc"] == 0
    ]

    # --------------------------------------------------------------------------
    # LAYER 3: Closed-Loop-Gated Soft Fuel Trim Saturation Events (Loop_CL == 1)
    # --------------------------------------------------------------------------
    soft_trim_events = []
    in_soft_event = False
    start_idx = 0

    for idx, r in enumerate(rows):
        is_pinned_rich = (r["loop_cl"] == 1 and r["ox_mv"] < 250 and r["fuel_trim_pct"] >= 15)
        if is_pinned_rich and not in_soft_event:
            in_soft_event = True
            start_idx = idx
        elif not is_pinned_rich and in_soft_event:
            in_soft_event = False
            if (idx - start_idx) >= 10:
                soft_trim_events.append((start_idx, idx - 1))

    if in_soft_event and (len(rows) - start_idx) >= 10:
        soft_trim_events.append((start_idx, len(rows) - 1))

    low_load_trims  = [r["fuel_trim_pct"] for r in cl_frames if r["rpm"] < 3800]
    high_load_trims = [r["fuel_trim_pct"] for r in cl_frames if r["rpm"] >= 3800]
    avg_low_trim    = sum(low_load_trims) / len(low_load_trims) if low_load_trims else 0.0
    avg_high_trim   = sum(high_load_trims) / len(high_load_trims) if high_load_trims else 0.0

    # --------------------------------------------------------------------------
    # LAYER 4: VAST Igniter IGf Confirmation & Code 14 Fuel-Cut Dissection
    # --------------------------------------------------------------------------
    igf_fault_events = []
    in_igf_event = False
    igf_start = 0

    for idx, r in enumerate(rows):
        is_igf_lost = (r["igf_ok"] == 0 or r["live_dtc"] == 14)
        if is_igf_lost and not in_igf_event:
            in_igf_event = True
            igf_start = idx
        elif not is_igf_lost and in_igf_event:
            in_igf_event = False
            igf_fault_events.append((igf_start, idx - 1))

    if in_igf_event:
        igf_fault_events.append((igf_start, len(rows) - 1))

    log_p("=" * 84)
    log_p(f"  HACHI FLIGHT DATA RECORDER — LESSON 12 4-LAYER DEBRIEF ({archive_file})")
    log_p("=" * 84)
    log_p(f"  - Total 20Hz Frames Dissected      : {len(rows):,} frames ({total_duration_s:.2f}s of driving)")
    log_p(f"  - Thermal Envelope (Start -> Peak) : {start_temp} deg F -> {peak_temp} deg F")
    log_p(f"  - TCCS Loop Split (Open / Closed)  : {ol_pct:.1f}% Open-Loop | {cl_pct:.1f}% Closed-Loop")
    log_p(f"  - Hard Sensor Fault Incidents      : {len(hard_sensor_events)} incident(s) (Code 4 / Code 7)")
    log_p(f"  - VAST Igniter Code 14 Incidents   : {len(igf_fault_events)} incident(s) (Missing IGf -> Fuel-Cut!)")
    log_p(f"  - Soft Fuel-Trim Anomaly Incidents : {len(soft_trim_events)} incident(s) (Silent / No CEL!)")
    log_p("-" * 84)

    # Report Layer 1: Hard Sensor Faults
    if hard_sensor_events:
        log_p("  [SECTION 1: HARD OBD-I SENSOR FAULT DISSECTION (CODE 4 / CODE 7)]")
        for ev_num, trig_idx in enumerate(hard_sensor_events, start=1):
            trig = rows[trig_idx]
            prev = rows[trig_idx - 1] if trig_idx > 0 else trig
            cause = f"RC-FILTERED OR SWITCH SHORT (RawTHW {prev['raw_thw']} -> {trig['raw_thw']})"
            post_end = min(len(rows), trig_idx + 200)
            fan_failsafe_ok = all(x["fan"] == 1 for x in rows[trig_idx:post_end])
            log_p(f"  * Hard Sensor Incident #{ev_num} @ SimTime = {trig['sim_ms']} ms (Code {trig['live_dtc']}):")
            log_p(f"    - Signature & Fan Failsafe : {cause} | Fan = {'100% ON [OK]' if fan_failsafe_ok else 'FAILED'}")
        log_p("-" * 84)

    # Report Layer 2: Warmup & Closed-Loop Entry Audit
    log_p("  [SECTION 2: COLD-START WARMUP, AAV IDLE-UP & CLOSED-LOOP ENTRY AUDIT]")
    if ol_frames:
        min_ol_rpm = min(r["rpm"] for r in ol_frames)
        min_cl_rpm = min(r["rpm"] for r in cl_frames) if cl_frames else min_ol_rpm
        log_p(f"  * Bi-Metal AAV Idle-Up Floor : {min_ol_rpm} RPM (Cold Open-Loop) -> {min_cl_rpm} RPM (Warm Closed-Loop)")

    if cl_entry_idx is not None:
        entry_row = rows[cl_entry_idx]
        warmup_s  = (entry_row["sim_ms"] - rows[0]["sim_ms"]) / 1000.0
        log_p(f"  * Closed-Loop Entry Achieved : YES @ SimTime = {entry_row['sim_ms']} ms ({warmup_s:.2f}s warmup)")
        if cl_dropouts:
            drop_row = rows[cl_dropouts[0]]
            log_p(f"  * [!] THERMAL DROPOUT ALERT  : ECU fell BACK into Open-Loop @ {drop_row['sim_ms']} ms ({drop_row['coolant_f']} deg F < 140 deg F!)")
    else:
        log_p(f"  * Closed-Loop Entry Achieved : FAILED (Peak Coolant = {peak_temp} deg F < 140 deg F)")
    log_p("-" * 84)

    # Report Layer 3: Closed-Loop-Gated Soft Fuel Trim Dissection
    log_p("  [SECTION 3: CLOSED-LOOP FUEL TRIM & VACUUM LEAK DISSECTION (GATED: Loop_CL == 1)]")
    if not cl_frames:
        log_p("  * Closed-Loop Trim Audit     : SKIPPED (No Closed-Loop frames recorded)")
    else:
        log_p(f"  * Low-Load (<3800 RPM) Avg Trim: {avg_low_trim:+.1f}% | High-Load (>=3800 RPM) Avg Trim: {avg_high_trim:+.1f}%")
        if not soft_trim_events:
            log_p("  * Closed-Loop Health Verdict : HEALTHY STOICHIOMETRIC DITHERING (No Vacuum Leak Detected)")
        else:
            log_p(f"  * Vacuum Leak Events Caught  : {len(soft_trim_events)} low-load rich-trim saturation window(s) confirmed!")
    log_p("-" * 84)

    # Report Layer 4: VAST Igniter IGf Handshake & Code 14 Fuel-Cut Dissection
    log_p("  [SECTION 4: VAST IGNITER IGf CONFIRMATION & CODE 14 FUEL-CUT FAILSAFE AUDIT]")
    if not igf_fault_events:
        log_p("  * Dual-ISR Handshake Status  : 100% HEALTHY (Every IG- spark confirmed by VAST IGf return pulse)")
    else:
        for i_num, (i_start, i_end) in enumerate(igf_fault_events, start=1):
            slice_r  = rows[i_start:i_end + 1]
            dur_s    = (rows[i_end]["sim_ms"] - rows[i_start]["sim_ms"]) / 1000.0
            onset_r  = rows[i_start]
            # Allow 3 frames (~150ms) for RC low-pass filters on VF and OX to settle after fuel-cut
            settle_r = rows[min(i_end, i_start + 3):i_end + 1]
            min_ox   = min(x["ox_mv"] for x in settle_r) if settle_r else onset_r["ox_mv"]
            max_vf   = max(x["vf_mv"] for x in settle_r) if settle_r else onset_r["vf_mv"]
            cl_cut   = all(x["loop_cl"] == 0 for x in slice_r)
            fuel_cut_verified = (cl_cut and max_vf < 350 and min_ox < 150)

            heal_str = "Self-Healed ('N' Command Restored IGf)" if i_end < len(rows) - 1 else "Active to End of Log"

            log_p(f"  * Code 14 Incident #{i_num} @ SimTime = {onset_r['sim_ms']} ms -> {rows[i_end]['sim_ms']} ms ({dur_s:.2f}s):")
            log_p(f"    - Dual-ISR Watchdog Catch  : INT0 (IG-) spinning @ {onset_r['rpm']} RPM while INT1 (IGf) = 0 pulses!")
            log_p(f"    - OBD-I Fault Latched      : Live_DTC = {onset_r['live_dtc']} | Latched_DTC = {onset_r['latched_dtc']} (Code 14: No IGf Signal)")
            log_p(f"    - TCCS Fuel-Cut Telemetry  : Loop_CL -> 0 | VF -> {max_vf} mV (0.0V) | OX -> {min_ox} mV (Dead-Lean Air)")
            log_p(f"    - Catalytic Converter Safe : {'VERIFIED FUEL-CUT ENGAGED [PASS]' if fuel_cut_verified else 'FAILED TO CUT FUEL [FAIL]'}")
            log_p(f"    - Recovery Status          : {heal_str}")

    log_p("=" * 84)

    # Save Complete 14-Channel Archive
    with open(archive_file, mode="w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow(["--- OFFLINE FLIGHT LOG DISSECTOR REPORT (LESSON 12) ---"])
        for line in report_lines:
            writer.writerow([line])
        writer.writerow([])
        writer.writerow(["--- FULL CONTINUOUS 20Hz SPI SD TELEMETRY LOG (14 CHANNELS) ---"])
        writer.writerow(header)
        for r in rows:
            writer.writerow([
                f"{r['host_ts']:.3f}", r["sim_ms"], r["rpm"], r["tps"], r["raw_thw"],
                r["coolant_f"], r["tvis"], r["fan"], r["live_dtc"], r["latched_dtc"],
                r["ox_mv"], r["vf_mv"], r["fuel_trim_pct"], r["loop_cl"], r["igf_ok"]
            ])

    print(f"\n[+] Repository Archive Saved : {archive_file}")

    if WIPE_SD_AFTER_ARCHIVE:
        with open(SD_CARD_FILENAME, mode="w", newline="") as f:
            pass
        print(f"[+] SD Card Wiped & Reset    : '{SD_CARD_FILENAME}' is now 0 KB and ready for next test run!")


if __name__ == "__main__":
    main()