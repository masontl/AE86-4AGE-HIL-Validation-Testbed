# ==============================================================================
# LESSON 11: 3-LAYER OFFLINE SD CARD LOG DISSECTOR (sd_post_processor.py)
# Performs Triple-Layer Flight Data Analysis:
#   1. Layer 1 — Hard OBD-I Electrical Faults (Live_DTC != 0: Switch vs. RC Slew)
#   2. Layer 2 — Thermal Warmup, Bi-Metal AAV & Open-to-Closed-Loop Entry Audit
#                (Detects Stuck-Open Thermostat / Failure to Reach 140°F CL Gate)
#   3. Layer 3 — Closed-Loop-Gated Soft Fuel Trim Dissection (Only evaluates
#                Loop_CL == 1 frames to isolate Intake/Brake Booster Vacuum Leaks!)
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
            if len(r) in (13, 14):
                try:
                    loop_cl = int(r[13]) if len(r) == 14 else (1 if int(r[5]) >= 140 else 0)
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
    peak_rpm = max(r["rpm"] for r in rows)
    valid_temps = [r["coolant_f"] for r in rows if r["coolant_f"] > 0]
    start_temp = valid_temps[0] if valid_temps else -99
    peak_temp  = max(valid_temps) if valid_temps else -99

    # --------------------------------------------------------------------------
    # LAYER 1: Scan for Hard OBD-I Fault Onset Edges (Live_DTC: 0 -> non-zero)
    # --------------------------------------------------------------------------
    hard_fault_events = []
    for idx, r in enumerate(rows):
        prev_dtc = rows[idx - 1]["live_dtc"] if idx > 0 else 0
        if r["live_dtc"] != 0 and prev_dtc == 0:
            hard_fault_events.append(idx)

    # --------------------------------------------------------------------------
    # LAYER 2: Thermal Warmup & Open-to-Closed-Loop Transition Audit
    # --------------------------------------------------------------------------
    ol_frames = [r for r in rows if r["loop_cl"] == 0 and r["live_dtc"] == 0]
    cl_frames = [r for r in rows if r["loop_cl"] == 1 and r["live_dtc"] == 0]
    ol_pct = (len(ol_frames) / len(rows)) * 100.0
    cl_pct = (len(cl_frames) / len(rows)) * 100.0

    # Find exact frame where TCCS transitions from Open-Loop (0) to Closed-Loop (1)
    cl_entry_idx = next(
        (i for i in range(1, len(rows)) if rows[i]["loop_cl"] == 1 and rows[i - 1]["loop_cl"] == 0),
        0 if rows[0]["loop_cl"] == 1 else None
    )

    # Check if Closed-Loop ever dropped BACK to Open-Loop due to Stuck-Open Thermostat ('S') or Cold Reset ('C')
    cl_dropouts = [
        i for i in range(1, len(rows))
        if rows[i]["loop_cl"] == 0 and rows[i - 1]["loop_cl"] == 1 and rows[i]["live_dtc"] == 0
    ]

    # --------------------------------------------------------------------------
    # LAYER 3: Closed-Loop-Gated Soft Fuel Trim Saturation Events
    # Strictly requires Loop_CL == 1 so Open-Loop (VF=0V) never corrupts trim!
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

    # Compute Load-Regime Fuel Trim Averages STRICTLY across Closed-Loop frames!
    low_load_trims  = [r["fuel_trim_pct"] for r in cl_frames if r["rpm"] < 3800]
    high_load_trims = [r["fuel_trim_pct"] for r in cl_frames if r["rpm"] >= 3800]
    avg_low_trim    = sum(low_load_trims) / len(low_load_trims) if low_load_trims else 0.0
    avg_high_trim   = sum(high_load_trims) / len(high_load_trims) if high_load_trims else 0.0

    log_p("=" * 82)
    log_p(f"  HACHI FLIGHT DATA RECORDER — LESSON 11 3-LAYER DEBRIEF ({archive_file})")
    log_p("=" * 82)
    log_p(f"  - Total 20Hz Frames Dissected      : {len(rows):,} frames ({total_duration_s:.2f}s of driving)")
    log_p(f"  - Thermal Envelope (Start -> Peak) : {start_temp} deg F -> {peak_temp} deg F")
    log_p(f"  - TCCS Loop Split (Open / Closed)  : {ol_pct:.1f}% Open-Loop | {cl_pct:.1f}% Closed-Loop")
    log_p(f"  - Hard OBD-I Electrical Incidents  : {len(hard_fault_events)} incident(s) (Triggered CEL)")
    log_p(f"  - Soft Fuel-Trim Anomaly Incidents : {len(soft_trim_events)} incident(s) (Silent / No CEL!)")
    log_p("-" * 82)

    # Report Layer 1: Hard OBD-I Faults
    if hard_fault_events:
        log_p("  [SECTION 1: HARD OBD-I ELECTRICAL FAULT DISSECTION]")
        for ev_num, trig_idx in enumerate(hard_fault_events, start=1):
            trig = rows[trig_idx]
            prev = rows[trig_idx - 1] if trig_idx > 0 else trig

            if trig["raw_thw"] < 41:
                if prev["raw_thw"] > 180:
                    cause = f"INSTANT SHORT-TO-GROUND (RawTHW {prev['raw_thw']} -> {trig['raw_thw']} in 50ms: Manual SPST Switch)"
                else:
                    cause = f"RC-FILTERED SHORT-TO-GROUND (RawTHW slewed {prev['raw_thw']} -> {trig['raw_thw']} < 41 ADC: 'F' Cmd)"
            elif trig["raw_thw"] > 982:
                cause = f"OPEN-CIRCUIT / 5V PULL-UP FAULT (RawTHW={trig['raw_thw']} > 982 ADC)"
            else:
                cause = f"TRANSIENT SENSOR GLITCH (RawTHW={trig['raw_thw']})"

            post_end = min(len(rows), trig_idx + 200)
            fan_failsafe_ok = all(x["fan"] == 1 for x in rows[trig_idx:post_end])
            heal_idx = next((i for i in range(trig_idx + 1, len(rows)) if rows[i]["live_dtc"] == 0), None)
            duration_str = f"{(rows[heal_idx]['sim_ms'] - trig['sim_ms']) / 1000.0:.2f}s (Self-Healed)" if heal_idx else "Active to End of Log"

            log_p(f"  * Hard Incident #{ev_num} @ SimTime = {trig['sim_ms']} ms (Code {trig['live_dtc']}):")
            log_p(f"    - Root-Cause Signature  : {cause}")
            log_p(f"    - Fault Duration        : {duration_str}")
            log_p(f"    - Fan Failsafe Response : {'VERIFIED 100% ON [OK]' if fan_failsafe_ok else 'FAILED'}")
        log_p("-" * 82)

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
        log_p(f"    - Crossover Telemetry      : Coolant = {entry_row['coolant_f']} deg F | VF stepped 0 mV -> {entry_row['vf_mv']} mV")
        if cl_dropouts:
            drop_row = rows[cl_dropouts[0]]
            log_p(f"  * [!] THERMAL DROPOUT ALERT  : ECU fell BACK into Open-Loop @ {drop_row['sim_ms']} ms (Coolant dropped to {drop_row['coolant_f']} deg F < 140 deg F!)")
            log_p("    - Probable Root Cause      : Stuck-Open Thermostat ('S') or Cold Reset ('C') forcing Rich Warmup Enrichment!")
    else:
        log_p(f"  * Closed-Loop Entry Achieved : FAILED (0% Closed-Loop Frames! Peak Coolant = {peak_temp} deg F < 140 deg F)")
        log_p("  * AUTOMATED THERMAL VERDICT  : STUCK-OPEN THERMOSTAT / WARMUP FAILURE CONFIRMED!")
        log_p("    -> Explanation: Engine never reached the 140 deg F TCCS Closed-Loop threshold.")
        log_p("       The ECU remained locked in Open-Loop (VF = 0.0V) with continuous cold-start")
        log_p("       fuel enrichment, causing severe fuel economy loss with ZERO Check Engine Light!")
    log_p("-" * 82)

    # Report Layer 3: Closed-Loop-Gated Soft Mechanical / Fuel Trim Dissection
    log_p("  [SECTION 3: CLOSED-LOOP FUEL TRIM & VACUUM LEAK DISSECTION (GATED: Loop_CL == 1)]")
    if not cl_frames:
        log_p("  * Closed-Loop Trim Audit     : SKIPPED (Engine never entered Closed-Loop during this mission)")
    else:
        log_p("  * Closed-Loop Load-Regime Trim Correlation (Open-Loop Warmup Frames Excluded):")
        log_p(f"    - Low-Load / High-Vacuum (< 3800 RPM) Avg CL Trim : {avg_low_trim:+.1f}%")
        log_p(f"    - High-Load / WOT       (>= 3800 RPM) Avg CL Trim : {avg_high_trim:+.1f}%")

        if not soft_trim_events:
            log_p("  * Closed-Loop Health Verdict : HEALTHY STOICHIOMETRIC DITHERING (No Vacuum Leak Detected)")
        else:
            for s_num, (s_start, s_end) in enumerate(soft_trim_events, start=1):
                slice_rows = rows[s_start:s_end + 1]
                dur_s      = (rows[s_end]["sim_ms"] - rows[s_start]["sim_ms"]) / 1000.0
                mean_rpm   = sum(x["rpm"] for x in slice_rows) // len(slice_rows)
                min_ox     = min(x["ox_mv"] for x in slice_rows)
                max_vf     = max(x["vf_mv"] for x in slice_rows)
                max_trim   = max(x["fuel_trim_pct"] for x in slice_rows)

                after_idx = min(len(rows) - 1, s_end + 1)
                if rows[after_idx]["loop_cl"] == 0:
                    exit_reason = "ECU Dropped into Open-Loop (< 140 deg F) — VF Integrator Disabled (0.0V)"
                elif rows[after_idx]["rpm"] >= 3800:
                    exit_reason = f"Masked by High Intake Airflow as RPM climbed past {rows[after_idx]['rpm']} RPM (Low Manifold Vacuum)"
                else:
                    exit_reason = "Vacuum Leak Mechanically Sealed ('N' Command)"

                log_p(f"\n  [SOFT ANOMALY #{s_num} @ SimTime = {rows[s_start]['sim_ms']} ms -> {rows[s_end]['sim_ms']} ms ({dur_s:.2f}s)]")
                log_p("    * OBD-I Check Engine Light : OFF (Live_DTC = 0 — Invisible to 1986 ECU!)")
                log_p(f"    * Telemetry Signature      : OX pinned LEAN ({min_ox} mV) while VF maxed RICH ({max_vf} mV / {max_trim:+d}%)")
                log_p(f"    * Mean Engine Speed        : {mean_rpm} RPM (High Manifold Vacuum Regime)")
                log_p(f"    * Recovery / Exit Behavior : {exit_reason}")

            if avg_low_trim > (avg_high_trim + 5.0) or any(rows[min(len(rows)-1, e+1)]["rpm"] >= 3800 for _, e in soft_trim_events):
                verdict = (
                    "INTAKE / BRAKE BOOSTER VACUUM LEAK CONFIRMED (POST-WARMUP)!\n"
                    "      -> Explanation: While cold (< 140 deg F), the vacuum leak was masked by Open-Loop\n"
                    "         enrichment (VF = 0.0V). Once coolant crossed 140 deg F into Closed-Loop, unmetered\n"
                    "         air under high manifold vacuum (< 3800 RPM) pinned OX lean and drove VF to +20% Rich!"
                )
            else:
                verdict = "FUEL DELIVERY STARVATION (Weak Fuel Pump / Clogged Injector — Lean across all loads)"

            log_p(f"\n  * AUTOMATED ROOT-CAUSE VERDICT:\n      {verdict}")

    log_p("=" * 82)

    # Save Complete Archive
    with open(archive_file, mode="w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow(["--- OFFLINE FLIGHT LOG DISSECTOR REPORT (LESSON 11) ---"])
        for line in report_lines:
            writer.writerow([line])
        writer.writerow([])
        writer.writerow(["--- FULL CONTINUOUS 20Hz SPI SD TELEMETRY LOG (13 CHANNELS) ---"])
        writer.writerow(header)
        for r in rows:
            writer.writerow([
                f"{r['host_ts']:.3f}", r["sim_ms"], r["rpm"], r["tps"], r["raw_thw"],
                r["coolant_f"], r["tvis"], r["fan"], r["live_dtc"], r["latched_dtc"],
                r["ox_mv"], r["vf_mv"], r["fuel_trim_pct"], r["loop_cl"]
            ])

    print(f"\n[+] Repository Archive Saved : {archive_file}")

    if WIPE_SD_AFTER_ARCHIVE:
        with open(SD_CARD_FILENAME, mode="w", newline="") as f:
            pass
        print(f"[+] SD Card Wiped & Reset    : '{SD_CARD_FILENAME}' is now 0 KB and ready for next test run!")


if __name__ == "__main__":
    main()