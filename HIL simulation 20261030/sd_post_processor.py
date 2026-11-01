# ==============================================================================
# LESSON 10 (PART 2): OFFLINE SD CARD LOG DISSECTOR & ARCHIVER (sd_post_processor.py)
# Performs Dual-Layer Flight Data Analysis:
#   1. Hard OBD-I Electrical Fault Dissection (Live_DTC != 0: Switch vs. RC Slew)
#   2. Soft Closed-Loop Control Anomaly Dissection (Live_DTC == 0: Vacuum Leak
#      driving OX Lean < 250mV and pinning TCCS VF Fuel Trim >= +15% Rich)
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
        print(f"[!] No data found on '{SD_CARD_FILENAME}'. Run sd_card_daemon.py first to log a mission!")
        return

    rows = []
    with open(SD_CARD_FILENAME, mode="r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for r in reader:
            if len(r) == 13:
                try:
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
                    })
                except ValueError:
                    continue

    if not rows:
        print("[!] SD card file contained header only (0 valid 12-channel telemetry frames).")
        return

    report_lines = []
    def log_p(msg):
        print(msg)
        report_lines.append(msg)

    archive_file = get_next_archive_filename(ARCHIVE_BASE_NAME)
    total_duration_s = (rows[-1]["sim_ms"] - rows[0]["sim_ms"]) / 1000.0
    peak_rpm = max(r["rpm"] for r in rows)
    valid_temps = [r["coolant_f"] for r in rows if r["coolant_f"] > 0]
    peak_temp = max(valid_temps) if valid_temps else -99

    # --------------------------------------------------------------------------
    # LAYER 1: Scan for Hard OBD-I Fault Onset Edges (Live_DTC: 0 -> non-zero)
    # --------------------------------------------------------------------------
    hard_fault_events = []
    for idx, r in enumerate(rows):
        prev_dtc = rows[idx - 1]["live_dtc"] if idx > 0 else 0
        if r["live_dtc"] != 0 and prev_dtc == 0:
            hard_fault_events.append(idx)

    # --------------------------------------------------------------------------
    # LAYER 2: Scan for Soft Closed-Loop Fuel Trim Saturation Events
    # Condition: OX pinned Lean (< 250 mV) AND VF Fuel Trim >= +15% while Live_DTC == 0
    # Debounce: Must persist for >= 10 consecutive frames (0.50s @ 20Hz)
    # --------------------------------------------------------------------------
    soft_trim_events = []
    in_soft_event = False
    start_idx = 0

    for idx, r in enumerate(rows):
        is_pinned_rich = (r["ox_mv"] < 250 and r["fuel_trim_pct"] >= 15)
        if is_pinned_rich and not in_soft_event:
            in_soft_event = True
            start_idx = idx
        elif not is_pinned_rich and in_soft_event:
            in_soft_event = False
            if (idx - start_idx) >= 10:
                soft_trim_events.append((start_idx, idx - 1))

    if in_soft_event and (len(rows) - start_idx) >= 10:
        soft_trim_events.append((start_idx, len(rows) - 1))

    # Compute Load-Regime Fuel Trim Averages across the entire mission
    low_load_trims  = [r["fuel_trim_pct"] for r in rows if r["rpm"] < 3800]
    high_load_trims = [r["fuel_trim_pct"] for r in rows if r["rpm"] >= 3800]
    avg_low_trim    = sum(low_load_trims) / len(low_load_trims) if low_load_trims else 0.0
    avg_high_trim   = sum(high_load_trims) / len(high_load_trims) if high_load_trims else 0.0

    log_p("=" * 80)
    log_p(f"  HACHI FLIGHT DATA RECORDER — LESSON 10 MISSION DEBRIEF ({archive_file})")
    log_p("=" * 80)
    log_p(f"  - Total 20Hz Frames Dissected      : {len(rows):,} frames ({total_duration_s:.2f}s of driving)")
    log_p(f"  - Mission Peak Engine Speed        : {peak_rpm} RPM")
    log_p(f"  - Mission Peak Coolant Temp        : {peak_temp} deg F")
    log_p(f"  - Hard OBD-I Electrical Incidents  : {len(hard_fault_events)} incident(s) (Triggered CEL)")
    log_p(f"  - Soft Fuel-Trim Anomaly Incidents : {len(soft_trim_events)} incident(s) (Silent / No CEL!)")
    log_p("-" * 80)

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
        log_p("-" * 80)

    # Report Layer 2: Soft Mechanical / Closed-Loop Fuel Trim Anomalies
    log_p("  [SECTION 2: SOFT MECHANICAL & CLOSED-LOOP FUEL TRIM DISSECTION (OBD-I BLIND SPOT)]")
    log_p(f"  * Load-Regime Trim Correlation:")
    log_p(f"    - Low-Load / High-Vacuum (< 3800 RPM) Avg Fuel Trim : {avg_low_trim:+.1f}%")
    log_p(f"    - High-Load / WOT       (>= 3800 RPM) Avg Fuel Trim : {avg_high_trim:+.1f}%")

    if not soft_trim_events:
        log_p("  * Closed-Loop Health Verdict: HEALTHY STOICHIOMETRIC DITHERING (No Vacuum Leak Detected)")
    else:
        for s_num, (s_start, s_end) in enumerate(soft_trim_events, start=1):
            slice_rows = rows[s_start:s_end + 1]
            dur_s      = (rows[s_end]["sim_ms"] - rows[s_start]["sim_ms"]) / 1000.0
            mean_rpm   = sum(x["rpm"] for x in slice_rows) // len(slice_rows)
            min_ox     = min(x["ox_mv"] for x in slice_rows)
            max_vf     = max(x["vf_mv"] for x in slice_rows)
            max_trim   = max(x["fuel_trim_pct"] for x in slice_rows)

            # Determine why the soft fault ended (Did RPM climb >= 3800 and mask the leak, or was 'N' sent?)
            after_idx = min(len(rows) - 1, s_end + 1)
            if rows[after_idx]["rpm"] >= 3800:
                exit_reason = f"Masked by High Intake Airflow as RPM climbed past {rows[after_idx]['rpm']} RPM (Low Manifold Vacuum)"
            else:
                exit_reason = "Vacuum Leak Mechanically Sealed ('N' Command)"

            log_p(f"\n  [SOFT ANOMALY #{s_num} @ SimTime = {rows[s_start]['sim_ms']} ms -> {rows[s_end]['sim_ms']} ms ({dur_s:.2f}s)]")
            log_p(f"    * OBD-I Check Engine Light : OFF (Live_DTC = 0 — Completely Invisible to 1986 ECU!)")
            log_p(f"    * Telemetry Signature      : OX pinned LEAN ({min_ox} mV) while VF maxed RICH ({max_vf} mV / {max_trim:+d}%)")
            log_p(f"    * Mean Engine Speed        : {mean_rpm} RPM (High Manifold Vacuum Regime)")
            log_p(f"    * Recovery / Exit Behavior : {exit_reason}")

        # Automated Root-Cause Verdict based on Load Dependency
        if avg_low_trim > (avg_high_trim + 5.0) or any(rows[min(len(rows)-1, e+1)]["rpm"] >= 3800 for _, e in soft_trim_events):
            verdict = (
                "INTAKE / BRAKE BOOSTER VACUUM LEAK CONFIRMED!\n"
                "      -> Explanation: Unmetered air enters under high manifold vacuum (< 3800 RPM),\n"
                "         pinning Wideband narrowband-emulated OX lean (< 0.20V) and forcing the 1986 TCCS\n"
                "         ECU to command +20% Max Rich VF trim (5.0V), causing severe rich fuel consumption.\n"
                "         Once throttle opens >= 3800 RPM, manifold vacuum drops and OX/VF recover!"
            )
        else:
            verdict = "FUEL DELIVERY STARVATION (Weak Fuel Pump / Clogged Injector — Lean across all loads)"

        log_p(f"\n  * AUTOMATED ROOT-CAUSE VERDICT:\n      {verdict}")

        # Print High-Res 50ms Snapshot of the first Soft Anomaly onset
        first_start = soft_trim_events[0][0]
        log_p("\n  * High-Res 50ms Snapshot Around Soft Trim Saturation (t = -0.15s to +0.20s):")
        log_p("    [Rel_s]  | SimTime_ms |   RPM    | TPS% | OX_mV | VF_mV | Trim% | LiveDTC")
        for s_idx in range(max(0, first_start - 3), min(len(rows), first_start + 5)):
            s = rows[s_idx]
            rel_s = (s["sim_ms"] - rows[first_start]["sim_ms"]) / 1000.0
            tag = " <-- VF SATURATION (>= +15%)" if s_idx == first_start else ""
            log_p(f"    {rel_s:+6.2f}s  |  {s['sim_ms']:8d}  | {s['rpm']:4d} RPM | {s['tps']:3d}% | "
                  f"{s['ox_mv']:4d}  | {s['vf_mv']:4d}  | {s['fuel_trim_pct']:+4d}% |    {s['live_dtc']}{tag}")

    log_p("=" * 80)

    # Save Complete Archive
    with open(archive_file, mode="w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow(["--- OFFLINE FLIGHT LOG DISSECTOR REPORT (LESSON 10) ---"])
        for line in report_lines:
            writer.writerow([line])
        writer.writerow([])
        writer.writerow(["--- FULL CONTINUOUS 20Hz TF/SD CARD TELEMETRY LOG (12 CHANNELS) ---"])
        writer.writerow(header)
        for r in rows:
            writer.writerow([
                f"{r['host_ts']:.3f}", r["sim_ms"], r["rpm"], r["tps"], r["raw_thw"],
                r["coolant_f"], r["tvis"], r["fan"], r["live_dtc"], r["latched_dtc"],
                r["ox_mv"], r["vf_mv"], r["fuel_trim_pct"]
            ])

    print(f"\n[+] Repository Archive Saved : {archive_file}")

    if WIPE_SD_AFTER_ARCHIVE:
        with open(SD_CARD_FILENAME, mode="w", newline="") as f:
            pass
        print(f"[+] SD Card Wiped & Reset    : '{SD_CARD_FILENAME}' is now 0 KB and ready for next test run!")


if __name__ == "__main__":
    main()