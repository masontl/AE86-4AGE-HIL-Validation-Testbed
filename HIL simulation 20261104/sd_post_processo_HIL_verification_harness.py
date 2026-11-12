# ==============================================================================
# LESSON 13: AUTOMATED PYTHON HIL VERIFICATION HARNESS & 4-LAYER DISSECTOR
# File: sd_post_processor.py
#
# Performs:
#   Part A — 4-Layer Flight Data Dissection (Hard Faults, Warmup/AAV, CL Fuel Trim, IGf Code 14)
#   Part B — Automated Requirements Traceability Matrix (REQ-HIL-01 to REQ-HIL-07)
#            Evaluates deterministic PASS / FAIL / N/A criteria and exits with
#            CI/CD status code (0 = ALL PASS, 1 = REGRESSION FAILURE).
# ==============================================================================

import csv
import os
import re
import sys

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


def evaluate_hil_requirements(rows):
    """
    Evaluates the 7 Formal HIL System Requirements (REQ-HIL-01 through REQ-HIL-07)
    against the extracted 14-channel 20Hz SPI SD telemetry frames.
    Returns a list of dicts: {id, name, status, metric, criteria}
    """
    results = []

    # --------------------------------------------------------------------------
    # REQ-HIL-01: 20Hz Deterministic Frame Timing & Monotonic Clock Integrity
    # --------------------------------------------------------------------------
    if len(rows) >= 2:
        deltas = [rows[i]["sim_ms"] - rows[i - 1]["sim_ms"] for i in range(1, len(rows))]
        mean_dt = sum(deltas) / len(deltas)
        non_monotonic = sum(1 for d in deltas if d <= 0)
        req1_pass = (48.0 <= mean_dt <= 52.0) and (non_monotonic == 0)
        results.append({
            "id": "REQ-HIL-01",
            "name": "20Hz Deterministic Scheduler",
            "status": "PASS" if req1_pass else "FAIL",
            "metric": f"Mean dt={mean_dt:.2f}ms, Drops={non_monotonic}",
            "criteria": "48.0ms <= Mean dt <= 52.0ms & 0 clock drops"
        })
    else:
        results.append({
            "id": "REQ-HIL-01",
            "name": "20Hz Deterministic Scheduler",
            "status": "FAIL",
            "metric": f"Frames={len(rows)}",
            "criteria": ">= 2 frames required"
        })

    # --------------------------------------------------------------------------
    # REQ-HIL-02: Bi-Metal AAV Cold Idle-Up Floor (>= 1100 RPM when < 140°F)
    # --------------------------------------------------------------------------
    cold_ol_frames = [r for r in rows if 0 < r["coolant_f"] < 140 and r["live_dtc"] == 0]
    if cold_ol_frames:
        min_cold_rpm = min(r["rpm"] for r in cold_ol_frames)
        req2_pass = (min_cold_rpm >= 1100)
        results.append({
            "id": "REQ-HIL-02",
            "name": "Bi-Metal AAV Cold Idle-Up",
            "status": "PASS" if req2_pass else "FAIL",
            "metric": f"Min Cold RPM={min_cold_rpm} RPM ({len(cold_ol_frames)} frames)",
            "criteria": "RPM >= 1100 RPM across all <140F frames"
        })
    else:
        results.append({
            "id": "REQ-HIL-02",
            "name": "Bi-Metal AAV Cold Idle-Up",
            "status": "PASS (N/A)",
            "metric": "0 Cold (<140F) frames in log",
            "criteria": "Started warm or fault-gated"
        })

    # --------------------------------------------------------------------------
    # REQ-HIL-03: T-VIS Secondary Intake Runner Hysteresis (4400 ON / 4250 OFF)
    # Note: Allows 1-frame ADC/RPM discretization tolerance at boundary
    # --------------------------------------------------------------------------
    tvis_checked = 0
    tvis_compliant = 0
    for r in rows:
        # Unambiguous High-RPM Open Zone (RPM >= 4550 and TPS >= 22%)
        if r["rpm"] >= 4550 and r["tps"] >= 22:
            tvis_checked += 1
            if r["tvis"] == 1:
                tvis_compliant += 1
        # Unambiguous Low-RPM Shut Zone (RPM <= 4150)
        elif r["rpm"] <= 4150:
            tvis_checked += 1
            if r["tvis"] == 0:
                tvis_compliant += 1

    if tvis_checked > 0:
        tvis_acc = (tvis_compliant / tvis_checked) * 100.0
        req3_pass = (tvis_acc >= 98.0)
        results.append({
            "id": "REQ-HIL-03",
            "name": "T-VIS Runner Hysteresis Gate",
            "status": "PASS" if req3_pass else "FAIL",
            "metric": f"{tvis_acc:.1f}% compliant ({tvis_compliant}/{tvis_checked})",
            "criteria": ">= 98.0% state compliance outside hysteresis band"
        })
    else:
        results.append({
            "id": "REQ-HIL-03",
            "name": "T-VIS Runner Hysteresis Gate",
            "status": "PASS (N/A)",
            "metric": "0 unambiguous zone frames",
            "criteria": "Sweep RPM across 4400 RPM threshold"
        })

    # --------------------------------------------------------------------------
    # REQ-HIL-04: Open-Loop Warmup & Fault Gate Isolation (Zero False Trim)
    # Verifies that whenever Loop_CL == 0, Fuel_Trim_Pct is strictly 0% (never -20%)
    # and whenever Coolant < 140F or Live_DTC != 0, Loop_CL is strictly 0.
    # --------------------------------------------------------------------------
    ol_violations = 0
    for r in rows:
        if r["loop_cl"] == 0 and r["fuel_trim_pct"] != 0:
            ol_violations += 1
        if (0 < r["coolant_f"] < 140 or r["live_dtc"] != 0) and r["loop_cl"] != 0:
            ol_violations += 1

    req4_pass = (ol_violations == 0)
    results.append({
        "id": "REQ-HIL-04",
        "name": "140F Open/Closed-Loop Gate",
        "status": "PASS" if req4_pass else "FAIL",
        "metric": f"{ol_violations} gate/trim violations in {len(rows)} frames",
        "criteria": "Loop_CL==0 & Trim==0% when <140F or DTC!=0"
    })

    # --------------------------------------------------------------------------
    # REQ-HIL-05: Radiator Fan Thermal Control (>=195F) & Code 4 Failsafe Interlock
    # --------------------------------------------------------------------------
    hot_or_code4 = [r for r in rows if r["raw_thw"] < 300 or r["live_dtc"] == 4 or r["latched_dtc"] == 4]
    if hot_or_code4:
        fan_on_count = sum(1 for r in hot_or_code4 if r["fan"] == 1)
        fan_pct = (fan_on_count / len(hot_or_code4)) * 100.0
        req5_pass = (fan_pct == 100.0)
        results.append({
            "id": "REQ-HIL-05",
            "name": "Thermal & Code 4 Fan Interlock",
            "status": "PASS" if req5_pass else "FAIL",
            "metric": f"Fan ON {fan_pct:.1f}% ({fan_on_count}/{len(hot_or_code4)} frames)",
            "criteria": "100% Fan ON when >=195F or Code 4 active"
        })
    else:
        # Check that fan stayed OFF when cool (< 185F / Raw_THW > 345) and no fault
        cool_frames = [r for r in rows if r["raw_thw"] > 345 and r["latched_dtc"] == 0]
        fan_off_ok = all(r["fan"] == 0 for r in cool_frames)
        results.append({
            "id": "REQ-HIL-05",
            "name": "Thermal & Code 4 Fan Interlock",
            "status": "PASS" if fan_off_ok else "FAIL",
            "metric": f"Cool regime verified ({len(cool_frames)} frames Fan=OFF)",
            "criteria": "Fan OFF <185F (Inject 'F' or >195F to test ON)"
        })

    # --------------------------------------------------------------------------
    # REQ-HIL-06: Closed-Loop Fuel-Trim Integrator & Vacuum Leak Detection
    # --------------------------------------------------------------------------
    cl_frames = [r for r in rows if r["loop_cl"] == 1 and r["live_dtc"] == 0]
    lean_low_load = [r for r in cl_frames if r["rpm"] < 3800 and r["ox_mv"] < 250]
    if len(lean_low_load) >= 10:
        peak_vf = max(r["vf_mv"] for r in lean_low_load)
        peak_trim = max(r["fuel_trim_pct"] for r in lean_low_load)
        req6_pass = (peak_vf >= 4000 and peak_trim >= 12)
        results.append({
            "id": "REQ-HIL-06",
            "name": "CL Fuel-Trim Vacuum Leak Catch",
            "status": "PASS" if req6_pass else "FAIL",
            "metric": f"Peak VF={peak_vf}mV ({peak_trim:+d}% Trim)",
            "criteria": "VF slews >= 4000mV (+12% to +20%) on low-load lean OX"
        })
    elif cl_frames:
        mean_trim = sum(r["fuel_trim_pct"] for r in cl_frames) / len(cl_frames)
        req6_pass = (-8.0 <= mean_trim <= 8.0)
        results.append({
            "id": "REQ-HIL-06",
            "name": "CL Stoichiometric Trim Balance",
            "status": "PASS" if req6_pass else "FAIL",
            "metric": f"Mean CL Trim={mean_trim:+.1f}% ({len(cl_frames)} frames)",
            "criteria": "Mean CL Trim within +/-8.0% when no leak active"
        })
    else:
        results.append({
            "id": "REQ-HIL-06",
            "name": "CL Fuel-Trim Integrator Audit",
            "status": "PASS (N/A)",
            "metric": "0 Closed-Loop frames recorded",
            "criteria": "Reach >=140F to evaluate Closed-Loop trim"
        })

    # --------------------------------------------------------------------------
    # REQ-HIL-07: Dual-ISR VAST Igniter IGf Watchdog & Code 14 Fuel-Cut Failsafe
    # --------------------------------------------------------------------------
    igf_fault_frames = [i for i, r in enumerate(rows) if r["igf_ok"] == 0 or r["live_dtc"] == 14]
    if igf_fault_frames:
        # Check that every IGf fault frame latched Code 14 and dropped Loop_CL to 0
        code14_latched = all(rows[i]["live_dtc"] == 14 and rows[i]["loop_cl"] == 0 for i in igf_fault_frames)
        # Check settled frames (at least 3 frames after fault onset) for VF < 350mV and OX < 150mV
        settled_indices = [
            i for i in igf_fault_frames
            if i >= 3 and (rows[i - 3]["igf_ok"] == 0 or rows[i - 3]["live_dtc"] == 14)
        ]
        if settled_indices:
            max_vf_cut = max(rows[i]["vf_mv"] for i in settled_indices)
            min_ox_cut = min(rows[i]["ox_mv"] for i in settled_indices)
            settled_ok = (max_vf_cut < 350 and min_ox_cut < 150)
            metric_str = f"DTC=14, Loop_CL=0, Settled VF={max_vf_cut}mV, OX={min_ox_cut}mV"
        else:
            settled_ok = True
            metric_str = f"DTC=14, Loop_CL=0 ({len(igf_fault_frames)} transient frames)"

        req7_pass = (code14_latched and settled_ok)
        results.append({
            "id": "REQ-HIL-07",
            "name": "Code 14 IGf Fuel-Cut Failsafe",
            "status": "PASS" if req7_pass else "FAIL",
            "metric": metric_str,
            "criteria": "IGf_OK==0 triggers DTC 14, Loop_CL=0, VF<350mV, OX<150mV"
        })
    else:
        # Verify 100% healthy IGf confirmation when fault 'I' was not injected
        all_igf_ok = all(r["igf_ok"] == 1 and r["live_dtc"] != 14 for r in rows)
        results.append({
            "id": "REQ-HIL-07",
            "name": "Dual-ISR IGf Handshake Watchdog",
            "status": "PASS" if all_igf_ok else "FAIL",
            "metric": f"100% IGf_OK across {len(rows)} frames (No 'I' Fault Injected)",
            "criteria": "Zero false Code 14 triggers during normal spark operation"
        })

    return results


def main():
    if not os.path.exists(SD_CARD_FILENAME) or os.path.getsize(SD_CARD_FILENAME) == 0:
        print(f"[!] No data found on '{SD_CARD_FILENAME}'. Run SDimg_create_CSV.py first!")
        sys.exit(1)

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
        sys.exit(1)

    report_lines = []
    def log_p(msg):
        print(msg)
        report_lines.append(msg)

    archive_file = get_next_archive_filename(ARCHIVE_BASE_NAME)
    total_duration_s = (rows[-1]["sim_ms"] - rows[0]["sim_ms"]) / 1000.0
    valid_temps = [r["coolant_f"] for r in rows if r["coolant_f"] > 0]
    start_temp = valid_temps[0] if valid_temps else -99
    peak_temp  = max(valid_temps) if valid_temps else -99

    # Layer 1-4 Event Extraction
    hard_sensor_events = [
        idx for idx, r in enumerate(rows)
        if r["live_dtc"] in (4, 7) and (rows[idx - 1]["live_dtc"] if idx > 0 else 0) != r["live_dtc"]
    ]
    ol_frames = [r for r in rows if r["loop_cl"] == 0 and r["live_dtc"] == 0]
    cl_frames = [r for r in rows if r["loop_cl"] == 1 and r["live_dtc"] == 0]
    ol_pct = (len(ol_frames) / len(rows)) * 100.0
    cl_pct = (len(cl_frames) / len(rows)) * 100.0

    cl_entry_idx = next(
        (i for i in range(1, len(rows)) if rows[i]["loop_cl"] == 1 and rows[i - 1]["loop_cl"] == 0),
        0 if rows[0]["loop_cl"] == 1 else None
    )

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

    log_p("=" * 88)
    log_p(f"  HACHI HIL FLIGHT DATA RECORDER — LESSON 13 VERIFICATION & DEBRIEF ({archive_file})")
    log_p("=" * 88)
    log_p(f"  - Total 20Hz Frames Dissected      : {len(rows):,} frames ({total_duration_s:.2f}s of driving)")
    log_p(f"  - Thermal Envelope (Start -> Peak) : {start_temp} deg F -> {peak_temp} deg F")
    log_p(f"  - TCCS Loop Split (Open / Closed)  : {ol_pct:.1f}% Open-Loop | {cl_pct:.1f}% Closed-Loop")
    log_p(f"  - Fault Incidents Captured         : {len(hard_sensor_events)} Sensor | {len(igf_fault_events)} Igniter (Code 14) | {len(soft_trim_events)} Vacuum Leak")
    log_p("-" * 88)

    # --------------------------------------------------------------------------
    # EXECUTE AUTOMATED HIL REQUIREMENTS TRACEABILITY MATRIX (REQ-HIL-01 .. 07)
    # --------------------------------------------------------------------------
    req_results = evaluate_hil_requirements(rows)
    failed_reqs = [r for r in req_results if r["status"] == "FAIL"]

    log_p("  [AUTOMATED HIL REQUIREMENTS VERIFICATION MATRIX (LESSON 13)]")
    log_p(f"  {'REQ ID':<11} | {'SUBSYSTEM UNDER TEST':<30} | {'STATUS':<10} | {'MEASURED TELEMETRY METRIC'}")
    log_p("  " + "-" * 84)
    for req in req_results:
        log_p(f"  {req['id']:<11} | {req['name']:<30} | [{req['status']:^8}] | {req['metric']}")
    log_p("  " + "-" * 84)

    overall_verdict = "PASS (100% REQUIREMENTS COMPLIANT)" if not failed_reqs else f"FAIL ({len(failed_reqs)} REQUIREMENT REGRESSION(S))"
    log_p(f"  * OVERALL HIL BENCH VERDICT : {overall_verdict}")
    log_p("=" * 88)

    # Save Complete 14-Channel Archive + Requirements Scorecard
    with open(archive_file, mode="w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow(["--- OFFLINE FLIGHT LOG DISSECTOR & HIL VERIFICATION REPORT (LESSON 13) ---"])
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

    # Exit with standard CI/CD return code (0 = All Pass, 1 = Failure)
    sys.exit(0 if not failed_reqs else 1)


if __name__ == "__main__":
    main()