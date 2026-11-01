# ==============================================================================
# LESSON 9 (PART 2): OFFLINE SD CARD LOG DISSECTOR & ARCHIVER (sd_post_processor.py)
# Reads VIRTUAL_SD_512MB.CSV, dissects all fault incidents, archives the full
# mission report to the PC repository, and WIPES the SD card for the next drive!
# ==============================================================================

import csv
import os
import re

SD_CARD_FILENAME = "VIRTUAL_SD_512MB.CSV"
ARCHIVE_BASE_NAME = "hachi_mission_archive"
WIPE_SD_AFTER_ARCHIVE = True  # Standard UAV workflow: Wipe card after downloading to repository!


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
            if len(r) == 10:
                try:
                    rows.append({
                        "host_ts":     float(r[0]),
                        "sim_ms":      int(r[1]),
                        "rpm":         int(r[2]),
                        "tps":         int(r[3]),
                        "raw_thw":     int(r[4]),
                        "coolant_f":   int(r[5]),
                        "tvis":        int(r[6]),
                        "fan":         int(r[7]),
                        "live_dtc":    int(r[8]),
                        "latched_dtc": int(r[9]),
                    })
                except ValueError:
                    continue

    if not rows:
        print("[!] SD card file contained header only (0 telemetry frames).")
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

    # Scan continuous log for distinct Fault Onset Edges (Live_DTC transitions from 0 -> non-zero)
    fault_events = []
    for idx, r in enumerate(rows):
        prev_dtc = rows[idx - 1]["live_dtc"] if idx > 0 else 0
        if r["live_dtc"] != 0 and prev_dtc == 0:
            fault_events.append(idx)

    log_p("=" * 76)
    log_p(f"  HACHI FLIGHT DATA RECORDER — OFFLINE MISSION DEBRIEF ({archive_file})")
    log_p("=" * 76)
    log_p(f"  - Total 20Hz Frames Dissected : {len(rows):,} frames ({total_duration_s:.2f} seconds of driving)")
    log_p(f"  - Mission Peak Engine Speed   : {peak_rpm} RPM")
    log_p(f"  - Mission Peak Coolant Temp   : {peak_temp} deg F")
    log_p(f"  - Distinct Fault Incidents    : {len(fault_events)} incident(s) detected on SD Card")
    log_p("-" * 76)

    for ev_num, trig_idx in enumerate(fault_events, start=1):
        trig = rows[trig_idx]
        prev = rows[trig_idx - 1] if trig_idx > 0 else trig

        # Root-Cause Dissection at exact 50ms onset edge
        if trig["raw_thw"] < 41:
            if prev["raw_thw"] > 180:
                cause = f"INSTANT SHORT-TO-GROUND (RawTHW dropped {prev['raw_thw']} -> {trig['raw_thw']} in 50ms: Manual SPST Switch)"
            else:
                cause = f"RC-FILTERED SHORT-TO-GROUND (RawTHW slewed {prev['raw_thw']} -> {trig['raw_thw']} < 41 ADC: Sensor Wire / 'FFF')"
        elif trig["raw_thw"] > 982:
            cause = f"OPEN-CIRCUIT / 5V PULL-UP FAULT (RawTHW={trig['raw_thw']} > 982 ADC)"
        else:
            cause = f"TRANSIENT GLITCH (RawTHW={trig['raw_thw']})"

        # Slice 20s Pre-Trigger (400 frames @ 20Hz) and 10s Post-Trigger (200 frames @ 20Hz)
        pre_start = max(0, trig_idx - 400)
        post_end  = min(len(rows), trig_idx + 200)
        post_slice = rows[trig_idx:post_end]
        fan_failsafe_ok = all(x["fan"] == 1 for x in post_slice)

        # Find if/when the fault healed later in the mission
        heal_idx = next((i for i in range(trig_idx + 1, len(rows)) if rows[i]["live_dtc"] == 0), None)
        duration_str = f"{(rows[heal_idx]['sim_ms'] - trig['sim_ms']) / 1000.0:.2f}s (Self-Healed)" if heal_idx else "Remained Active Until End of Log"

        log_p(f"  [INCIDENT #{ev_num} DISSECTION @ SimTime = {trig['sim_ms']} ms]")
        log_p(f"    * Diagnostic Trouble Code : Code {trig['live_dtc']}")
        log_p(f"    * Engine State at Onset   : {trig['rpm']} RPM | TPS={trig['tps']}% | RawTHW={trig['raw_thw']} ADC")
        log_p(f"    * Root-Cause Signature    : {cause}")
        log_p(f"    * Fault Duration          : {duration_str}")
        log_p(f"    * Fan Failsafe Response   : {'VERIFIED 100% ON [OK]' if fan_failsafe_ok else 'FAILED'}")
        log_p(f"    * Extracted Window Slice  : {(trig['sim_ms'] - rows[pre_start]['sim_ms'])/1000.0:.1f}s Pre-Fault / "
              f"{(rows[post_end - 1]['sim_ms'] - trig['sim_ms'])/1000.0:.1f}s Post-Fault")
        log_p("    * High-Res 50ms Snapshot Around Fault Onset (t = -0.20s to +0.25s):")
        log_p("      [Rel_s]  | SimTime_ms |   RPM    | TPS% | RawTHW | TVIS | Fan | LiveDTC")
        for s_idx in range(max(0, trig_idx - 4), min(len(rows), trig_idx + 6)):
            s = rows[s_idx]
            rel_s = (s["sim_ms"] - trig["sim_ms"]) / 1000.0
            tag = " <-- FAULT ONSET" if s_idx == trig_idx else ""
            log_p(f"      {rel_s:+6.2f}s  |  {s['sim_ms']:8d}  | {s['rpm']:4d} RPM | {s['tps']:3d}% |  {s['raw_thw']:4d}  |  "
                  f" {s['tvis']}   |  {s['fan']}  |    {s['live_dtc']}{tag}")
        log_p("-" * 76)

    # Save Complete Archive (Mission Summary Scorecard + Full Raw 20Hz SD Telemetry)
    with open(archive_file, mode="w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow(["--- OFFLINE FLIGHT LOG DISSECTOR REPORT ---"])
        for line in report_lines:
            writer.writerow([line])
        writer.writerow([])
        writer.writerow(["--- FULL CONTINUOUS 20Hz TF/SD CARD TELEMETRY LOG ---"])
        writer.writerow(header)
        for r in rows:
            writer.writerow([
                f"{r['host_ts']:.3f}", r["sim_ms"], r["rpm"], r["tps"], r["raw_thw"],
                r["coolant_f"], r["tvis"], r["fan"], r["live_dtc"], r["latched_dtc"]
            ])

    print(f"\n[+] Repository Archive Saved : {archive_file}")

    if WIPE_SD_AFTER_ARCHIVE:
        with open(SD_CARD_FILENAME, mode="w", newline="") as f:
            pass  # Truncate file to 0 bytes (Wiped & formatted for next mission!)
        print(f"[+] SD Card Wiped & Reset    : '{SD_CARD_FILENAME}' is now 0 KB and ready for next test run!")


if __name__ == "__main__":
    main()