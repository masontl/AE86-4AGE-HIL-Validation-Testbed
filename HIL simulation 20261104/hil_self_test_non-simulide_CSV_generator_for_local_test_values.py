# ==============================================================================
# LESSON 13 COMPANION: STANDALONE CI/CD SYNTHETIC MISSION GENERATOR
# File: hil_self_test_generator.py
# Generates a 600-frame (30.0s @ 20Hz) 14-channel 'VIRTUAL_SD_512MB.CSV'
# exercising all 7 HIL requirements so anyone cloning the repo can run
# 'python sd_post_processor.py' immediately!
# ==============================================================================

import csv

OUT_CSV_FILENAME = "VIRTUAL_SD_512MB.CSV"


def main():
    with open(OUT_CSV_FILENAME, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Host_Timestamp", "SimTime_ms", "RPM", "TPS_Pct", "Raw_THW",
            "Coolant_F", "TVIS", "Fan", "Live_DTC", "Latched_DTC",
            "OX_mV", "VF_mV", "Fuel_Trim_Pct", "Loop_CL", "IGf_OK"
        ])

        coolant_f = 70.0
        rpm = 2200
        rpm_dir = 80
        latched_dtc = 0

        for frame in range(600):
            sim_ms = (frame + 1) * 50
            host_ts = f"{sim_ms / 1000.0:.3f}"

            # 1. Thermal warmup (70F -> 175F by frame 150)
            if coolant_f < 175.0:
                coolant_f += 0.75

            # 2. RPM sweep between 1200 and 6800 RPM
            rpm += rpm_dir
            if rpm >= 6800:
                rpm = 6800
                rpm_dir = -160
            elif rpm <= 1300:
                rpm = 1300
                rpm_dir = 110

            tps_pct = int(20 + (rpm - 1200) * 75 / 5600)
            tvis = 1 if (rpm >= 4400 and tps_pct >= 20) else 0
            raw_thw = int(923 - (coolant_f - 60) * (923 - 140) / (230 - 60))

            # Default healthy states
            live_dtc = 0
            igf_ok = 1
            fan = 1 if raw_thw < 300 else 0
            loop_cl = 1 if coolant_f >= 140.0 else 0
            ox_mv = 820 if (frame % 10 < 5) else 180
            vf_mv = 2500 if loop_cl == 1 else 0
            fuel_trim_pct = 0

            # Window A (Frames 200-260): Inject 'V' Brake Booster Vacuum Leak at Low Load
            if 200 <= frame <= 260:
                rpm = 2400
                tps_pct = 28
                tvis = 0
                loop_cl = 1
                ox_mv = 100
                vf_mv = 4850
                fuel_trim_pct = 19

            # Window B (Frames 320-370): Inject 'I' Code 14 Missing IGf Spark Confirmation
            elif 320 <= frame <= 370:
                igf_ok = 0
                live_dtc = 14
                latched_dtc = 14
                loop_cl = 0
                vf_mv = 0
                ox_mv = 20
                fuel_trim_pct = 0

            # Window C (Frames 430-470): Inject 'F' Code 4 THW Sensor Short-to-Ground
            elif 430 <= frame <= 470:
                raw_thw = 0
                live_dtc = 4
                latched_dtc = 4
                fan = 1
                loop_cl = 0
                vf_mv = 0
                fuel_trim_pct = 0

            if latched_dtc == 4:
                fan = 1

            logged_coolant = -99 if live_dtc == 4 else int(coolant_f)

            writer.writerow([
                host_ts, sim_ms, rpm, tps_pct, raw_thw,
                logged_coolant, tvis, fan, live_dtc, latched_dtc,
                ox_mv, vf_mv, fuel_trim_pct, loop_cl, igf_ok
            ])

    print(f"[+] Generated 600-frame synthetic HIL verification log: '{OUT_CSV_FILENAME}'")
    print("    -> Run 'python SD_post_processor.py' to test the 7-Requirement Verification Matrix!")


if __name__ == "__main__":
    main()