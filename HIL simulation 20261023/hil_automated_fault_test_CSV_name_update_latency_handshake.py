import serial
import time
import csv
import os
import re

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
BASE_NAME = "hachi_latency_benchmark"


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
log_print(f"[HIL BENCHMARK] Starting Synchronized Fault Injection & Latency Test...")
log_print(f"[HIL BENCHMARK] Auto-Selected Output File: {csv_filename}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(0.2)

    # 1. Pre-Test Sync: Heal sensor ('N'), reset latch ('R'), and drain ALL old com0com backlog!
    ser.write(b'NNN\n')
    ser.write(b'RRR\n')
    time.sleep(0.3)
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)
        time.sleep(0.05)
    ser.readline()  # Discard partial line so Frame 0 is 100% live and synchronized!

    fault_injected = False
    fault_inject_wall_time = 0.0
    fault_inject_sim_ms = 0
    wall_latency_ms = None
    sim_latency_ms = None
    min_thw_during_fault = 1023
    recovery_verified = False
    fault_cleared_sent = False

    with open(csv_filename, "w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(["SimTime_ms", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC", "Test_Phase"])

        for frame in range(120):
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

            # Step 2: At Frame 25, record live timestamp and inject fault ('F')
            if frame == 25 and not fault_injected:
                ser.write(b'FFF\n')
                fault_injected = True
                fault_inject_wall_time = time.perf_counter()
                fault_inject_sim_ms = t_ms
                log_print(f"[Frame {frame}] >>> Sent 'FFF' Command: Injected THW Short-to-Ground Fault (SimTime={t_ms}ms)!")

            # Closed-Loop Command Retry: If MCU #1 missed 'F' and THW is still >400, re-send 'F'!
            if fault_injected and not fault_cleared_sent and frame < 70 and thw > 400:
                ser.write(b'F\n')

            if fault_injected and not fault_cleared_sent and frame < 75:
                if thw < min_thw_during_fault:
                    min_thw_during_fault = thw

            if fault_injected and wall_latency_ms is None:
                phase = "2_FAULT_DECAYING"
                if dtc == 4 and fan == 1:
                    wall_latency_ms = (time.perf_counter() - fault_inject_wall_time) * 1000.0
                    sim_latency_ms = t_ms - fault_inject_sim_ms
                    log_print(f"[Frame {frame}] >>> ECU Caught Fault! (Raw THW={thw}, Fan=ON, DTC=4)")
                    log_print(f"            >>> Wall-Clock Latency: {wall_latency_ms:.1f} ms | Simulated MCU Latency: {sim_latency_ms} ms")
            elif fault_injected and wall_latency_ms is not None and not fault_cleared_sent:
                phase = "2_FAULT_LATCHED"

            # Step 4: At Frame 75, heal the wire ('NNN')
            if frame == 75:
                ser.write(b'NNN\n')
                log_print(f"[Frame {frame}] >>> Sent 'NNN' Command: Restored THW Sensor Wire (Recharging RC Cap)...")

            # Step 5: At Frame 90, reset the ECU fault latch ('RRR')
            if frame == 90 and not fault_cleared_sent:
                ser.write(b'RRR\n')
                fault_cleared_sent = True
                log_print(f"[Frame {frame}] >>> Sent 'RRR' Command: Resetting ECU Fault Latch...")

            if fault_cleared_sent:
                phase = "3_POST_RECOVERY"
                if dtc == 0 and thw > 200:
                    recovery_verified = True

            writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc, phase])

        log_print("")
        log_print("========================================================")
        log_print(f"   AUTOMATED HIL SCORECARD ({csv_filename})")
        log_print("========================================================")
        log_print(f" 1. Fault Injection & Fail-Safe Triggered : {'PASSED [OK]' if wall_latency_ms else 'FAILED'}")
        if wall_latency_ms is not None:
            log_print(f" 2. Wall-Clock Reaction Latency           : {wall_latency_ms:.1f} ms")
            log_print(f" 3. Simulated MCU Reaction Latency        : {sim_latency_ms} ms")
        else:
            log_print(f" 2. Diagnostic Min Raw_THW Reached        : {min_thw_during_fault} (Threshold is < 41)")
        log_print(f" 4. Self-Healing & Latch Reset Verified   : {'PASSED [OK]' if recovery_verified else 'FAILED'}")
        log_print("========================================================")

        writer.writerow([])
        writer.writerow(["--- PYTHON TERMINAL EXECUTION LOG & SCORECARD ---"])
        for log_line in terminal_log:
            writer.writerow([log_line])

    ser.close()
    print(f"\n[COMPLETE] Telemetry + Terminal Log saved to: {csv_filename}")

except Exception as e:
    print(f"[ERROR] Serial connection issue: {e}")