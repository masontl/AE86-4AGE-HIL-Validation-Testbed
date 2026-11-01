import serial
import time
import csv

# Update to match your working com0com port and baud rate from yesterday
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

print("[HIL BENCHMARK] Starting Hands-Free Fault Injection & Latency Test...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(0.5)

    # Step 1: Ensure normal sensor state ('N') and clear any old ECU latches ('R')
    ser.write(b'NR\n')
    ser.reset_input_buffer()

    fault_injected = False
    fault_inject_time = 0.0
    detection_latency_ms = None
    recovery_verified = False
    fault_cleared_sent = False

    with open("hachi_latency_benchmark_04.csv", "w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(["SimTime_ms", "RPM", "TPS_Pct", "Raw_THW", "TVIS", "Fan", "DTC", "Test_Phase"])

        for frame in range(120):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not (line.startswith("$TELEM") and line.endswith("*")):
                continue

            parts = line[7:-1].split(",")
            if len(parts) != 7:
                continue

            # Written cleanly without tuple-comma unpacking to avoid Thonny plugin bug!
            vals = [int(x) for x in parts]
            t_ms = vals[0]
            rpm  = vals[1]
            tps  = vals[2]
            thw  = vals[3]
            tvis = vals[4]
            fan  = vals[5]
            dtc  = vals[6]

            phase = "1_BASELINE_NORMAL"

            # Step 2: At Frame 25, Python automatically injects the wire fault ('F')!
            if frame == 25 and not fault_injected:
                ser.write(b'F\n')
                fault_injected = True
                fault_inject_time = time.perf_counter()
                print(f"[Frame {frame}] >>> Sent 'F' Command: Injected THW Short-to-Ground Fault!")

            # Step 3: Measure exact millisecond latency until ECU latches DTC Code 4
            if fault_injected and detection_latency_ms is None:
                phase = "2_FAULT_DECAYING"
                if dtc == 4 and fan == 1:
                    detection_latency_ms = (time.perf_counter() - fault_inject_time) * 1000.0
                    print(f"[Frame {frame}] >>> ECU Caught Fault! (Raw THW={thw}, Fan=ON, DTC=4)")
                    print(f"            >>> Measured RC + ECU Reaction Latency: {detection_latency_ms:.1f} ms")

            # Step 4: At Frame 75, heal the wire ('N') and wait 10 frames for RC cap to recharge
            if frame == 75:
                ser.write(b'N\n')
                print(f"[Frame {frame}] >>> Sent 'N' Command: Restored THW Sensor Wire (Recharging RC Cap)...")

            # Step 5: At Frame 90, send 'R' to reset the ECU fault latch and verify recovery
            if frame == 90 and not fault_cleared_sent:
                ser.write(b'R\n')
                fault_cleared_sent = True
                print(f"[Frame {frame}] >>> Sent 'R' Command: Resetting ECU Fault Latch...")

            if fault_cleared_sent:
                phase = "3_POST_RECOVERY"
                if dtc == 0 and thw > 200:
                    recovery_verified = True

            writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc, phase])

    ser.close()

    print("\n========================================================")
    print("   AUTOMATED HIL FAULT-INJECTION SCORECARD")
    print("========================================================")
    print(f" 1. Fault Injection & Fail-Safe Triggered : {'PASSED [OK]' if detection_latency_ms else 'FAILED'}")
    if detection_latency_ms:
        print(f" 2. Total RC + ECU Reaction Time          : {detection_latency_ms:.1f} ms")
    print(f" 3. Self-Healing & Latch Reset Verified   : {'PASSED [OK]' if recovery_verified else 'FAILED'}")
    print("========================================================")
    print("Full time-series log saved to: hachi_latency_benchmark.csv")

except Exception as e:
    print(f"[ERROR] Serial connection issue: {e}")