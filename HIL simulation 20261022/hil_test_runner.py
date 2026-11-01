import serial
import time
import csv

# Set to your virtual COM port (e.g. 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

print("[HIL RUNNER] Initializing Automated Test Suite for 4A-GE Controller...")
test_results = {"TVIS_Crossover_Verified": False, "Fan_Thermal_Control_Verified": False, "FailSafe_Verified": False}

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(1) # Allow connection to settle
    
    with open("hachi_hil_execution_report.csv", "w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(["Timestamp_ms", "RPM", "Throttle_Pct", "Raw_THW", "TVIS_State", "Fan_State", "DTC_Code"])
        
        print("[HIL RUNNER] Logging live telemetry and evaluating assertions...")
        for _ in range(100):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("$TELEM") and line.endswith("*"):
                parts = line[7:-1].split(",")
                if len(parts) == 7:
                    t_ms, rpm, tps, thw, tvis, fan, dtc = map(int, parts)
                    writer.writerow([t_ms, rpm, tps, thw, tvis, fan, dtc])
                    
                    # Assertion 1: T-VIS opens above 4400 RPM under throttle
                    if rpm >= 4400 and tps >= 20 and tvis == 1:
                        test_results["TVIS_Crossover_Verified"] = True
                        
                    # Assertion 2: Radiator Fan kicks on when coolant gets hot
                    if thw < 460 and fan == 1:
                        test_results["Fan_Thermal_Control_Verified"] = True
                        
                    # Assertion 3: Emergency Fail-Safe engages if sensor faults
                    if dtc == 4 and fan == 1:
                        test_results["FailSafe_Verified"] = True

    ser.close()
    print("\n[HIL RUNNER] Test Run Completed. Automated Verification Summary:")
    for test, passed in test_results.items():
        status = "PASSED [OK]" if passed else "FAILED [CHECK THRESHOLDS]"
        print(f" - {test}: {status}")
    print("\nTelemetry saved to: hachi_hil_execution_report.csv")

except Exception as e:
    print(f"[NOTE] Live serial port not connected ({e}). Inside SimulIDE, verify via the Virtual Terminal screen!")