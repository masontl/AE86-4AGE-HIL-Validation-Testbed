// ====================================================================
// LESSON 10 — MCU #1: Virtual 1986 4A-GE Engine + Closed-Loop OX & VF Trim
// SimulIDE HIL Test Bench - Runs on MCU #1
// Commands on RX (115200 baud):
//   'F' = Inject THW Coolant Fault (Code 4: 0V Short-to-Ground on D11)
//   'T' = Inject VTA TPS Fault     (Code 7: 0V Short-to-Ground on D10)
//   'B' = Inject BOTH THW + VTA Hard Faults simultaneously
//   'V' = Inject Brake Booster Vacuum Leak (Soft Mechanical Fault: Low-Load Lean OX -> Max Rich VF)
//   'N' = Restore Normal Operation on all sensors & seal vacuum leak
// ====================================================================

#include <Arduino.h>

const int PIN_VF_OUT   = 5;   // Output: TCCS VF Fuel Trim Diagnostic Voltage (0-5V PWM -> MCU#2 A3)
const int PIN_OX_OUT   = 6;   // Output: Wideband's Emulated Narrowband OX (0-1V PWM -> MCU#2 A2)
const int PIN_FAN_IN   = 7;   // Input:  Closed-loop feedback from MCU #2 Radiator Fan (D9)
const int PIN_TVIS_IN  = 8;   // Input:  Feedback from MCU #2 T-VIS solenoid (D8)
const int PIN_IG_MINUS = 9;   // Output: Simulated IG- tachometer pulse to MCU #2 (D2)
const int PIN_VTA_OUT  = 10;  // Output: Simulated Throttle Position (PWM -> RC -> MCU#2 A0)
const int PIN_THW_OUT  = 11;  // Output: Simulated Coolant Temp (PWM -> RC -> MCU#2 A1)

unsigned long lastPulseMicros = 0;
unsigned long lastPhysicsMs   = 0;
int currentRpm   = 2000;
int rpmDirection = 50;
bool pulseState  = LOW;

float coolantTempF = 180.0;

// Closed-Loop Fuel Trim Integrator State (2.50V = 0% Base Trim, Range: 0.5V to 4.9V)
float vfVoltage  = 2.50;
bool  oxRichFlip = false;
int   oxToggleCounter = 0;

// Fault Injection Flags
bool thwFaultInjected = false; // 'F' or 'B' (Hard Electrical Fault)
bool vtaFaultInjected = false; // 'T' or 'B' (Hard Electrical Fault)
bool vacLeakInjected  = false; // 'V'        (Soft Mechanical Vacuum Leak)

void setup() {
  pinMode(PIN_VF_OUT,   OUTPUT);
  pinMode(PIN_OX_OUT,   OUTPUT);
  pinMode(PIN_IG_MINUS, OUTPUT);
  pinMode(PIN_VTA_OUT,  OUTPUT);
  pinMode(PIN_THW_OUT,  OUTPUT);
  pinMode(PIN_TVIS_IN,  INPUT);
  pinMode(PIN_FAN_IN,   INPUT);

  Serial.begin(115200);
}

void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // 1. Listen for Fault Injection Commands on Pin D0 (RX)
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'F' || cmd == 'f') {
      thwFaultInjected = true;
    } else if (cmd == 'T' || cmd == 't') {
      vtaFaultInjected = true;
    } else if (cmd == 'B' || cmd == 'b') {
      thwFaultInjected = true;
      vtaFaultInjected = true;
    } else if (cmd == 'V' || cmd == 'v') {
      vacLeakInjected = true;   // Crack open the simulated brake booster vacuum hose!
    } else if (cmd == 'N' || cmd == 'n') {
      thwFaultInjected = false;
      vtaFaultInjected = false;
      vacLeakInjected  = false; // Heal all electrical & mechanical faults
    }
  }

  // 2. Dynamic RPM, Thermal & Closed-Loop Fuel Trim Physics (Every 50ms / 20Hz)
  if (currentMillis - lastPhysicsMs >= 50) {
    lastPhysicsMs = currentMillis;

    // Sweep 2000 to 7700 RPM
    if (currentRpm >= 7700) {
      currentRpm = 7700;
      rpmDirection = -250;
    } else if (currentRpm <= 2000) {
      currentRpm = 2000;
      rpmDirection = 50;
    }

    if (rpmDirection > 0) {
      if (digitalRead(PIN_TVIS_IN) == HIGH || currentRpm >= 4400) {
        rpmDirection = 200; // T-VIS secondary runners open!
      } else {
        rpmDirection = 50;
      }
    }
    currentRpm += rpmDirection;

    // Output VTA Voltage
    if (vtaFaultInjected) {
      analogWrite(PIN_VTA_OUT, 0);
    } else {
      int throttlePwm = map(constrain(currentRpm, 850, 7700), 850, 7700, 45, 235);
      analogWrite(PIN_VTA_OUT, throttlePwm);
    }

    // Thermal Physics
    float heatGenerated = (currentRpm / 7500.0) * 0.18;
    float fanCooling    = digitalRead(PIN_FAN_IN) ? 0.22 : 0.04;
    coolantTempF += (heatGenerated - fanCooling);
    coolantTempF = constrain(coolantTempF, 160.0, 230.0);

    if (thwFaultInjected) {
      analogWrite(PIN_THW_OUT, 0);
    } else {
      int thwPwm = map((int)coolantTempF, 160, 230, 165, 45);
      analogWrite(PIN_THW_OUT, thwPwm);
    }

    // =========================================================================
    // LESSON 10: Brake Booster Vacuum Leak + OX Sensor + TCCS VF Fuel Trim Loop
    // =========================================================================
    // High manifold vacuum occurs at Low RPM / Low Throttle (< 3800 RPM).
    // Once RPM climbs >= 3800 RPM (high airflow / low manifold vacuum), the
    // vacuum leak is drowned out by main intake airflow!
    bool highManifoldVacuum = (currentRpm < 3800);
    float oxVoltage = 0.45;

    if (vacLeakInjected && highManifoldVacuum) {
      // Unmetered air enters exhaust -> Wideband's emulated narrowband OX pins LEAN (~0.10V)
      oxVoltage = 0.10;
      // TCCS ECU sees lean OX and slews VF Fuel Trim rapidly toward 4.90V (+20% Max Rich)
      vfVoltage += 0.12;
      if (vfVoltage > 4.90) vfVoltage = 4.90;
    } else {
      // Normal Stoichiometric Closed-Loop Operation (or High-RPM Airflow masking the leak):
      // OX switches cleanly every ~250ms (5 ticks) between 0.18V (Lean) and 0.82V (Rich)
      oxToggleCounter++;
      if (oxToggleCounter >= 5) {
        oxToggleCounter = 0;
        oxRichFlip = !oxRichFlip;
      }
      oxVoltage = oxRichFlip ? 0.82 : 0.18;

      // VF Fuel Trim returns smoothly toward neutral 2.50V (0% trim) with slight dithering
      float targetVf = oxRichFlip ? 2.40 : 2.60;
      if (vfVoltage < targetVf) {
        vfVoltage = min(targetVf, vfVoltage + 0.08f);
      } else {
        vfVoltage = max(targetVf, vfVoltage - 0.08f);
      }
    }

    // Convert OX (0.0V - 1.0V) and VF (0.0V - 5.0V) to 8-bit PWM (0-255 = 0.0V-5.0V)
    int oxPwm = constrain((int)((oxVoltage / 5.0) * 255.0), 0, 255);
    int vfPwm = constrain((int)((vfVoltage / 5.0) * 255.0), 0, 255);
    analogWrite(PIN_OX_OUT, oxPwm);
    analogWrite(PIN_VF_OUT, vfPwm);
  }

  // 3. Generate 4A-GE IG- Tachometer Pulses
  unsigned long halfPeriodMicros = 15000000UL / currentRpm;
  if (currentMicros - lastPulseMicros >= halfPeriodMicros) {
    lastPulseMicros = currentMicros;
    pulseState = !pulseState;
    digitalWrite(PIN_IG_MINUS, pulseState);
  }
}