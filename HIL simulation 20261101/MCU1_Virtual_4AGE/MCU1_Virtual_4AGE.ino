// ====================================================================
// LESSON 11 — MCU #1: Virtual 1986 4A-GE Engine Plant
// Cold-Start Warmup + Bi-Metal AAV Idle-Up + Open/Closed-Loop Gating
// SimulIDE HIL Test Bench - Runs on MCU #1
// Commands on RX (115200 baud):
//   'C' = Trigger Cold-Start Reset (Drops Coolant to 70°F ambient)
//   'S' = Inject Stuck-Open Thermostat (Soft Thermal Fault: Caps THW ~125°F < 140°F CL threshold!)
//   'V' = Inject Brake Booster Vacuum Leak (Soft Mechanical Fault: Low-Load Lean OX -> Max Rich VF in CL)
//   'F' = Inject THW Coolant Fault (Hard Code 4: 0V Short-to-Ground on D11)
//   'T' = Inject VTA TPS Fault     (Hard Code 7: 0V Short-to-Ground on D10)
//   'B' = Inject BOTH THW + VTA Hard Faults simultaneously
//   'N' = Restore Normal Operation on all sensors, thermostat & vacuum hose
// ====================================================================

#include <Arduino.h>

const int PIN_VF_OUT   = 5;   // Output: TCCS VF Fuel Trim Diagnostic Voltage (0-5V PWM -> MCU#2 A3)
const int PIN_OX_OUT   = 6;   // Output: Wideband's Emulated Narrowband OX (0-1V PWM -> MCU#2 A2)
const int PIN_FAN_IN   = 7;   // Input:  Closed-loop feedback from MCU #2 Radiator Fan (D9)
const int PIN_TVIS_IN  = 8;   // Input:  Feedback from MCU #2 T-VIS solenoid (D8)
const int PIN_IG_MINUS = 9;   // Output: Simulated IG- tachometer pulse to MCU #2 (D2)
const int PIN_VTA_OUT  = 10;  // Output: Simulated Throttle Position (PWM -> RC -> MCU#2 A0)
const int PIN_THW_OUT  = 11;  // Output: Simulated Coolant Temp (PWM -> RC -> MCU#2 A1)

const float CLOSED_LOOP_ENTRY_F = 140.0; // 1986 TCCS 4A-GE Closed-Loop Warmup Threshold (60°C / 140°F)

unsigned long lastPulseMicros = 0;
unsigned long lastPhysicsMs   = 0;
int currentRpm   = 2200;
int rpmDirection = 50;
bool pulseState  = LOW;

// Start Cold at 70.0°F so every mission captures the Open-Loop -> Closed-Loop Warmup Transition!
float coolantTempF = 70.0;

// Closed-Loop Fuel Trim Integrator State (0.00V in Open-Loop; 2.50V = 0% Base Trim in Closed-Loop)
float vfVoltage       = 0.00;
bool  oxRichFlip      = false;
int   oxToggleCounter = 0;

// Fault Injection Flags
bool thwFaultInjected        = false; // 'F' or 'B' (Hard Electrical Fault: Code 4)
bool vtaFaultInjected        = false; // 'T' or 'B' (Hard Electrical Fault: Code 7)
bool vacLeakInjected         = false; // 'V'        (Soft Mechanical: Brake Booster Vacuum Leak)
bool stuckThermostatInjected = false; // 'S'        (Soft Thermal: Stuck-Open Thermostat < 140°F)

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
      vacLeakInjected = true;         // Crack open simulated brake booster vacuum hose
    } else if (cmd == 'S' || cmd == 's') {
      stuckThermostatInjected = true; // Jam thermostat open (over-cooling -> traps ECU < 140°F)
    } else if (cmd == 'C' || cmd == 'c') {
      coolantTempF = 70.0;            // Reset engine block to 70°F ambient cold start
      vfVoltage    = 0.00;
    } else if (cmd == 'N' || cmd == 'n') {
      thwFaultInjected        = false;
      vtaFaultInjected        = false;
      vacLeakInjected         = false;
      stuckThermostatInjected = false; // Heal all electrical, mechanical & thermal faults
    }
  }

  // 2. Dynamic RPM, AAV Idle-Up, Thermal & Open/Closed-Loop Physics (Every 50ms / 20Hz)
  if (currentMillis - lastPhysicsMs >= 50) {
    lastPhysicsMs = currentMillis;

    // Bi-Metal Auxiliary Air Valve (AAV) Cold Idle-Up Modeling:
    // Holds minimum RPM floor at 2200 RPM at 70°F, tapering smoothly to 1100 RPM by 140°F.
    int aavFloorRpm = 1100;
    if (coolantTempF < CLOSED_LOOP_ENTRY_F) {
      aavFloorRpm = map((int)constrain(coolantTempF, 70.0, 140.0), 70, 140, 2200, 1100);
    }

    // Sweep between AAV Floor RPM and 7700 RPM Redline
    if (currentRpm >= 7700) {
      currentRpm = 7700;
      rpmDirection = -250;
    } else if (currentRpm <= aavFloorRpm) {
      currentRpm = aavFloorRpm;
      rpmDirection = 50;
    }

    if (rpmDirection > 0) {
      if (digitalRead(PIN_TVIS_IN) == HIGH || currentRpm >= 4400) {
        rpmDirection = 200; // T-VIS secondary intake runners open!
      } else {
        rpmDirection = 50;
      }
    }
    currentRpm += rpmDirection;

    // Output VTA Throttle Position Voltage
    if (vtaFaultInjected) {
      analogWrite(PIN_VTA_OUT, 0);
    } else {
      int throttlePwm = map(constrain(currentRpm, 850, 7700), 850, 7700, 45, 235);
      analogWrite(PIN_VTA_OUT, throttlePwm);
    }

    // Thermal Physics (60°F to 230°F range)
    // While thermostat is closed (< 175°F normal), block warms up briskly (~8-10 seconds to 140°F).
    // If Stuck-Open Thermostat ('S') is active, cold radiator flow caps block temp around 125°F!
    float heatGenerated = (currentRpm / 7500.0) * 0.65 + 0.18;
    float coolingLoss   = 0.04;

    if (stuckThermostatInjected) {
      // Unregulated radiator flow strips heat once coolant approaches 125°F
      coolingLoss = (coolantTempF > 122.0) ? (heatGenerated + 0.25) : 0.15;
    } else if (digitalRead(PIN_FAN_IN) == HIGH) {
      coolingLoss = 0.85; // Electric radiator fan active (> 195°F)
    } else if (coolantTempF >= 180.0) {
      coolingLoss = 0.42; // Normal regulated 180°F thermostat equilibrium
    }

    coolantTempF += (heatGenerated - coolingLoss);
    coolantTempF = constrain(coolantTempF, 60.0, 230.0);

    // Output NTC Coolant Thermistor (THW) Voltage:
    // 60°F (Cold) = PWM 230 (~4.51V / ~923 ADC) -> 230°F (Hot) = PWM 35 (~0.69V / ~140 ADC)
    if (thwFaultInjected) {
      analogWrite(PIN_THW_OUT, 0);
    } else {
      int thwPwm = map((int)coolantTempF, 60, 230, 230, 35);
      analogWrite(PIN_THW_OUT, thwPwm);
    }

    // =========================================================================
    // LESSON 11: TCCS Open-Loop (< 140°F) vs. Closed-Loop (>= 140°F) Fuel Control
    // =========================================================================
    bool inClosedLoop       = (coolantTempF >= CLOSED_LOOP_ENTRY_F) && !thwFaultInjected;
    bool highManifoldVacuum = (currentRpm < 3800);
    float oxVoltage         = 0.45;

    if (!inClosedLoop) {
      // OPEN-LOOP WARMUP (or THW Fault Failsafe):
      // TCCS ignores OX sensor, injects fixed cold-start warmup enrichment,
      // and outputs 0.00V on the VF diagnostic pin!
      vfVoltage = 0.00;
      // Rich cold-start enrichment keeps exhaust OX reading rich (~0.88V), or 0.20V if vacuum leak is huge
      oxVoltage = (vacLeakInjected && highManifoldVacuum) ? 0.20 : 0.88;
    } else {
      // CLOSED-LOOP OPERATION (Coolant >= 140°F):
      // If just entering Closed-Loop from Open-Loop (0.0V), snap VF integrator to neutral 2.50V (0% trim)
      if (vfVoltage < 0.40) {
        vfVoltage = 2.50;
      }

      if (vacLeakInjected && highManifoldVacuum) {
        // Unmetered air under high manifold vacuum (< 3800 RPM) pins OX lean (~0.10V)
        oxVoltage = 0.10;
        // TCCS integrator slews VF rapidly toward 4.90V (+20% Max Rich)
        vfVoltage += 0.12;
        if (vfVoltage > 4.90) vfVoltage = 4.90;
      } else {
        // Normal Stoichiometric Closed-Loop Dithering (or High-RPM Airflow masking the leak)
        oxToggleCounter++;
        if (oxToggleCounter >= 5) {
          oxToggleCounter = 0;
          oxRichFlip = !oxRichFlip;
        }
        oxVoltage = oxRichFlip ? 0.82 : 0.18;

        float targetVf = oxRichFlip ? 2.40 : 2.60;
        if (vfVoltage < targetVf) {
          vfVoltage = min(targetVf, vfVoltage + 0.08f);
        } else {
          vfVoltage = max(targetVf, vfVoltage - 0.08f);
        }
      }
    }

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