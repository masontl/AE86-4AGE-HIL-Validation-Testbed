// ====================================================================
// LESSON 12 — MCU #1: Virtual 1986 4A-GE Engine Plant
// Cold-Start Warmup + Bi-Metal AAV + Open/Closed-Loop + VAST IGf Handshake
// SimulIDE HIL Test Bench - Runs on MCU #1
// Commands on RX (115200 baud):
//   'C' = Trigger Cold-Start Reset (Drops Coolant to 70°F ambient)
//   'S' = Inject Stuck-Open Thermostat (Soft Thermal Fault: Caps THW ~125°F < 140°F CL threshold!)
//   'V' = Inject Brake Booster Vacuum Leak (Soft Mechanical Fault: Low-Load Lean OX -> Max Rich VF in CL)
//   'I' = Inject VAST Igniter IGf Fault (Hard Code 14: Kills IGf pulses on D4 while IG- spins -> Fuel Cut!)
//   'F' = Inject THW Coolant Fault (Hard Code 4: 0V Short-to-Ground on D11)
//   'T' = Inject VTA TPS Fault     (Hard Code 7: 0V Short-to-Ground on D10)
//   'B' = Inject BOTH THW + VTA Hard Faults simultaneously
//   'N' = Restore Normal Operation on all sensors, igniter, thermostat & vacuum hose
// ====================================================================

#include <Arduino.h>

const int PIN_IGF_OUT  = 4;   // Output: Simulated VAST Igniter IGf Spark Confirmation (D4 -> MCU#2 D3/INT1)
const int PIN_VF_OUT   = 5;   // Output: TCCS VF Fuel Trim Diagnostic Voltage (0-5V PWM -> MCU#2 A3)
const int PIN_OX_OUT   = 6;   // Output: Wideband's Emulated Narrowband OX (0-1V PWM -> MCU#2 A2)
const int PIN_FAN_IN   = 7;   // Input:  Closed-loop feedback from MCU #2 Radiator Fan (D9)
const int PIN_TVIS_IN  = 8;   // Input:  Feedback from MCU #2 T-VIS solenoid (D8)
const int PIN_IG_MINUS = 9;   // Output: Simulated IG- tachometer pulse to MCU #2 (D2/INT0)
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
bool igfFaultInjected        = false; // 'I'        (Hard Ignition Fault: Code 14 Missing IGf Signal)
unsigned long igfDroppedSparks = 0;   // Counts missed IGf pulses to trigger TCCS Fuel-Cut Failsafe

void setup() {
  pinMode(PIN_IGF_OUT,  OUTPUT);
  pinMode(PIN_VF_OUT,   OUTPUT);
  pinMode(PIN_OX_OUT,   OUTPUT);
  pinMode(PIN_IG_MINUS, OUTPUT);
  pinMode(PIN_VTA_OUT,  OUTPUT);
  pinMode(PIN_THW_OUT,  OUTPUT);
  pinMode(PIN_TVIS_IN,  INPUT);
  pinMode(PIN_FAN_IN,   INPUT);

  digitalWrite(PIN_IGF_OUT, HIGH);
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
    } else if (cmd == 'I' || cmd == 'i') {
      igfFaultInjected = true;        // Sever VAST Igniter IGf confirmation line (Code 14!)
      igfDroppedSparks = 0;
    } else if (cmd == 'C' || cmd == 'c') {
      coolantTempF = 70.0;            // Reset engine block to 70°F ambient cold start
      vfVoltage    = 0.00;
    } else if (cmd == 'N' || cmd == 'n') {
      thwFaultInjected        = false;
      vtaFaultInjected        = false;
      vacLeakInjected         = false;
      stuckThermostatInjected = false;
      igfFaultInjected        = false; // Heal all electrical, ignition, mechanical & thermal faults
      igfDroppedSparks        = 0;
    }
  }

  // 2. Dynamic RPM, AAV Idle-Up, Thermal & Open/Closed-Loop Physics (Every 50ms / 20Hz)
  if (currentMillis - lastPhysicsMs >= 50) {
    lastPhysicsMs = currentMillis;

    // Bi-Metal Auxiliary Air Valve (AAV) Cold Idle-Up Modeling:
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

    // In MCU #1 dynamic physics loop:
    if (rpmDirection > 0) {
      if (digitalRead(PIN_TVIS_IN) == HIGH || currentRpm >= 4400) {
        rpmDirection = 100; // T-VIS secondary intake runners open!
      } else {
        rpmDirection = 30;
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
    // Note: If Code 14 Fuel-Cut is active (igfDroppedSparks >= 6), combustion ceases and block cools!
    bool code14FuelCut = (igfFaultInjected && igfDroppedSparks >= 6);
    float heatGenerated = code14FuelCut ? 0.02 : ((currentRpm / 7500.0) * 0.65 + 0.18);
    float coolingLoss   = 0.04;

    if (stuckThermostatInjected) {
      coolingLoss = (coolantTempF > 122.0) ? (heatGenerated + 0.25) : 0.15;
    } else if (digitalRead(PIN_FAN_IN) == HIGH) {
      coolingLoss = 0.85; // Electric radiator fan active (> 195°F)
    } else if (coolantTempF >= 180.0) {
      coolingLoss = 0.42; // Normal regulated 180°F thermostat equilibrium
    }

    coolantTempF += (heatGenerated - coolingLoss);
    coolantTempF = constrain(coolantTempF, 60.0, 230.0);

    // Output NTC Coolant Thermistor (THW) Voltage
    if (thwFaultInjected) {
      analogWrite(PIN_THW_OUT, 0);
    } else {
      int thwPwm = map((int)coolantTempF, 60, 230, 230, 35);
      analogWrite(PIN_THW_OUT, thwPwm);
    }

    // =========================================================================
    // LESSON 12: TCCS Open/Closed-Loop + Code 14 Igniter Fuel-Cut Failsafe
    // =========================================================================
    bool inClosedLoop       = (coolantTempF >= CLOSED_LOOP_ENTRY_F) && !thwFaultInjected && !code14FuelCut;
    bool highManifoldVacuum = (currentRpm < 3800);
    float oxVoltage         = 0.45;

    if (code14FuelCut) {
      // CODE 14 IGNITER FAILSAFE (No IGf Confirmation -> TCCS Cuts Fuel Injectors):
      // Unburned air floods exhaust (OX drops to 0.02V dead-lean) & ECU drops out of Closed-Loop (VF = 0.00V)
      vfVoltage = 0.00;
      oxVoltage = 0.02;
    } else if (!inClosedLoop) {
      // OPEN-LOOP WARMUP (or THW Fault Failsafe):
      vfVoltage = 0.00;
      oxVoltage = (vacLeakInjected && highManifoldVacuum) ? 0.20 : 0.88;
    } else {
      // CLOSED-LOOP OPERATION (Coolant >= 140°F, Sensors & Igniter Healthy):
      if (vfVoltage < 0.40) {
        vfVoltage = 2.50;
      }

      if (vacLeakInjected && highManifoldVacuum) {
        oxVoltage = 0.10;
        vfVoltage += 0.12;
        if (vfVoltage > 4.90) vfVoltage = 4.90;
      } else {
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

  // 3. Generate 4A-GE IG- Tachometer Pulses (D9) & VAST IGf Confirmation Pulses (D4)
  unsigned long halfPeriodMicros = 15000000UL / currentRpm;
  if (currentMicros - lastPulseMicros >= halfPeriodMicros) {
    lastPulseMicros = currentMicros;
    pulseState = !pulseState;
    digitalWrite(PIN_IG_MINUS, pulseState);

    if (!igfFaultInjected) {
      // Normal VAST Igniter operation: return matching IGf confirmation edge on D4
      digitalWrite(PIN_IGF_OUT, pulseState);
    } else {
      // Fault 'I' active: Igniter fails to return IGf pulse (idles HIGH via pull-up)
      digitalWrite(PIN_IGF_OUT, HIGH);
      if (pulseState == LOW) {
        igfDroppedSparks++; // Count every falling spark edge that lacked an IGf return pulse
      }
    }
  }
}