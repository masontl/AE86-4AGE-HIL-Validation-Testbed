// ====================================================================
// LESSON 8 — MCU #1: Virtual 1986 4A-GE Engine (Built on Proven Baseline)
// SimulIDE HIL Test Bench - Runs on MCU #1
// Commands on RX (115200 baud):
//   'F' = Inject THW Coolant Fault (Code 4: 0V Short-to-Ground on D11)
//   'T' = Inject VTA TPS Fault     (Code 7: 0V Short-to-Ground on D10)
//   'B' = Inject BOTH THW + VTA Faults simultaneously
//   'N' = Restore Normal Operation on all sensors
// ====================================================================

#include <Arduino.h>

const int PIN_IG_MINUS = 9;   // Output: Simulated IG- tachometer pulse to MCU #2
const int PIN_VTA_OUT  = 10;  // Output: Simulated Throttle Position (PWM -> RC Filter)
const int PIN_THW_OUT  = 11;  // Output: Simulated Coolant Temp (PWM -> RC Filter)
const int PIN_TVIS_IN  = 8;   // Input:  Feedback from MCU #2 T-VIS solenoid (D8)
const int PIN_FAN_IN   = 7;   // Input:  Closed-loop feedback from MCU #2 Radiator Fan (D9)

unsigned long lastPulseMicros = 0;
unsigned long lastPhysicsMs   = 0;
int currentRpm   = 2000;
int rpmDirection = 50;
bool pulseState  = LOW;

float coolantTempF = 180.0;

// Lesson 8 Multi-Sensor Fault Injection Flags
bool thwFaultInjected = false; // 'F' or 'B'
bool vtaFaultInjected = false; // 'T' or 'B'

void setup() {
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

  // 1. Listen for Automated Fault Injection Commands from Python on Pin D0 (RX)
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'F' || cmd == 'f') {
      thwFaultInjected = true;  // Simulate snapped/grounded THW wire (Code 4)
    } else if (cmd == 'T' || cmd == 't') {
      vtaFaultInjected = true;  // Simulate snapped/grounded VTA TPS wire (Code 7)
    } else if (cmd == 'B' || cmd == 'b') {
      thwFaultInjected = true;  // Simulate dual fault (THW + VTA)
      vtaFaultInjected = true;
    } else if (cmd == 'N' || cmd == 'n') {
      thwFaultInjected = false; // Heal both sensors back to normal!
      vtaFaultInjected = false;
    }
  }

  // 2. Dynamic RPM & Thermal Physics Loop (Every 50ms)
  if (currentMillis - lastPhysicsMs >= 50) {
    lastPhysicsMs = currentMillis;

    // Check if we hit the top (7700 RPM) or bottom (2000 RPM) of the sweep
    if (currentRpm >= 7700) {
      currentRpm = 7700;
      rpmDirection = -250; // Decelerate at -250/tick
    } else if (currentRpm <= 2000) {
      currentRpm = 2000;
      rpmDirection = 50;   // Accelerate from 2000 RPM at +50/tick
    }

    // If accelerating, boost to +200 once T-VIS opens (4400+ RPM)
    if (rpmDirection > 0) {
      if (digitalRead(PIN_TVIS_IN) == HIGH || currentRpm >= 4400) {
        rpmDirection = 200; // T-VIS secondary runners OPEN -> +200 surge!
      } else {
        rpmDirection = 50;  // Below 4400 RPM -> +50 low-end pull
      }
    }

    currentRpm += rpmDirection;

    // Output VTA Voltage (Normal: 45 to 235 PWM = ~180 to ~940 ADC on MCU#2)
    if (vtaFaultInjected) {
      analogWrite(PIN_VTA_OUT, 0); // Force 0.0V (Short to Ground -> Code 7)
    } else {
      int throttlePwm = map(constrain(currentRpm, 850, 7700), 850, 7700, 45, 235);
      analogWrite(PIN_VTA_OUT, throttlePwm);
    }

    // Proven Thermal Physics (12 deg F natural hysteresis w/ MCU#2 460/540 ADC thresholds)
    float heatGenerated = (currentRpm / 7500.0) * 0.18;
    float fanCooling    = digitalRead(PIN_FAN_IN) ? 0.22 : 0.04;
    coolantTempF += (heatGenerated - fanCooling);
    coolantTempF = constrain(coolantTempF, 160.0, 230.0);

    // Output THW Voltage (Normal: 160F->165 PWM [~660 ADC], 230F->45 PWM [~180 ADC])
    if (thwFaultInjected) {
      analogWrite(PIN_THW_OUT, 0); // Force 0.0V (Short to Ground -> Code 4)
    } else {
      int thwPwm = map((int)coolantTempF, 160, 230, 165, 45);
      analogWrite(PIN_THW_OUT, thwPwm);
    }
  }

  // 3. Generate 4A-GE IG- Tachometer Pulses (2 pulses/rev -> Full period = 30,000,000 / RPM)
  unsigned long halfPeriodMicros = 15000000UL / currentRpm;
  if (currentMicros - lastPulseMicros >= halfPeriodMicros) {
    lastPulseMicros = currentMicros;
    pulseState = !pulseState;
    digitalWrite(PIN_IG_MINUS, pulseState);
  }
}