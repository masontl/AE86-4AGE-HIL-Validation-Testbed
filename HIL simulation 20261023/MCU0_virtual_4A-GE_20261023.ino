// ====================================================================
// MCU #1: Virtual 1986 4A-GE Engine with Python Serial Fault Injection
// SimulIDE HIL Test Bench - Runs on MCU #1
// ====================================================================

const int PIN_IG_MINUS   = 9;   // Output: Simulated IG- tachometer pulse to MCU #2
const int PIN_VTA_OUT    = 10;  // Output: Simulated Throttle Position (PWM -> RC Filter)
const int PIN_THW_OUT    = 11;  // Output: Simulated Coolant Temp (PWM -> RC Filter)
const int PIN_TVIS_IN    = 8;   // Input:  Feedback from MCU #2 T-VIS solenoid
const int PIN_FAN_IN     = 7;   // Input:  Closed-loop feedback from MCU #2 Radiator Fan

unsigned long lastPulseMicros = 0;
unsigned long lastPhysicsMs   = 0;
int currentRpm = 850;
int rpmDirection = 50;
bool pulseState = LOW;

float coolantTempF = 180.0;
bool faultInjectedByPython = false; // Controlled over Serial RX (Pin D0) by Python!

void setup() {
  pinMode(PIN_IG_MINUS, OUTPUT);
  pinMode(PIN_VTA_OUT, OUTPUT);
  pinMode(PIN_THW_OUT, OUTPUT);
  pinMode(PIN_TVIS_IN, INPUT);
  pinMode(PIN_FAN_IN, INPUT);
  
  // Match the baud rate of your SerialPort / MCU #2 (115200 or 9600)
  Serial.begin(115200);
}

void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // 1. Listen for Automated Fault Injection Commands from Python on Pin D0 (RX)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'F' || cmd == 'f') {
      faultInjectedByPython = true;  // Simulate snapped/grounded THW sensor wire!
    } else if (cmd == 'N' || cmd == 'n') {
      faultInjectedByPython = false; // Heal the sensor wire back to normal!
    }
  }

  // 2. Dynamic RPM & Thermal Physics Loop (Every 50ms)
  if (currentMillis - lastPhysicsMs >= 50) {
    lastPhysicsMs = currentMillis;

    currentRpm += rpmDirection;
    if (currentRpm >= 7500) { rpmDirection = -75; }
    if (currentRpm <= 900)  { rpmDirection = 50;  }

    int throttlePwm = map(currentRpm, 850, 7500, 40, 255);
    analogWrite(PIN_VTA_OUT, throttlePwm);

    // Thermal Physics
    float heatGenerated = (currentRpm / 7500.0) * 0.18;
    float fanCooling    = digitalRead(PIN_FAN_IN) ? 0.22 : 0.04;
    coolantTempF += (heatGenerated - fanCooling);
    coolantTempF = constrain(coolantTempF, 160.0, 230.0);

    // Output THW Voltage (Unless Python injected a broken-wire fault!)
    if (faultInjectedByPython) {
      analogWrite(PIN_THW_OUT, 0); // Force 0.0V (Short to Ground / Broken Wire)
    } else {
      int thwPwm = map((int)coolantTempF, 160, 230, 165, 45);
      analogWrite(PIN_THW_OUT, thwPwm);
    }
  }

  // 3. Generate 4A-GE IG- Tachometer Pulses
  unsigned long halfPeriodMicros = 15000000UL / currentRpm;
  if (currentMicros - lastPulseMicros >= halfPeriodMicros) {
    lastPulseMicros = currentMicros;
    pulseState = !pulseState;
    digitalWrite(PIN_IG_MINUS, pulseState);
  }
}