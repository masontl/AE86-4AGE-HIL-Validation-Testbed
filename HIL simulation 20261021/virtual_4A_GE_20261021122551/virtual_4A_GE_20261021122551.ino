// ====================================================================
// MCU #1: Virtual 1986 4A-GE Engine with Dynamic Thermal Physics
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

// Thermal Simulation Variables (deg F)
float coolantTempF = 175.0;     // Starts warm, heats to 215F under load

void setup() {
  pinMode(PIN_IG_MINUS, OUTPUT);
  pinMode(PIN_VTA_OUT, OUTPUT);
  pinMode(PIN_THW_OUT, OUTPUT);
  pinMode(PIN_TVIS_IN, INPUT);
  pinMode(PIN_FAN_IN, INPUT);
  Serial.begin(115200);
  Serial.println("[Virtual 4A-GE] Engine & Thermal Plant Online.");
}

void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // 1. Dynamic RPM & Thermal Physics Loop (Updates every 50ms)
  if (currentMillis - lastPhysicsMs >= 50) {
    lastPhysicsMs = currentMillis;

    // RPM Sweep (Idle to Redline)
    currentRpm += rpmDirection;
    if (currentRpm >= 7500) { rpmDirection = -75; }
    if (currentRpm <= 900)  { rpmDirection = 50;  }

    // Throttle PWM (15% to 100%)
    int throttlePwm = map(currentRpm, 850, 7500, 40, 255);
    analogWrite(PIN_VTA_OUT, throttlePwm);

    // Thermal Physics Model:
    // Engine combustion heats coolant; electric fan cools coolant
    float heatGenerated = (currentRpm / 7500.0) * 0.18;  // Heat added per 50ms
    float fanCooling    = digitalRead(PIN_FAN_IN) ? 0.22 : 0.04; // Fan active vs passive air
    coolantTempF += (heatGenerated - fanCooling);

    // Keep coolant inside realistic engine limits (160F to 230F)
    coolantTempF = constrain(coolantTempF, 160.0, 230.0);

    // Toyota 4A-GE THW Sensor Curve:
    // NTC Thermistor: Hotter engine = LOWER voltage. 
    // 160F = ~3.2V (PWM 163), 220F = ~1.0V (PWM 51)
    int thwPwm = map((int)coolantTempF, 160, 230, 165, 45);
    analogWrite(PIN_THW_OUT, thwPwm);
  }

  // 2. Generate 4A-GE IG- Tachometer Pulses
  unsigned long halfPeriodMicros = 15000000UL / currentRpm;
  if (currentMicros - lastPulseMicros >= halfPeriodMicros) {
    lastPulseMicros = currentMicros;
    pulseState = !pulseState;
    digitalWrite(PIN_IG_MINUS, pulseState);
  }

  // 3. Telemetry Output
  static unsigned long lastLogMs = 0;
  if (currentMillis - lastLogMs >= 500) {
    lastLogMs = currentMillis;
    bool fanRunning = digitalRead(PIN_FAN_IN);
    Serial.print("[Virtual Engine] RPM: ");
    Serial.print(currentRpm);
    Serial.print(" | Coolant: ");
    Serial.print(coolantTempF, 1);
    Serial.print(" deg F | Fan Status: ");
    Serial.println(fanRunning ? "SPINNING [COOLING]" : "OFF");
  }
}