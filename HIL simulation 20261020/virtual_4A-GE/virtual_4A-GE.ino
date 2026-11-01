// ====================================================================
// MCU #1: Virtual 1986 Toyota 4A-GE Engine Simulator (The "Plant")
// SimulIDE HIL Test Bench - Runs on MCU #1
// ====================================================================

const int PIN_IG_MINUS   = 9;   // Output: Simulated IG- tachometer pulse to MCU #2
const int PIN_VTA_OUT    = 10;  // Output: Simulated Throttle Position (PWM/Analog)
const int PIN_TVIS_IN    = 8;   // Input:  Feedback from MCU #2 T-VIS solenoid driver

unsigned long lastPulseMicros = 0;
unsigned long lastRpmSweepMs  = 0;
int currentRpm = 850;           // Start at idle
int rpmDirection = 50;          // Sweep step (RPM / 50ms)
bool pulseState = LOW;

void setup() {
  pinMode(PIN_IG_MINUS, OUTPUT);
  pinMode(PIN_VTA_OUT, OUTPUT);
  pinMode(PIN_TVIS_IN, INPUT);
  Serial.begin(115200);
  Serial.println("[Virtual 4A-GE] Engine Plant Initialized.");
}

void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // 1. Dynamic RPM Sweep: Simulates revving up and down the powerband
  if (currentMillis - lastRpmSweepMs >= 50) {
    lastRpmSweepMs = currentMillis;
    currentRpm += rpmDirection;
    if (currentRpm >= 7500) { rpmDirection = -75; }  // Hit redline, drop revs
    if (currentRpm <= 900)  { rpmDirection = 50;  }  // Hit idle, climb revs

    // Simulate throttle pedal opening in proportion to RPM
    int throttlePwm = map(currentRpm, 850, 7500, 40, 255);
    analogWrite(PIN_VTA_OUT, throttlePwm);
  }

  // 2. Generate 4A-GE IG- Tachometer Pulses
  // 4-cylinder, 4-stroke: 2 ignition events per crank revolution
  // Interval (microseconds) = (60,000,000 / (RPM * 2)) / 2 for half-period
  unsigned long halfPeriodMicros = 15000000UL / currentRpm;

  if (currentMicros - lastPulseMicros >= halfPeriodMicros) {
    lastPulseMicros = currentMicros;
    pulseState = !pulseState;
    digitalWrite(PIN_IG_MINUS, pulseState);
  }

  // 3. Closed-Loop Telemetry Check
  static unsigned long lastLogMs = 0;
  if (currentMillis - lastLogMs >= 500) {
    lastLogMs = currentMillis;
    bool tvisActive = digitalRead(PIN_TVIS_IN);
    Serial.print("[Virtual Engine] RPM: ");
    Serial.print(currentRpm);
    Serial.print(" | T-VIS Secondary Runners: ");
    Serial.println(tvisActive ? "OPEN (High-Flow)" : "CLOSED (High-Torque)");
  }
}