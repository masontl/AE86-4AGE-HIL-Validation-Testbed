// ====================================================================
// MCU #2: Hachi Smart T-VIS & RPM Controller (Device Under Test)
// SimulIDE HIL Test Bench - Runs on MCU #2
// ====================================================================

const int PIN_IG_INPUT    = 2;   // Input:  Interrupt 0 reads 4A-GE IG- pulses
const int PIN_TPS_INPUT   = A0;  // Input:  Analog VTA throttle position sensor
const int PIN_TVIS_OUTPUT = 8;   // Output: Drives T-VIS VSV relay & shift LED

volatile unsigned long lastEdgeTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile bool newPulseDetected = false;

// 4A-GE Crossover Parameters
const int TVIS_ACTIVATE_RPM   = 4400; // Factory Toyota crossover point
const int TVIS_HYSTERESIS_RPM = 150;  // Prevents relay chattering at boundary
const int MIN_THROTTLE_RAW    = 200;  // >20% throttle required for T-VIS engagement

bool tvisState = false;

// Hardware Interrupt Service Routine for IG- Pulse Timing
void onIgnitionPulse() {
  unsigned long now = micros();
  pulseIntervalMicros = now - lastEdgeTimeMicros;
  lastEdgeTimeMicros = now;
  newPulseDetected = true;
}

void setup() {
  pinMode(PIN_IG_INPUT, INPUT_PULLUP);
  pinMode(PIN_TVIS_OUTPUT, OUTPUT);
  digitalWrite(PIN_TVIS_OUTPUT, LOW);

  // Attach interrupt on falling edge of ignition pulse
  attachInterrupt(digitalPinToInterrupt(PIN_IG_INPUT), onIgnitionPulse, FALLING);

  Serial.begin(115200);
  Serial.println("[Hachi Controller] Online. Monitoring IG- and VTA...");
}

void loop() {
  static unsigned long lastCalcMs = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastCalcMs >= 50) {
    lastCalcMs = currentMillis;

    // 1. Calculate RPM from measured pulse interval
    // 4-cylinder: RPM = (60,000,000 / (Interval * 2))
    unsigned long interval;
    noInterrupts();
    interval = pulseIntervalMicros;
    interrupts();

    int calculatedRpm = 0;
    if (interval > 1000 && interval < 100000) {
      calculatedRpm = 30000000UL / interval;
    }

    // 2. Read Throttle Position
    int rawTps = analogRead(PIN_TPS_INPUT);
    int throttlePercent = map(rawTps, 0, 1023, 0, 100);

    // 3. Closed-Loop T-VIS Actuation Logic with Hysteresis
    if (!tvisState) {
      if (calculatedRpm >= TVIS_ACTIVATE_RPM && rawTps >= MIN_THROTTLE_RAW) {
        tvisState = true;
        digitalWrite(PIN_TVIS_OUTPUT, HIGH);
      }
    } else {
      if (calculatedRpm < (TVIS_ACTIVATE_RPM - TVIS_HYSTERESIS_RPM) || rawTps < MIN_THROTTLE_RAW) {
        tvisState = false;
        digitalWrite(PIN_TVIS_OUTPUT, LOW);
      }
    }

    // 4. Output Real-Time Telemetry to Virtual Terminal
    Serial.print("RPM: ");
    Serial.print(calculatedRpm);
    Serial.print(" | TPS: ");
    Serial.print(throttlePercent);
    Serial.print("% | T-VIS Solenoid: ");
    Serial.println(tvisState ? "ENGAGED [1]" : "BYPASS [0]");
  }
}