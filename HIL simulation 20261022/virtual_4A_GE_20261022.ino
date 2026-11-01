// ====================================================================
// MCU #2: Hachi Smart Controller & Telemetry Bridge
// SimulIDE HIL Test Bench - Runs on MCU #2
// ====================================================================

const int PIN_IG_INPUT    = 2;   // Input:  Interrupt 0 reads 4A-GE IG- pulses
const int PIN_TPS_INPUT   = A0;  // Input:  Analog VTA throttle position
const int PIN_THW_INPUT   = A1;  // Input:  Analog THW coolant temp
const int PIN_TVIS_OUTPUT = 8;   // Output: T-VIS VSV relay & shift tone
const int PIN_FAN_OUTPUT  = 9;   // Output: Radiator Fan driver
const int PIN_CHECK_ENGINE= 13;  // Output: Check Engine Light (Code 4)

volatile unsigned long lastEdgeTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;

// Operating Thresholds
const int TVIS_ACTIVATE_RPM   = 4400;
const int TVIS_HYSTERESIS_RPM = 150;
const int MIN_THROTTLE_RAW    = 200;

// Diagnostic Voltage Window: 0.2V to 4.8V (ADC: 41 to 982)
const int SENSOR_MIN_VALID = 41;
const int SENSOR_MAX_VALID = 982;

bool tvisState = false;
bool sensorFaultLatched = false;
int activeDtcCode = 0; // 0 = No Fault, 4 = Coolant Circuit Fault

void onIgnitionPulse() {
  unsigned long now = micros();
  pulseIntervalMicros = now - lastEdgeTimeMicros;
  lastEdgeTimeMicros = now;
}

void setup() {
  pinMode(PIN_IG_INPUT, INPUT_PULLUP);
  pinMode(PIN_TVIS_OUTPUT, OUTPUT);
  pinMode(PIN_FAN_OUTPUT, OUTPUT);
  pinMode(PIN_CHECK_ENGINE, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_IG_INPUT), onIgnitionPulse, FALLING);
  Serial.begin(115200);
}

void loop() {
  static unsigned long lastLoopMs = 0;
  unsigned long currentMillis = millis();

  // 1. Process Incoming Serial Commands from Host PC
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      sensorFaultLatched = false;
      activeDtcCode = 0;
      Serial.println("[CMD] Fault Latches Reset by Host.");
    }
  }

  // 2. Main Control & Telemetry Loop (Runs at 20 Hz / 50ms)
  if (currentMillis - lastLoopMs >= 50) {
    lastLoopMs = currentMillis;

    // Calculate RPM
    unsigned long interval;
    noInterrupts();
    interval = pulseIntervalMicros;
    interrupts();

    int calculatedRpm = 0;
    if (interval > 1000 && interval < 100000) {
      calculatedRpm = 30000000UL / interval;
    }

    // Read Filtered Analog Sensors
    int rawTps = analogRead(PIN_TPS_INPUT);
    int rawThw = analogRead(PIN_THW_INPUT);
    int throttlePct = map(rawTps, 0, 1023, 0, 100);

    // Fault Detection Window
    if (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID) {
      sensorFaultLatched = true;
      activeDtcCode = 4; // Toyota Code 4: Coolant Sensor Circuit
    }

    // Closed-Loop Thermal & Fan Control (With Fixed ADC Scaling)
    if (sensorFaultLatched) {
      digitalWrite(PIN_FAN_OUTPUT, HIGH); // Fail-safe: Fan 100% ON
      bool blink = ((currentMillis / 250) % 2) == 0;
      digitalWrite(PIN_CHECK_ENGINE, blink);
    } else {
      digitalWrite(PIN_CHECK_ENGINE, LOW);
      if (rawThw < 460) {       // Coolant >= 195F (scaled to 10-bit ADC)
        digitalWrite(PIN_FAN_OUTPUT, HIGH);
      } else if (rawThw > 540) { // Coolant <= 185F
        digitalWrite(PIN_FAN_OUTPUT, LOW);
      }
    }

    // Closed-Loop T-VIS Control
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

    // 3. Emit Structured Telemetry Sentence for Automated Python Parser
    // Format: $TELEM,TimestampMs,RPM,ThrottlePct,RawTHW,TVIS,Fan,DTC*
    Serial.print("$TELEM,");
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(calculatedRpm);
    Serial.print(",");
    Serial.print(throttlePct);
    Serial.print(",");
    Serial.print(rawThw);
    Serial.print(",");
    Serial.print(tvisState ? 1 : 0);
    Serial.print(",");
    Serial.print(digitalRead(PIN_FAN_OUTPUT));
    Serial.print(",");
    Serial.print(activeDtcCode);
    Serial.println("*");
  }
}