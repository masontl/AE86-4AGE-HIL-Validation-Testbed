// ====================================================================
// MCU #2: Hachi Smart Controller & Fail-Safe Diagnostic Computer
// SimulIDE HIL Test Bench - Runs on MCU #2
// ====================================================================

const int PIN_IG_INPUT    = 2;   // Input:  Interrupt 0 reads 4A-GE IG- pulses
const int PIN_TPS_INPUT   = A0;  // Input:  Analog VTA throttle position sensor
const int PIN_THW_INPUT   = A1;  // Input:  Analog THW coolant temp sensor
const int PIN_TVIS_OUTPUT = 8;   // Output: T-VIS VSV relay & shift tone
const int PIN_FAN_OUTPUT  = 9;   // Output: Closed-loop Radiator Fan driver
const int PIN_CHECK_ENGINE= 13;  // Output: Toyota 'W' pin Check Engine blink LED

volatile unsigned long lastEdgeTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;

// Operating Thresholds
const int TVIS_ACTIVATE_RPM   = 4400;
const int TVIS_HYSTERESIS_RPM = 150;
const int MIN_THROTTLE_RAW    = 200;

// Valid Sensor Voltage Window: 0.2V to 4.8V (ADC range: 41 to 982)
const int SENSOR_MIN_VALID = 41;   // Below 0.2V = Short to Ground / Snapped Wire
const int SENSOR_MAX_VALID = 982;  // Above 4.8V = Short to 5V / Disconnected Pullup

bool tvisState = false;
bool sensorFaultLatched = false;
unsigned long faultStartTimeMs = 0;

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
  Serial.println("[Hachi Controller] Online. Diagnostic Window: 0.2V - 4.8V Active.");
}

void loop() {
  static unsigned long lastCalcMs = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastCalcMs >= 50) {
    lastCalcMs = currentMillis;

    // 1. Calculate Engine RPM
    unsigned long interval;
    noInterrupts();
    interval = pulseIntervalMicros;
    interrupts();

    int calculatedRpm = 0;
    if (interval > 1000 && interval < 100000) {
      calculatedRpm = 30000000UL / interval;
    }

    // 2. Read Sensors (Filtered Voltages)
    int rawTps = analogRead(PIN_TPS_INPUT);
    int rawThw = analogRead(PIN_THW_INPUT);

    // 3. FAULT DETECTION: 0.2V - 4.8V Out-of-Range Window Check
    if (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID) {
      if (!sensorFaultLatched) {
        sensorFaultLatched = true;
        faultStartTimeMs = currentMillis;
        Serial.println("\n[ALERT] *** CRITICAL SENSOR FAULT DETECTED! ***");
        Serial.println("[ALERT] THW Coolant Voltage Out-of-Bounds (<0.2V or >4.8V)!");
        Serial.println("[ALERT] Entering ECU Fail-Safe Mode: Radiator Fan FORCED 100% ON.\n");
      }
    } else {
      sensorFaultLatched = false;
    }

    // 4. Closed-Loop Fan Control (With Emergency Fail-Safe Override)
    if (sensorFaultLatched) {
      // FAIL-SAFE RULE: If coolant sensor is shorted or broken, NEVER let the engine melt.
      // Force radiator fan 100% ON and flash Check Engine Light (Code 4)
      digitalWrite(PIN_FAN_OUTPUT, HIGH);
      
      // Blink Pin 13 Check Engine LED (Toyota Blink Code 4: 4 fast flashes)
      bool blinkState = ((currentMillis / 250) % 2) == 0;
      digitalWrite(PIN_CHECK_ENGINE, blinkState);

    } else {
      digitalWrite(PIN_CHECK_ENGINE, LOW);

      // Normal Closed-Loop Operation: Turn fan ON when hot (THW ADC < 115, ~195 deg F)
      // NTC thermistor: lower ADC reading = hotter coolant
      if (rawThw < 460) {
        digitalWrite(PIN_FAN_OUTPUT, HIGH); // Coolant >= 195F -> Fan ON
      } else if (rawThw > 540) {
        digitalWrite(PIN_FAN_OUTPUT, LOW);  // Coolant <= 185F -> Fan OFF
      }
    }

    // 5. Normal T-VIS Control
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
  }
}