// ====================================================================
// MCU #2: Hachi 20x4 I2C Cockpit Display (AIP31068_I2C @ Address 39)
// SimulIDE HIL Test Bench - Runs on MCU #2 (SDA = A4, SCL = A5)
// ====================================================================

#include <Wire.h>
#include <LiquidCrystal_AIP31068_I2C.h>

// Initialize AIP31068 I2C Display: Address = 39 (0x27), 20 Cols, 4 Rows
LiquidCrystal_AIP31068_I2C lcd(39, 20, 4);

const int PIN_IG_INPUT    = 2;   // Input:  Interrupt 0 reads 4A-GE IG- pulses
const int PIN_TPS_INPUT   = A0;  // Input:  Analog VTA throttle position (0-5V)
const int PIN_THW_INPUT   = A1;  // Input:  Analog THW coolant temp (0-5V)
const int PIN_SHIFT_HMI   = 7;   // Output: Driver Shift-Light & Acoustic Beep
const int PIN_TVIS_OUTPUT = 8;   // Output: T-VIS VSV Relay Status Indicator
const int PIN_FAN_OUTPUT  = 9;   // Output: Electric Radiator Fan Driver
const int PIN_CHECK_ENGINE= 13;  // Output: Check Engine Light (Code 4)

volatile unsigned long lastEdgeTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;

// 4A-GE Operating Thresholds
const int TVIS_ACTIVATE_RPM   = 4400;
const int TVIS_HYSTERESIS_RPM = 150;
const int SHIFT_LIGHT_RPM     = 7200; // Shift alert before 7600 RPM redline
const int MIN_THROTTLE_RAW    = 200;

// Diagnostic Voltage Window (0.2V to 4.8V -> ADC 41 to 982)
const int SENSOR_MIN_VALID = 41;
const int SENSOR_MAX_VALID = 982;

bool tvisState = false;
bool sensorFaultLatched = false;
int activeDtcCode = 0;

void onIgnitionPulse() {
  unsigned long now = micros();
  pulseIntervalMicros = now - lastEdgeTimeMicros;
  lastEdgeTimeMicros = now;
}

// Helper to print a right-aligned 4-digit number without flickering lcd.clear()
void printPadded4(int val) {
  if (val < 1000) lcd.print(" ");
  if (val < 100)  lcd.print(" ");
  if (val < 10)   lcd.print(" ");
  lcd.print(val);
}

// Helper to print a right-aligned 3-digit number
void printPadded3(int val) {
  if (val < 100) lcd.print(" ");
  if (val < 10)  lcd.print(" ");
  lcd.print(val);
}

// Custom 5x8 CGRAM Bitmaps for 40-Step Sub-Pixel RPM Bar Graph
byte bar1[8] = {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000};
byte bar2[8] = {0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000};
byte bar3[8] = {0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100};
byte bar4[8] = {0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110};
byte bar5[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};

void setup() {
  pinMode(PIN_IG_INPUT, INPUT_PULLUP);
  pinMode(PIN_SHIFT_HMI, OUTPUT);
  pinMode(PIN_TVIS_OUTPUT, OUTPUT);
  pinMode(PIN_FAN_OUTPUT, OUTPUT);
  pinMode(PIN_CHECK_ENGINE, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_IG_INPUT), onIgnitionPulse, FALLING);
  Serial.begin(115200);

  // Initialize 20x4 I2C Cockpit Display (SDA=A4, SCL=A5)
  lcd.init();
  lcd.createChar(1, bar1); // 1 pixel column wide
  lcd.createChar(2, bar2); // 2 pixel columns wide
  lcd.createChar(3, bar3); // 3 pixel columns wide
  lcd.createChar(4, bar4); // 4 pixel columns wide
  lcd.createChar(5, bar5); // 5 pixel columns wide (full block)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("====================");
  lcd.setCursor(0, 1);
  lcd.print("  THE HACHI (AE86)  ");
  lcd.setCursor(0, 2);
  lcd.print(" 4A-GE TELEMETRY OS ");
  lcd.setCursor(0, 3);
  lcd.print("====================");
  delay(800);
  lcd.clear();
}

void loop() {
  static unsigned long lastLoopMs = 0;
  static unsigned long lastLcdMs  = 0;
  unsigned long currentMillis = millis();

  // 1. Listen for Host Serial Reset Commands ('R')
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      sensorFaultLatched = false;
      activeDtcCode = 0;
    }
  }

  // 2. High-Speed Physics & Control Loop (Every 50ms / 20 Hz)
  if (currentMillis - lastLoopMs >= 50) {
    lastLoopMs = currentMillis;

    unsigned long interval;
    noInterrupts();
    interval = pulseIntervalMicros;
    interrupts();

    int calculatedRpm = 0;
    if (interval > 1000 && interval < 100000) {
      calculatedRpm = 30000000UL / interval;
    }

    int rawTps = analogRead(PIN_TPS_INPUT);
    int rawThw = analogRead(PIN_THW_INPUT);
    int throttlePct = map(rawTps, 0, 1023, 0, 100);

    // Convert NTC Thermistor ADC reading to degrees F
    int coolantF = map(constrain(rawThw, 180, 660), 660, 180, 165, 225);

    // 0.2V - 4.8V Window Fault Check
    if (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID) {
      sensorFaultLatched = true;
      activeDtcCode = 4;
    }

    // Closed-Loop Fan & Fail-Safe Override
    if (sensorFaultLatched) {
      digitalWrite(PIN_FAN_OUTPUT, HIGH);
      bool blink = ((currentMillis / 250) % 2) == 0;
      digitalWrite(PIN_CHECK_ENGINE, blink);
    } else {
      digitalWrite(PIN_CHECK_ENGINE, LOW);
      if (rawThw < 460) {
        digitalWrite(PIN_FAN_OUTPUT, HIGH);
      } else if (rawThw > 540) {
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

    // Shift-Light HMI Output (Pin D7)
    bool shiftAlert = (calculatedRpm >= SHIFT_LIGHT_RPM);
    digitalWrite(PIN_SHIFT_HMI, shiftAlert ? HIGH : LOW);

    // 3. Emit Structured $TELEM Sentence for Python on COM3
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

    // 4. Update 20x4 I2C Cockpit Display (Every 150ms)
    if (currentMillis - lastLcdMs >= 50) {
      lastLcdMs = currentMillis;

	  // ROW 0: "RPM: 4450 [|||||   ]" (40-step sub-pixel sweep across 8 cells)
      lcd.setCursor(0, 0);
      lcd.print("RPM: ");
      printPadded4(calculatedRpm);
      lcd.print(" [");
      int totalPixels = map(constrain(calculatedRpm, 850, 7500), 850, 7500, 0, 40);
      for (int i = 0; i < 8; i++) {
        int cellPixels = constrain(totalPixels - (i * 5), 0, 5);
        if (cellPixels == 0) {
          lcd.print(" ");
        } else {
          lcd.write(cellPixels); // Prints custom CGRAM slot 1, 2, 3, 4, or 5
        }
      }
      lcd.print("]");

      // ROW 1: "TPS:  62%  TVIS:OPEN" (20 chars)
      lcd.setCursor(0, 1);
      lcd.print("TPS: ");
      printPadded3(throttlePct);
      lcd.print("%  TVIS:");
      lcd.print(tvisState ? "OPEN" : "SHUT");

      // ROW 2: "THW: 194F  FAN : OFF" (20 chars)
      lcd.setCursor(0, 2);
      lcd.print("THW: ");
      if (sensorFaultLatched) {
        lcd.print("ERR!  ");
      } else {
        printPadded3(coolantF);
        lcd.print("F  ");
      }
      lcd.print("FAN : ");
      lcd.print(digitalRead(PIN_FAN_OUTPUT) ? "ON " : "OFF");

      // ROW 3: Diagnostic Status & Shift Alert Banner (20 chars)
      lcd.setCursor(0, 3);
      if (sensorFaultLatched) {
        lcd.print("ALERT: DTC CODE 04! ");
      } else if (shiftAlert) {
        lcd.print("*** SHIFT! 7200+ ***");
      } else {
        lcd.print("ECU: SYSTEM HEALTHY ");
      }
    }
  }
}