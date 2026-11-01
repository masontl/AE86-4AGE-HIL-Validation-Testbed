// ====================================================================
// LESSON 9 — MCU #2: Hachi 20x4 I2C Cockpit OS + Continuous 20Hz TF/SD Logger
// SimulIDE HIL Test Bench - Runs on MCU #2 (SDA=A4, SCL=A5 @ Addr 39)
// Architecture:
//   - Always-On Continuous 20Hz (50ms) Telemetry Stream to External TF/SD Card
//   - Decoupled from offline analysis (True UAV / Vehicle R&D Flight Recorder)
// ====================================================================

#include <Wire.h>
#include <LiquidCrystal_AIP31068_I2C.h>

LiquidCrystal_AIP31068_I2C lcd(39, 20, 4);

const int PIN_IG_INPUT     = 2;   // Input:  Interrupt 0 reads 4A-GE IG- pulses
const int PIN_TPS_INPUT    = A0;  // Input:  Analog VTA throttle position
const int PIN_THW_INPUT    = A1;  // Input:  Analog THW coolant temp
const int PIN_SHIFT_HMI    = 7;   // Output: Driver Shift-Light / Acoustic Cue
const int PIN_TVIS_OUTPUT  = 8;   // Output: T-VIS VSV Relay Status Indicator
const int PIN_FAN_OUTPUT   = 9;   // Output: Electric Radiator Fan Driver
const int PIN_CHECK_ENGINE = 13;  // Output: Check Engine Light (Code 4)

volatile unsigned long lastEdgeTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;

// Operating Thresholds
const int TVIS_ACTIVATE_RPM   = 4400;
const int TVIS_HYSTERESIS_RPM = 150;
const int SHIFT_LIGHT_RPM     = 7200;
const int MIN_THROTTLE_RAW    = 200;
const int SENSOR_MIN_VALID    = 41;
const int SENSOR_MAX_VALID    = 982;

bool tvisState          = false;
bool sensorFaultLatched = false;
int  activeDtcCode      = 0;
unsigned long sdFramesWritten = 0;

// Custom 5x8 CGRAM Bitmaps for 40-Step Sub-Pixel Bar Graph
byte bar1[8] = {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000};
byte bar2[8] = {0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000};
byte bar3[8] = {0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100};
byte bar4[8] = {0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110};
byte bar5[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};

void onIgnitionPulse() {
  unsigned long now = micros();
  pulseIntervalMicros = now - lastEdgeTimeMicros;
  lastEdgeTimeMicros = now;
}

void printPadded4(int val) {
  if (val < 1000) lcd.print(" ");
  if (val < 100)  lcd.print(" ");
  if (val < 10)   lcd.print(" ");
  lcd.print(val);
}

void printPadded3(int val) {
  if (val < 100) lcd.print(" ");
  if (val < 10)  lcd.print(" ");
  lcd.print(val);
}

void setup() {
  pinMode(PIN_IG_INPUT, INPUT_PULLUP);
  pinMode(PIN_SHIFT_HMI, OUTPUT);
  pinMode(PIN_TVIS_OUTPUT, OUTPUT);
  pinMode(PIN_FAN_OUTPUT, OUTPUT);
  pinMode(PIN_CHECK_ENGINE, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_IG_INPUT), onIgnitionPulse, FALLING);
  Serial.begin(115200);

  lcd.init();
  lcd.createChar(1, bar1);
  lcd.createChar(2, bar2);
  lcd.createChar(3, bar3);
  lcd.createChar(4, bar4);
  lcd.createChar(5, bar5);

  // 800ms Boot Splash Screen (Charges 10uF RC Filter Capacitors prior to logging)
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  THE HACHI (AE86)  ");
  lcd.setCursor(0, 2); lcd.print(" 20Hz TF/SD LOGGER  ");
  lcd.setCursor(0, 3); lcd.print("====================");
  delay(800);
  lcd.clear();

  // Announce New Mission Header to Virtual TF/SD Card
  Serial.println("$SD_MOUNT,HACHI_AE86_FDR_V1,512MB*");
}

void loop() {
  static unsigned long lastLoopMs = 0;
  static unsigned long lastLcdMs  = 0;
  unsigned long currentMillis = millis();

  // Optional Serial Command 'R' to clear latched dashboard warning light
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      sensorFaultLatched = false;
      activeDtcCode      = 0;
    }
  }

  // 1. High-Speed 20Hz (50ms) Acquisition, Control & Continuous SD Card Stream
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

    int rawTps      = analogRead(PIN_TPS_INPUT);
    int rawThw      = analogRead(PIN_THW_INPUT);
    int throttlePct = map(rawTps, 0, 1023, 0, 100);
    int coolantF    = map(constrain(rawThw, 180, 660), 660, 180, 165, 225);

    // T-VIS Control
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

    // Shift Light HMI
    bool shiftAlert = (calculatedRpm >= SHIFT_LIGHT_RPM);
    digitalWrite(PIN_SHIFT_HMI, shiftAlert ? HIGH : LOW);

    // Fault Detection & Failsafe Fan Control
    bool liveFault = (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID);
    if (liveFault) {
      sensorFaultLatched = true;
      activeDtcCode      = 4;
    }

    if (sensorFaultLatched) {
      digitalWrite(PIN_FAN_OUTPUT, HIGH); // Failsafe: Force Fan 100% ON
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

    // 2. Always-On Continuous 20Hz Write to External TF/SD Card Port
    // Format: $TELEM,SimTime_ms,RPM,TPS_Pct,Raw_THW,Coolant_F,TVIS,Fan,Live_Fault,Latched_DTC*
    sdFramesWritten++;
    Serial.print("$TELEM,");
    Serial.print(currentMillis);               Serial.print(",");
    Serial.print(calculatedRpm);               Serial.print(",");
    Serial.print(throttlePct);                 Serial.print(",");
    Serial.print(rawThw);                      Serial.print(",");
    Serial.print(liveFault ? -99 : coolantF);  Serial.print(",");
    Serial.print(tvisState ? 1 : 0);           Serial.print(",");
    Serial.print(digitalRead(PIN_FAN_OUTPUT)); Serial.print(",");
    Serial.print(liveFault ? 4 : 0);           Serial.print(",");
    Serial.print(activeDtcCode);               Serial.println("*");

    // 3. Update 20x4 I2C Cockpit Screen (Every 100ms)
    if (currentMillis - lastLcdMs >= 100) {
      lastLcdMs = currentMillis;

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
          lcd.write(cellPixels);
        }
      }
      lcd.print("]");

      lcd.setCursor(0, 1);
      lcd.print("TPS: ");
      printPadded3(throttlePct);
      lcd.print("%  TVIS:");
      lcd.print(tvisState ? "OPEN" : "SHUT");

      lcd.setCursor(0, 2);
      lcd.print("THW: ");
      if (liveFault) {
        lcd.print("ERR!  ");
      } else {
        printPadded3(coolantF);
        lcd.print("F  ");
      }
      lcd.print("FAN : ");
      lcd.print(digitalRead(PIN_FAN_OUTPUT) ? "ON " : "OFF");

      // Row 3: Live TF/SD Card Recording Counter & DTC Alert
      lcd.setCursor(0, 3);
      if (sensorFaultLatched) {
        lcd.print("DTC:04 SD-REC:");
        printPadded4((int)(sdFramesWritten % 10000));
        lcd.print(" ");
      } else if (shiftAlert) {
        lcd.print("*** SHIFT! 7200+ ***");
      } else {
        lcd.print("SD:REC 20Hz #");
        printPadded4((int)(sdFramesWritten % 10000));
        lcd.print("  ");
      }
    }
  }
}