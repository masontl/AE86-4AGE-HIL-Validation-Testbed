// ====================================================================
// MCU #2: Hachi 20x4 I2C Cockpit OS - Lesson 7 Fixed FIFO Black-Box
// SimulIDE HIL Test Bench - Runs on MCU #2 (SDA=A4, SCL=A5 @ Addr 39)
// ====================================================================

#include <Wire.h>
#include <EEPROM.h>
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

// Dyno Pull Timer Variables
unsigned long pullStartTimeMs = 0;
float lastPullTimeSec = 0.0;
bool pullInProgress = false;
int allTimePeakRpm = 0;

// Compact 6-Byte Flight-Data Record (30 slots x 6 bytes = 180 bytes of SRAM)
struct TelemetryRecord {
  uint16_t rpm;
  uint8_t  tps;
  uint16_t rawThw;
  uint8_t  flags; // Bit 0: TVIS, Bit 1: Fan, Bit 2: Shift, Bit 3: DTC Active
};

const int RING_BUFFER_SIZE = 30;
TelemetryRecord ringBuffer[RING_BUFFER_SIZE];
int ringHead = 0;
unsigned long lastRingSampleMs = 0;

// EEPROM Map: Total 187 Bytes (Addresses 0..186)
const int EEPROM_ADDR_MAGIC   = 0; // 0x42 = Valid snapshot exists
const int EEPROM_ADDR_DTC     = 1; // Stored DTC code (4)
const int EEPROM_ADDR_PEAK_H  = 2; // All-time Peak RPM (High Byte)
const int EEPROM_ADDR_PEAK_L  = 3; // All-time Peak RPM (Low Byte)
const int EEPROM_ADDR_RPM_H   = 4; // Fault Trigger RPM (High Byte)
const int EEPROM_ADDR_RPM_L   = 5; // Fault Trigger RPM (Low Byte)
const int EEPROM_ADDR_TPS     = 6; // Fault Trigger TPS %
const int EEPROM_ADDR_HISTORY = 7; // 30 records x 6 bytes = 180 bytes (Addr 7..186)
const int TOTAL_EEPROM_BYTES  = 187;

// Non-Blocking EEPROM Writer State Machine
bool eepromDumpNeeded = false;
int eepromWriteIndex  = 0;
byte eepromStaging[TOTAL_EEPROM_BYTES];

bool tvisState = false;
bool sensorFaultLatched = false;
bool freezeFrameSaved = false;
int activeDtcCode = 0;

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

  if (EEPROM.read(EEPROM_ADDR_MAGIC) == 0x42) {
    allTimePeakRpm = (EEPROM.read(EEPROM_ADDR_PEAK_H) << 8) | EEPROM.read(EEPROM_ADDR_PEAK_L);
  }

  // Pre-fill SRAM Ring Buffer so early faults (<6s) still dump a clean 30-row table
  for (int i = 0; i < RING_BUFFER_SIZE; i++) {
    ringBuffer[i].rpm    = 0;
    ringBuffer[i].tps    = 0;
    ringBuffer[i].rawThw = 500;
    ringBuffer[i].flags  = 0;
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  THE HACHI (AE86)  ");
  lcd.setCursor(0, 2); lcd.print(" FIFO BLACK-BOX V2  ");
  lcd.setCursor(0, 3); lcd.print("====================");
  delay(800);
  lcd.clear();
}

void loop() {
  static unsigned long lastLoopMs = 0;
  static unsigned long lastLcdMs  = 0;
  unsigned long currentMillis = millis();

  // 0. Fast Background EEPROM Writer (1 byte per main loop pass -> finishes 187 bytes in ~0.6s!)
  if (eepromDumpNeeded) {
    EEPROM.update(eepromWriteIndex, eepromStaging[eepromWriteIndex]);
    eepromWriteIndex++;
    if (eepromWriteIndex >= TOTAL_EEPROM_BYTES) {
      eepromDumpNeeded = false;
    }
  }

  // 1. Process Host Serial Commands from Python ('R', 'D', 'C')
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      sensorFaultLatched = false;
      activeDtcCode = 0;
      freezeFrameSaved = false; // Re-arm FIFO capture for the next fault!
    } else if (cmd == 'D' || cmd == 'd') {
      // Flush any remaining staged EEPROM bytes immediately before dumping
      while (eepromDumpNeeded) {
        EEPROM.update(eepromWriteIndex, eepromStaging[eepromWriteIndex]);
        eepromWriteIndex++;
        if (eepromWriteIndex >= TOTAL_EEPROM_BYTES) {
          eepromDumpNeeded = false;
        }
      }

      if (EEPROM.read(EEPROM_ADDR_MAGIC) == 0x42) {
        int fDtc  = EEPROM.read(EEPROM_ADDR_DTC);
        int fPeak = (EEPROM.read(EEPROM_ADDR_PEAK_H) << 8) | EEPROM.read(EEPROM_ADDR_PEAK_L);
        int fRpm  = (EEPROM.read(EEPROM_ADDR_RPM_H) << 8) | EEPROM.read(EEPROM_ADDR_RPM_L);
        int fTps  = EEPROM.read(EEPROM_ADDR_TPS);

        Serial.print("$FREEZE,");
        Serial.print(fDtc);  Serial.print(",");
        Serial.print(fRpm);  Serial.print(",");
        Serial.print(fTps);  Serial.print(",");
        Serial.print(fPeak); Serial.println("*");

        // Stream all 30 pre-fault FIFO records chronologically (Oldest -> Newest)
        for (int i = 0; i < RING_BUFFER_SIZE; i++) {
          int addr = EEPROM_ADDR_HISTORY + (i * 6);
          uint16_t rRpm = ((uint16_t)EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
          uint8_t  rTps = EEPROM.read(addr + 2);
          uint16_t rThw = ((uint16_t)EEPROM.read(addr + 3) << 8) | EEPROM.read(addr + 4);
          uint8_t  rFlg = EEPROM.read(addr + 5);

          Serial.print("$HIST,");
          Serial.print(i - 29); // Sample index (-29 = oldest, 0 = fault trigger moment)
          Serial.print(",");
          Serial.print(rRpm);   Serial.print(",");
          Serial.print(rTps);   Serial.print(",");
          Serial.print(rThw);   Serial.print(",");
          Serial.print((rFlg & 0x01) ? 1 : 0); Serial.print(",");
          Serial.print((rFlg & 0x02) ? 1 : 0); Serial.print(",");
          Serial.print((rFlg & 0x08) ? 4 : 0); Serial.println("*");
        }
      } else {
        Serial.println("$FREEZE,0,0,0,0*");
      }
    } else if (cmd == 'C' || cmd == 'c') {
      EEPROM.update(EEPROM_ADDR_MAGIC, 0x00);
      allTimePeakRpm = 0;
      freezeFrameSaved = false;
      eepromDumpNeeded = false;
      Serial.println("[CMD] EEPROM Black-Box & History Cleared.");
    }
  }

  // 2. High-Speed Physics & Control Loop (Every 50ms)
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

    if (calculatedRpm > allTimePeakRpm) {
      allTimePeakRpm = calculatedRpm;
      EEPROM.update(EEPROM_ADDR_PEAK_H, (allTimePeakRpm >> 8) & 0xFF);
      EEPROM.update(EEPROM_ADDR_PEAK_L, allTimePeakRpm & 0xFF);
    }

    // Dyno Pull Timer (2000-7500 RPM)
    if (calculatedRpm >= 2000 && calculatedRpm < 7500) {
      if (!pullInProgress && calculatedRpm <= 2200) {
        pullInProgress = true;
        pullStartTimeMs = currentMillis;
      }
    } else if (calculatedRpm >= 7500 && pullInProgress) {
      pullInProgress = false;
      lastPullTimeSec = (currentMillis - pullStartTimeMs) / 1000.0;
    } else if (calculatedRpm < 2000) {
      pullInProgress = false;
    }

    int rawTps = analogRead(PIN_TPS_INPUT);
    int rawThw = analogRead(PIN_THW_INPUT);
    int throttlePct = map(rawTps, 0, 1023, 0, 100);
    int coolantF = map(constrain(rawThw, 180, 660), 660, 180, 165, 225);

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

    // 3. Continuous 5 Hz (200ms) SRAM Ring Buffer Sampler (Infinite Endurance)
    // Captures every 200ms (30 samples = 6.0 seconds of high-res history)
    if (currentMillis - lastRingSampleMs >= 200) {
      lastRingSampleMs = currentMillis;

      uint8_t flags = 0;
      if (tvisState)                              flags |= 0x01;
      if (digitalRead(PIN_FAN_OUTPUT))            flags |= 0x02;
      if (shiftAlert)                             flags |= 0x04;
      if (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID) flags |= 0x08;

      ringBuffer[ringHead].rpm    = calculatedRpm;
      ringBuffer[ringHead].tps    = throttlePct;
      ringBuffer[ringHead].rawThw = rawThw;
      ringBuffer[ringHead].flags  = flags;

      ringHead = (ringHead + 1) % RING_BUFFER_SIZE;
    }

    // 4. Fault Detection & Pre-Trigger FIFO Lock
    if (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID) {
      sensorFaultLatched = true;
      activeDtcCode = 4;

      if (!freezeFrameSaved) {
        freezeFrameSaved = true;

        // Guarantee the exact fault-trigger frame is written into the final slot before locking!
        uint8_t faultFlags = 0x08 | 0x02; // DTC=4 + Fan=ON
        if (tvisState)  faultFlags |= 0x01;
        if (shiftAlert) faultFlags |= 0x04;

        ringBuffer[ringHead].rpm    = calculatedRpm;
        ringBuffer[ringHead].tps    = throttlePct;
        ringBuffer[ringHead].rawThw = rawThw;
        ringBuffer[ringHead].flags  = faultFlags;
        ringHead = (ringHead + 1) % RING_BUFFER_SIZE;

        // Stage Header Metadata (Bytes 0..6)
        eepromStaging[0] = 0x42;
        eepromStaging[1] = activeDtcCode;
        eepromStaging[2] = (allTimePeakRpm >> 8) & 0xFF;
        eepromStaging[3] = allTimePeakRpm & 0xFF;
        eepromStaging[4] = (calculatedRpm >> 8) & 0xFF;
        eepromStaging[5] = calculatedRpm & 0xFF;
        eepromStaging[6] = throttlePct;

        // Unroll circular buffer from Oldest (Sample -29) to Newest (Sample 0 = Fault Trigger)
        for (int i = 0; i < RING_BUFFER_SIZE; i++) {
          int srcIdx = (ringHead + i) % RING_BUFFER_SIZE;
          int base = EEPROM_ADDR_HISTORY + (i * 6);
          eepromStaging[base + 0] = (ringBuffer[srcIdx].rpm >> 8) & 0xFF;
          eepromStaging[base + 1] = ringBuffer[srcIdx].rpm & 0xFF;
          eepromStaging[base + 2] = ringBuffer[srcIdx].tps;
          eepromStaging[base + 3] = (ringBuffer[srcIdx].rawThw >> 8) & 0xFF;
          eepromStaging[base + 4] = ringBuffer[srcIdx].rawThw & 0xFF;
          eepromStaging[base + 5] = ringBuffer[srcIdx].flags;
        }

        // Arm fast non-blocking EEPROM writer
        eepromWriteIndex = 0;
        eepromDumpNeeded = true;
      }
    }

    // Closed-Loop Fan & Failsafe
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

    // 5. Emit Live $TELEM Sentence
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

    // 6. Update 20x4 I2C Cockpit Screen (Every 100ms)
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
      if (sensorFaultLatched) {
        lcd.print("ERR!  ");
      } else {
        printPadded3(coolantF);
        lcd.print("F  ");
      }
      lcd.print("FAN : ");
      lcd.print(digitalRead(PIN_FAN_OUTPUT) ? "ON " : "OFF");

      lcd.setCursor(0, 3);
      if (sensorFaultLatched) {
        lcd.print("ALERT: DTC CODE 04! ");
      } else if (shiftAlert) {
        lcd.print("*** SHIFT! 7200+ ***");
      } else if (pullInProgress) {
        lcd.print("DYNO PULL LOGGING...");
      } else {
        lcd.print("PULL:");
        if (lastPullTimeSec > 0.0) {
          lcd.print(lastPullTimeSec, 2);
          lcd.print("s ");
        } else {
          lcd.print("---s ");
        }
        lcd.print("PK:");
        printPadded4(allTimePeakRpm);
      }
    }
  }
}