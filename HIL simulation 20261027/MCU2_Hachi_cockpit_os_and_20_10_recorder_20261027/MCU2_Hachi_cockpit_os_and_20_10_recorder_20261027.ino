// ====================================================================
// MCU #2: Hachi 20x4 I2C Cockpit OS - Lesson 8 (60-Slot / 30s @ 2Hz Black-Box)
// SimulIDE HIL Test Bench - Runs on MCU #2 (SDA=A4, SCL=A5 @ Addr 39)
// Architecture:
//   - 100% FAULT-ONLY EEPROM BLACK-BOX (367 Bytes out of 1024B EEPROM)
//   - 60-Slot SRAM Ring Buffer @ 2 Hz (500ms/step = 30.0 Seconds Total):
//       * 40 Pre/Trigger Slots (Samples -39..0  = 20.0 Seconds Pre-Fault)
//       * 20 Post-Trigger Slots (Samples +1..+20 = 10.0 Seconds Post-Fault)
//   - Safe early-fault handling (<30s uptime) & instant flush on 'D' query
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

// Tracked purely in SRAM during normal driving; only saved to EEPROM if a fault occurs!
int allTimePeakRpm = 0;

// Compact 6-Byte Flight-Data Record (60 slots x 6 bytes = 360 bytes of SRAM)
struct TelemetryRecord {
  uint16_t rpm;
  uint8_t  tps;
  uint16_t rawThw;
  uint8_t  flags; // Bit 0: TVIS, Bit 1: Fan, Bit 2: Shift, Bit 3: DTC Active
};

// 2 Hz (500ms) x 60 Slots = 30.0 Seconds Total (20.0s Pre + 10.0s Post)
const int RING_BUFFER_SIZE      = 60;
const int PRE_TRIGGER_ZERO_IDX  = 39;  // Slot 39 = Sample 0 (0.0s); Slots 0..38 = -19.5s..-0.5s
const int POST_TRIGGER_SLOTS    = 20;  // 20 Post-Trigger samples @ 500ms = +10.0 seconds
const unsigned long RING_STEP_MS = 500; // 2 Hz sampling interval

TelemetryRecord ringBuffer[RING_BUFFER_SIZE];
int ringHead = 0;
unsigned long lastRingSampleMs = 0;

// Lesson 8 Asymmetric Capture State Variables
bool postCaptureActive    = false;
int  postTriggerRemaining = 0;
int  triggerRpmSnapshot   = 0;
int  triggerTpsSnapshot   = 0;

// EEPROM Map: Total 367 Bytes (Addresses 0..366 out of 1024 Bytes available)
const int EEPROM_ADDR_MAGIC   = 0; // 0x43 = Valid 60-slot (30s @ 2Hz) snapshot exists
const int EEPROM_ADDR_DTC     = 1; // Stored DTC code (4)
const int EEPROM_ADDR_PEAK_H  = 2; // Session Peak RPM prior to fault (High Byte)
const int EEPROM_ADDR_PEAK_L  = 3; // Session Peak RPM prior to fault (Low Byte)
const int EEPROM_ADDR_RPM_H   = 4; // Fault Trigger RPM at t=0.0s (High Byte)
const int EEPROM_ADDR_RPM_L   = 5; // Fault Trigger RPM at t=0.0s (Low Byte)
const int EEPROM_ADDR_TPS     = 6; // Fault Trigger TPS % at t=0.0s
const int EEPROM_ADDR_HISTORY = 7; // 60 records x 6 bytes = 360 bytes (Addr 7..366)
const int TOTAL_EEPROM_BYTES  = 367;

// Non-Blocking EEPROM Writer State Machine
bool eepromDumpNeeded = false;
int  eepromWriteIndex = 0;
byte eepromStaging[TOTAL_EEPROM_BYTES];

bool tvisState = false;
bool sensorFaultLatched = false;
bool freezeFrameSaved = false;
int  activeDtcCode = 0;

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

// Stages the 60-Slot Asymmetric Window (40 Pre/Trig [20s] + 20 Post [10s]) for Background EEPROM Commit
void stageAsymmetricWindowToEeprom() {
  eepromStaging[0] = 0x43;
  eepromStaging[1] = activeDtcCode;
  eepromStaging[2] = (allTimePeakRpm >> 8) & 0xFF;
  eepromStaging[3] = allTimePeakRpm & 0xFF;
  eepromStaging[4] = (triggerRpmSnapshot >> 8) & 0xFF;
  eepromStaging[5] = triggerRpmSnapshot & 0xFF;
  eepromStaging[6] = triggerTpsSnapshot;

  // Unroll circular buffer from Oldest (Sample -39) through Trigger (Sample 0) to Newest (Sample +20)
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

  eepromWriteIndex = 0;
  eepromDumpNeeded = true;
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

  // If a prior un-cleared fault is stored in EEPROM across power cycles, restore its latch state
  if (EEPROM.read(EEPROM_ADDR_MAGIC) == 0x43) {
    activeDtcCode      = EEPROM.read(EEPROM_ADDR_DTC);
    allTimePeakRpm     = (EEPROM.read(EEPROM_ADDR_PEAK_H) << 8) | EEPROM.read(EEPROM_ADDR_PEAK_L);
    sensorFaultLatched = true;
    freezeFrameSaved   = true;
  }

  // Pre-fill all 60 SRAM Ring Buffer slots with clean pre-boot markers (RPM=0)
  // so early faults (<20s after boot) still dump a complete, hang-free 60-row table!
  for (int i = 0; i < RING_BUFFER_SIZE; i++) {
    ringBuffer[i].rpm    = 0;
    ringBuffer[i].tps    = 0;
    ringBuffer[i].rawThw = 500;
    ringBuffer[i].flags  = 0;
  }

  // 800ms Boot Splash Screen (Allows 10uF RC Filter Capacitors to charge to normal voltage)
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  THE HACHI (AE86)  ");
  lcd.setCursor(0, 2); lcd.print(" 30s 2Hz BLACK-BOX  ");
  lcd.setCursor(0, 3); lcd.print("====================");
  delay(800);
  lcd.clear();
}

void loop() {
  static unsigned long lastLoopMs = 0;
  static unsigned long lastLcdMs  = 0;
  unsigned long currentMillis = millis();

  // 0. Fast Background EEPROM Writer (1 byte per main loop pass -> ONLY active after a fault!)
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
      sensorFaultLatched   = false;
      activeDtcCode        = 0;
      freezeFrameSaved     = false; // Re-arm 60-slot FIFO capture for the next fault!
      postCaptureActive    = false;
      postTriggerRemaining = 0;
    } else if (cmd == 'D' || cmd == 'd') {
      // If Python queries 'D' while the 10-second post-capture is still counting down,
      // stage immediately so Mode A never hangs or misses a fault!
      if (postCaptureActive) {
        postCaptureActive    = false;
        postTriggerRemaining = 0;
        stageAsymmetricWindowToEeprom();
      }
      // Flush any remaining staged EEPROM bytes immediately before dumping
      while (eepromDumpNeeded) {
        EEPROM.update(eepromWriteIndex, eepromStaging[eepromWriteIndex]);
        eepromWriteIndex++;
        if (eepromWriteIndex >= TOTAL_EEPROM_BYTES) {
          eepromDumpNeeded = false;
        }
      }

      if (EEPROM.read(EEPROM_ADDR_MAGIC) == 0x43) {
        int fDtc  = EEPROM.read(EEPROM_ADDR_DTC);
        int fPeak = (EEPROM.read(EEPROM_ADDR_PEAK_H) << 8) | EEPROM.read(EEPROM_ADDR_PEAK_L);
        int fRpm  = (EEPROM.read(EEPROM_ADDR_RPM_H) << 8) | EEPROM.read(EEPROM_ADDR_RPM_L);
        int fTps  = EEPROM.read(EEPROM_ADDR_TPS);

        Serial.print("$FREEZE,");
        Serial.print(fDtc);  Serial.print(",");
        Serial.print(fRpm);  Serial.print(",");
        Serial.print(fTps);  Serial.print(",");
        Serial.print(fPeak); Serial.println("*");

        // Stream all 60 Asymmetric FIFO records chronologically (500ms / 2Hz per step):
        //   i = 0..38 -> Sample -39..-1 (Pre-Trigger History: -19.5s to -0.5s)
        //   i = 39    -> Sample   0     (Exact Fault Trigger Moment: 0.0s)
        //   i = 40..59-> Sample +1..+20 (Post-Trigger Failsafe Window: +0.5s to +10.0s)
        for (int i = 0; i < RING_BUFFER_SIZE; i++) {
          int addr = EEPROM_ADDR_HISTORY + (i * 6);
          uint16_t rRpm = ((uint16_t)EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
          uint8_t  rTps = EEPROM.read(addr + 2);
          uint16_t rThw = ((uint16_t)EEPROM.read(addr + 3) << 8) | EEPROM.read(addr + 4);
          uint8_t  rFlg = EEPROM.read(addr + 5);

          Serial.print("$HIST,");
          Serial.print(i - PRE_TRIGGER_ZERO_IDX); // Sample index (-39 = oldest, 0 = trigger, +20 = newest)
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
      allTimePeakRpm       = 0;
      freezeFrameSaved     = false;
      postCaptureActive    = false;
      postTriggerRemaining = 0;
      eepromDumpNeeded     = false;
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

    // Track Peak RPM in fast SRAM ONLY (Zero EEPROM wear during normal driving!)
    if (calculatedRpm > allTimePeakRpm) {
      allTimePeakRpm = calculatedRpm;
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

    // 3. Fault Detection & Lesson 8 Asymmetric 20s Pre / 10s Post Trigger
    if (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID) {
      sensorFaultLatched = true;
      activeDtcCode = 4;

      // Immediately engage Fan Failsafe BEFORE recording the trigger slot!
      digitalWrite(PIN_FAN_OUTPUT, HIGH);

      if (!freezeFrameSaved) {
        freezeFrameSaved   = true;
        triggerRpmSnapshot = calculatedRpm;
        triggerTpsSnapshot = throttlePct;

        // Write the exact Fault Trigger Frame (Sample 0, t = 0.0s) into the ring buffer right now
        uint8_t faultFlags = 0x08 | 0x02; // DTC=4 + Fan=ON
        if (tvisState)  faultFlags |= 0x01;
        if (shiftAlert) faultFlags |= 0x04;

        ringBuffer[ringHead].rpm    = calculatedRpm;
        ringBuffer[ringHead].tps    = throttlePct;
        ringBuffer[ringHead].rawThw = rawThw;
        ringBuffer[ringHead].flags  = faultFlags;
        ringHead = (ringHead + 1) % RING_BUFFER_SIZE;

        // Arm the 20-Slot (10.0-Second @ 2Hz) Post-Trigger Countdown
        postCaptureActive    = true;
        postTriggerRemaining = POST_TRIGGER_SLOTS; // 20 samples = 10.0 seconds remaining
        lastRingSampleMs     = currentMillis;      // Sync 500ms timer to exact trigger edge
      }
    }

    // Closed-Loop Fan & Check Engine Light Failsafe
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

    // 4. Continuous 2 Hz (500ms) SRAM Ring Buffer Sampler (60 slots = 30.0s window)
    if (currentMillis - lastRingSampleMs >= RING_STEP_MS) {
      lastRingSampleMs = currentMillis;

      if (!freezeFrameSaved || postCaptureActive) {
        uint8_t flags = 0;
        if (tvisState)                              flags |= 0x01;
        if (digitalRead(PIN_FAN_OUTPUT))            flags |= 0x02;
        if (shiftAlert)                             flags |= 0x04;
        if (sensorFaultLatched || rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID) flags |= 0x08;

        ringBuffer[ringHead].rpm    = calculatedRpm;
        ringBuffer[ringHead].tps    = throttlePct;
        ringBuffer[ringHead].rawThw = rawThw;
        ringBuffer[ringHead].flags  = flags;

        ringHead = (ringHead + 1) % RING_BUFFER_SIZE;

        // If counting down the 20 Post-Trigger slots (+0.5s to +10.0s):
        if (postCaptureActive) {
          postTriggerRemaining--;
          if (postTriggerRemaining <= 0) {
            postCaptureActive = false;
            stageAsymmetricWindowToEeprom(); // All 40 Pre (20s) + 20 Post (10s) slots captured!
          }
        }
      }
    }

    // 5. Emit Live $TELEM Sentence (7-field format)
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

      // Row 3: Dedicated Diagnostic & 30s (20s/10s) Black-Box Recorder Status
      lcd.setCursor(0, 3);
      if (sensorFaultLatched) {
        if (postCaptureActive) {
          int remSec = (postTriggerRemaining + 1) / 2; // Convert 2Hz slots to seconds (+10s..+1s)
          lcd.print("DTC C04! POST:+");
          if (remSec < 10) lcd.print("0");
          lcd.print(remSec);
          lcd.print("s  ");
        } else if (eepromDumpNeeded) {
          lcd.print("DTC C04! [SAVING...]");
        } else {
          lcd.print("DTC C04! [SAVED E2P]");
        }
      } else if (shiftAlert) {
        lcd.print("*** SHIFT! 7200+ ***");
      } else {
        lcd.print("SYS:OK 20s/10s ARMED");
      }
    }
  }
}