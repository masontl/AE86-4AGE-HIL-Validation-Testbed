// ====================================================================
// LESSON 14 (SPI SD EDITION) — MCU #2: Hachi Cockpit OS + XOR Checksum Framing
// SimulIDE HIL Test Bench - Runs on MCU #2 (SDA=A4, SCL=A5 @ Addr 39)
// Logs 14 Channels + Deterministic XOR Checksum (*XX) at 20Hz to 'HACHI14.CSV'
// ====================================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_AIP31068_I2C.h>

LiquidCrystal_AIP31068_I2C lcd(39, 20, 4);

const int PIN_IG_INPUT     = 2;   // Input:  INT0 (D2) reads 4A-GE IG- trigger pulses
const int PIN_IGF_INPUT    = 3;   // Input:  INT1 (D3) reads VAST Igniter IGf confirmation pulses
const int PIN_CHECK_ENGINE = 6;   // Output: Check Engine LED (D13 reserved for SPI SCK)
const int PIN_SHIFT_HMI    = 7;   // Output: Driver Shift-Light / Acoustic Cue
const int PIN_TVIS_OUTPUT  = 8;   // Output: T-VIS VSV Relay Status Indicator
const int PIN_FAN_OUTPUT   = 9;   // Output: Electric Radiator Fan Driver
const int PIN_SD_CS        = 10;  // Output: Hardware SPI SD Card Chip Select (CS)

const int PIN_TPS_INPUT    = A0;  // Input:  Analog VTA throttle position
const int PIN_THW_INPUT    = A1;  // Input:  Analog THW coolant temp
const int PIN_OX_INPUT     = A2;  // Input:  Analog OX Narrowband O2 (0.0V - 1.0V)
const int PIN_VF_INPUT     = A3;  // Input:  Analog VF TCCS Fuel Trim (0.0V - 5.0V)

volatile unsigned long lastEdgeTimeMicros  = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile unsigned int  isrIgMinusCount     = 0;
volatile unsigned int  isrIgfConfirmCount  = 0;

// Operating Thresholds
const int TVIS_ACTIVATE_RPM     = 4400;
const int TVIS_HYSTERESIS_RPM   = 150;
const int SHIFT_LIGHT_RPM       = 7200;
const int MIN_THROTTLE_RAW      = 200;
const int SENSOR_MIN_VALID      = 41;
const int SENSOR_MAX_VALID      = 982;
const int CLOSED_LOOP_MIN_F     = 140; // 1986 4A-GE TCCS Warmup Gate (140°F / 60°C)
const int IGF_FAULT_SPARK_LIMIT = 6;   // Consecutive missed IGf sparks to trigger Code 14 Fuel-Cut

bool tvisState           = false;
bool sensorFaultLatched  = false;
int  activeDtcCode       = 0;
int  missedIgfSparks     = 0;
unsigned long sdFramesWritten = 0;

bool spiSdReady = false;
File logFile;

// Custom 5x8 CGRAM Bitmaps for 40-Step Sub-Pixel Bar Graph
byte bar1[8] = {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000};
byte bar2[8] = {0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000};
byte bar3[8] = {0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100, 0b11100};
byte bar4[8] = {0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110};
byte bar5[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};

// INT0 ISR: Triggered on every 4A-GE IG- primary ignition event
void onIgnitionPulse() {
  unsigned long now = micros();
  pulseIntervalMicros = now - lastEdgeTimeMicros;
  lastEdgeTimeMicros  = now;
  isrIgMinusCount++;
}

// INT1 ISR: Triggered on every VAST Igniter IGf confirmation pulse
void onIgfConfirmPulse() {
  isrIgfConfirmCount++;
}

// LESSON 14: Deterministic 8-bit Bitwise XOR Frame Checksum
uint8_t computeXorChecksum(const char* payload) {
  uint8_t cs = 0;
  while (*payload) {
    cs ^= (uint8_t)(*payload++);
  }
  return cs;
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

void printSignedTrim(int trimPct) {
  if (trimPct >= 0) {
    lcd.print("+");
  } else {
    lcd.print("-");
  }
  int absVal = abs(trimPct);
  if (absVal < 10) lcd.print("0");
  lcd.print(absVal);
  lcd.print("%");
}

void setup() {
  pinMode(PIN_IG_INPUT,     INPUT_PULLUP);
  pinMode(PIN_IGF_INPUT,    INPUT_PULLUP);
  pinMode(PIN_CHECK_ENGINE, OUTPUT);
  pinMode(PIN_SHIFT_HMI,    OUTPUT);
  pinMode(PIN_TVIS_OUTPUT,  OUTPUT);
  pinMode(PIN_FAN_OUTPUT,   OUTPUT);
  pinMode(PIN_SD_CS,        OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_IG_INPUT),  onIgnitionPulse,   FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_IGF_INPUT), onIgfConfirmPulse, FALLING);
  Serial.begin(115200);

  lcd.init();
  lcd.createChar(1, bar1);
  lcd.createChar(2, bar2);
  lcd.createChar(3, bar3);
  lcd.createChar(4, bar4);
  lcd.createChar(5, bar5);

  // Initialize Hardware SPI SD Card on D10 and open HACHI14.CSV (14 Channels + *CS)
  spiSdReady = SD.begin(PIN_SD_CS);
  if (spiSdReady) {
    logFile = SD.open("HACHI14.CSV", FILE_WRITE);
    if (logFile && logFile.size() == 0) {
      logFile.println(F("SimTime_ms,RPM,TPS_Pct,Raw_THW,Coolant_F,TVIS,Fan,Live_DTC,Latched_DTC,OX_mV,VF_mV,Fuel_Trim_Pct,Loop_CL,IGf_OK*XOR_CS"));
      logFile.flush();
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  THE HACHI (AE86)  ");
  lcd.setCursor(0, 2);
  lcd.print(spiSdReady ? " L14 SD+XOR: OK     " : " L14 SD: NOT FOUND  ");
  lcd.setCursor(0, 3); lcd.print("====================");
  delay(800);
  lcd.clear();

  Serial.println(spiSdReady ? "$SD_MOUNT,SPI_FAT16_XOR_OK,HACHI14.CSV*" : "$SD_MOUNT,UART_FALLBACK,HACHI14.CSV*");
}

void loop() {
  static unsigned long lastLoopMs = 0;
  static unsigned long lastLcdMs  = 0;
  unsigned long currentMillis = millis();

  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      sensorFaultLatched = false;
      activeDtcCode      = 0;
      missedIgfSparks    = 0;
    }
  }

  // 1. High-Speed 20Hz (50ms) Acquisition, Dual-ISR Handshake Audit & Framed SD Stream
  if (currentMillis - lastLoopMs >= 50) {
    lastLoopMs = currentMillis;

    unsigned long interval;
    unsigned int  igEdgesSnapshot;
    unsigned int  igfEdgesSnapshot;

    noInterrupts();
    interval           = pulseIntervalMicros;
    igEdgesSnapshot    = isrIgMinusCount;
    igfEdgesSnapshot   = isrIgfConfirmCount;
    isrIgMinusCount    = 0;
    isrIgfConfirmCount = 0;
    interrupts();

    int calculatedRpm = 0;
    if (interval > 1000 && interval < 100000) {
      calculatedRpm = 30000000UL / interval;
    }

    // Dual-Interrupt (INT0 IG- vs. INT1 IGf) Handshake Watchdog
    if (igEdgesSnapshot > 0) {
      if (igfEdgesSnapshot == 0) {
        missedIgfSparks += igEdgesSnapshot;
        if (missedIgfSparks > 250) missedIgfSparks = 250;
      } else {
        missedIgfSparks = 0;
      }
    }

    bool liveIgfFault = (missedIgfSparks >= IGF_FAULT_SPARK_LIMIT);
    int  igfOkFlag    = liveIgfFault ? 0 : 1;

    int rawTps = analogRead(PIN_TPS_INPUT);
    int rawThw = analogRead(PIN_THW_INPUT);
    int rawOx  = analogRead(PIN_OX_INPUT);
    int rawVf  = analogRead(PIN_VF_INPUT);

    int throttlePct = map(rawTps, 0, 1023, 0, 100);
    int coolantF    = map(constrain(rawThw, 140, 923), 923, 140, 60, 230);

    int oxMv = (int)((rawOx * 5000UL) / 1023);
    int vfMv = (int)((rawVf * 5000UL) / 1023);

    // Hard OBD-I Fault Priority Evaluation (Code 14 Igniter vs. Code 4 THW vs. Code 7 TPS)
    bool liveThwFault = (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID);
    bool liveTpsFault = (rawTps < SENSOR_MIN_VALID || rawTps > SENSOR_MAX_VALID);

    int liveDtcVal = 0;
    if (liveIgfFault) {
      liveDtcVal         = 14;
      sensorFaultLatched = true;
      activeDtcCode      = 14;
    } else if (liveThwFault) {
      liveDtcVal         = 4;
      sensorFaultLatched = true;
      if (activeDtcCode == 0) activeDtcCode = 4;
    } else if (liveTpsFault) {
      liveDtcVal         = 7;
      sensorFaultLatched = true;
      if (activeDtcCode == 0) activeDtcCode = 7;
    }

    bool isClosedLoop = (liveDtcVal == 0 && coolantF >= CLOSED_LOOP_MIN_F && vfMv >= 350);
    int  loopClFlag   = isClosedLoop ? 1 : 0;

    int fuelTrimPct = 0;
    if (isClosedLoop) {
      fuelTrimPct = map(constrain(vfMv, 0, 5000), 0, 5000, -20, 20);
    }

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

    // Radiator Fan & Check Engine Light Control
    if (sensorFaultLatched) {
      if (liveThwFault || activeDtcCode == 4) {
        digitalWrite(PIN_FAN_OUTPUT, HIGH);
      }
      bool blink = ((currentMillis / 250) % 2) == 0;
      digitalWrite(PIN_CHECK_ENGINE, blink);
    } else {
      digitalWrite(PIN_CHECK_ENGINE, LOW);
      if (rawThw < 300) {
        digitalWrite(PIN_FAN_OUTPUT, HIGH);
      } else if (rawThw > 345) {
        digitalWrite(PIN_FAN_OUTPUT, LOW);
      }
    }

    sdFramesWritten++;
    int loggedCoolant = liveThwFault ? -99 : coolantF;
    int fanState      = digitalRead(PIN_FAN_OUTPUT);

    // =========================================================================
    // LESSON 14: Format Deterministic Payload & Stamp 8-Bit XOR Checksum (*XX)
    // =========================================================================
    char framePayload[96];
    snprintf(
      framePayload, sizeof(framePayload),
      "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
      currentMillis, calculatedRpm, throttlePct, rawThw, loggedCoolant,
      tvisState ? 1 : 0, fanState, liveDtcVal, activeDtcCode,
      oxMv, vfMv, fuelTrimPct, loopClFlag, igfOkFlag
    );

    uint8_t xorCs = computeXorChecksum(framePayload);
    char csHex[4];
    snprintf(csHex, sizeof(csHex), "%02X", xorCs);

    // 2A. Write Checksum-Sealed Frame to Hardware SPI SD Card (HACHI14.CSV)
    if (spiSdReady && logFile) {
      logFile.print(framePayload);
      logFile.print('*');
      logFile.println(csHex);

      if (sdFramesWritten % 20 == 0) {
        logFile.flush();
      }
    }

    // 2B. Simultaneously stream Checksum-Sealed $TELEM over UART
    Serial.print(F("$TELEM,"));
    Serial.print(framePayload);
    Serial.print('*');
    Serial.println(csHex);

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
      if (liveThwFault) {
        lcd.print("ERR!  ");
      } else {
        printPadded3(coolantF);
        lcd.print("F ");
      }
      lcd.print(isClosedLoop ? "[CL] " : "[OL] ");
      lcd.print("IGf:");
      lcd.print(igfOkFlag ? "OK" : "NO");

      lcd.setCursor(0, 3);
      if (liveIgfFault || activeDtcCode == 14) {
        lcd.print("DTC:14 IGF FUEL-CUT!");
      } else if (sensorFaultLatched) {
        lcd.print("DTC:0");
        lcd.print(activeDtcCode);
        lcd.print(" SD-REC:");
        printPadded4((int)(sdFramesWritten % 10000));
        lcd.print("  ");
      } else if (shiftAlert) {
        lcd.print("*** SHIFT! 7200+ ***");
      } else if (!isClosedLoop) {
        lcd.print("OL:WARMUP VF:0V#");
        printPadded4((int)(sdFramesWritten % 10000));
      } else {
        lcd.print("OX:");
        lcd.print(oxMv < 450 ? "LN " : "RCH");
        lcd.print(" VF:");
        printSignedTrim(fuelTrimPct);
        lcd.print(" #");
        printPadded4((int)(sdFramesWritten % 10000));
      }
    }
  }
}