// ====================================================================
// LESSON 11 (SPI SD EDITION) — MCU #2: Hachi Cockpit OS + Open/Closed-Loop Gate
// SimulIDE HIL Test Bench - Runs on MCU #2 (SDA=A4, SCL=A5 @ Addr 39)
// Logs 13 Channels at 20Hz to 'HACHI11.CSV' on Hardware SPI SD Card (D10-D13)
// ====================================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_AIP31068_I2C.h>

LiquidCrystal_AIP31068_I2C lcd(39, 20, 4);

const int PIN_IG_INPUT     = 2;   // Input:  Interrupt 0 reads 4A-GE IG- pulses
const int PIN_CHECK_ENGINE = 6;   // Output: Check Engine LED (D13 reserved for SPI SCK)
const int PIN_SHIFT_HMI    = 7;   // Output: Driver Shift-Light / Acoustic Cue
const int PIN_TVIS_OUTPUT  = 8;   // Output: T-VIS VSV Relay Status Indicator
const int PIN_FAN_OUTPUT   = 9;   // Output: Electric Radiator Fan Driver
const int PIN_SD_CS        = 10;  // Output: Hardware SPI SD Card Chip Select (CS)

const int PIN_TPS_INPUT    = A0;  // Input:  Analog VTA throttle position
const int PIN_THW_INPUT    = A1;  // Input:  Analog THW coolant temp
const int PIN_OX_INPUT     = A2;  // Input:  Analog OX Narrowband O2 (0.0V - 1.0V)
const int PIN_VF_INPUT     = A3;  // Input:  Analog VF TCCS Fuel Trim (0.0V - 5.0V)

volatile unsigned long lastEdgeTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;

// Operating Thresholds
const int TVIS_ACTIVATE_RPM   = 4400;
const int TVIS_HYSTERESIS_RPM = 150;
const int SHIFT_LIGHT_RPM     = 7200;
const int MIN_THROTTLE_RAW    = 200;
const int SENSOR_MIN_VALID    = 41;
const int SENSOR_MAX_VALID    = 982;
const int CLOSED_LOOP_MIN_F   = 140;  // 1986 4A-GE TCCS Warmup Gate (140°F / 60°C)

bool tvisState          = false;
bool sensorFaultLatched = false;
int  activeDtcCode      = 0;
unsigned long sdFramesWritten = 0;

bool spiSdReady = false;
File logFile;

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
  pinMode(PIN_IG_INPUT, INPUT_PULLUP);
  pinMode(PIN_CHECK_ENGINE, OUTPUT);
  pinMode(PIN_SHIFT_HMI, OUTPUT);
  pinMode(PIN_TVIS_OUTPUT, OUTPUT);
  pinMode(PIN_FAN_OUTPUT, OUTPUT);
  pinMode(PIN_SD_CS, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_IG_INPUT), onIgnitionPulse, FALLING);
  Serial.begin(115200);

  lcd.init();
  lcd.createChar(1, bar1);
  lcd.createChar(2, bar2);
  lcd.createChar(3, bar3);
  lcd.createChar(4, bar4);
  lcd.createChar(5, bar5);

  // Initialize Hardware SPI SD Card on D10 and open HACHI11.CSV
  spiSdReady = SD.begin(PIN_SD_CS);
  if (spiSdReady) {
    logFile = SD.open("HACHI11.CSV", FILE_WRITE);
    if (logFile && logFile.size() == 0) {
      logFile.println(F("SimTime_ms,RPM,TPS_Pct,Raw_THW,Coolant_F,TVIS,Fan,Live_DTC,Latched_DTC,OX_mV,VF_mV,Fuel_Trim_Pct,Loop_CL"));
      logFile.flush();
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  THE HACHI (AE86)  ");
  lcd.setCursor(0, 2);
  lcd.print(spiSdReady ? " L11 SD: MOUNTED OK " : " L11 SD: NOT FOUND  ");
  lcd.setCursor(0, 3); lcd.print("====================");
  delay(800);
  lcd.clear();

  Serial.println(spiSdReady ? "$SD_MOUNT,SPI_FAT16_OK,HACHI11.CSV*" : "$SD_MOUNT,UART_FALLBACK,HACHI11.CSV*");
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
    }
  }

  // 1. High-Speed 20Hz (50ms) Acquisition, Control & Dual SPI/UART SD Stream
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
    int rawOx  = analogRead(PIN_OX_INPUT);
    int rawVf  = analogRead(PIN_VF_INPUT);

    int throttlePct = map(rawTps, 0, 1023, 0, 100);
    // Calibrated across full 60°F (923 ADC) to 230°F (140 ADC) range
    int coolantF    = map(constrain(rawThw, 140, 923), 923, 140, 60, 230);

    int oxMv = (int)((rawOx * 5000UL) / 1023);
    int vfMv = (int)((rawVf * 5000UL) / 1023);

    // Hard OBD-I Fault Detection
    bool liveFault = (rawThw < SENSOR_MIN_VALID || rawThw > SENSOR_MAX_VALID);
    if (liveFault) {
      sensorFaultLatched = true;
      activeDtcCode      = 4;
    }

    // LESSON 11: Open-Loop vs. Closed-Loop Gate Detection
    // In 1986 TCCS, VF outputs 0.0V (< 350mV after RC filter) when Open-Loop (< 140°F or THW fault).
    bool isClosedLoop = (!liveFault && coolantF >= CLOSED_LOOP_MIN_F && vfMv >= 350);
    int  loopClFlag   = isClosedLoop ? 1 : 0;

    // Only compute +/-20% Closed-Loop Fuel Trim when in Closed-Loop!
    // In Open-Loop (VF = 0V), report 0% trim so Open-Loop frames never masquerade as -20% lean trim!
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

    // Radiator Fan Control (Activates >= 195°F -> rawThw < 300 ADC, shuts off <= 185°F -> rawThw > 345 ADC)
    if (sensorFaultLatched) {
      digitalWrite(PIN_FAN_OUTPUT, HIGH);
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
    int loggedCoolant = liveFault ? -99 : coolantF;
    int fanState      = digitalRead(PIN_FAN_OUTPUT);
    int liveDtcVal    = liveFault ? 4 : 0;

    // 2A. Write 13 Channels to Hardware SPI SD Card (HACHI11.CSV)
    if (spiSdReady && logFile) {
      logFile.print(currentMillis);     logFile.print(',');
      logFile.print(calculatedRpm);     logFile.print(',');
      logFile.print(throttlePct);       logFile.print(',');
      logFile.print(rawThw);            logFile.print(',');
      logFile.print(loggedCoolant);     logFile.print(',');
      logFile.print(tvisState ? 1 : 0); logFile.print(',');
      logFile.print(fanState);          logFile.print(',');
      logFile.print(liveDtcVal);        logFile.print(',');
      logFile.print(activeDtcCode);     logFile.print(',');
      logFile.print(oxMv);              logFile.print(',');
      logFile.print(vfMv);              logFile.print(',');
      logFile.print(fuelTrimPct);       logFile.print(',');
      logFile.println(loopClFlag);

      if (sdFramesWritten % 20 == 0) {
        logFile.flush();
      }
    }

    // 2B. Simultaneously stream 13-channel $TELEM over UART
    Serial.print(F("$TELEM,"));
    Serial.print(currentMillis);     Serial.print(',');
    Serial.print(calculatedRpm);     Serial.print(',');
    Serial.print(throttlePct);       Serial.print(',');
    Serial.print(rawThw);            Serial.print(',');
    Serial.print(loggedCoolant);     Serial.print(',');
    Serial.print(tvisState ? 1 : 0); Serial.print(',');
    Serial.print(fanState);          Serial.print(',');
    Serial.print(liveDtcVal);        Serial.print(',');
    Serial.print(activeDtcCode);     Serial.print(',');
    Serial.print(oxMv);              Serial.print(',');
    Serial.print(vfMv);              Serial.print(',');
    Serial.print(fuelTrimPct);       Serial.print(',');
    Serial.print(loopClFlag);        Serial.println('*');

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
        lcd.print("F ");
      }
      lcd.print(isClosedLoop ? "[CL] " : "[OL] ");
      lcd.print("F:");
      lcd.print(fanState ? "ON " : "OFF");

      lcd.setCursor(0, 3);
      if (sensorFaultLatched) {
        lcd.print("DTC:04 SD-REC:");
        printPadded4((int)(sdFramesWritten % 10000));
        lcd.print(" ");
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