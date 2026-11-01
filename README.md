# 1986 Toyota 4A-GE (AE86) Hardware-in-the-Loop (HIL) Validation Testbed & "Flight" Data Recorder

**Author:** Mason Litzenberger (`masontl`)  
**Architecture:** Dual-MCU ATmega328P HIL Plant & Observer | Hardware RC Signal Conditioning | Python Automated Fault Injection & Offline "Flight" Log Dissector  
**Target System:** 1986 Toyota Corolla GT-S (AE86) — 16V 4A-GE Nippondenso Toyota Computer-Controlled System (TCCS) Engine Control System (ECU) 

---

## 1. Executive Summary & Engineering Objective

This project implements a full **Hardware-in-the-Loop (HIL) simulation, real-time cockpit observation suite, and offline telemetry post-processing pipeline** designed to model and diagnose a 1986 Toyota 4A-GE engine and its factory Nippondenso TCCS ECU.

Rather than modifying or replacing the high-reliability factory 1986 ECU, this system follows an **aerospace/flight-test instrumentation philosophy**:
1. **100% Non-Invasive High-Impedance Eavesdropping:** Observes factory analog and digital control signals (`IG-`, `VTA`, `THW`, `T-VIS`, `OX`, `VF`) without loading down the primary vehicle harness.
2. **Closed-Loop Plant Simulation (MCU #1):** A dedicated C++ real-time engine physics & fault-injection plant running on a separate microcontroller, feeding conditioned analog and pulse-train signals through hardware RC filters into the System Under Test (MCU #2).
3. **Dual-Architecture Telemetry Recording (MCU #2 + Host Pipeline):**
   * **Sealed-ECU OBD-II Style Freeze-Frame:** Asymmetric `2/3 Pre-Trigger (20s) : 1/3 Post-Trigger (10s)` circular buffer stored in non-volatile on-chip `EEPROM` using non-blocking trickle writes.
   * **UAV / R&D Flight Data Recorder (FDR):** Continuous `20 Hz` (`50 ms`) 12-channel telemetry streaming over **Hardware SPI (`SD.h` FAT16 disk image)** and **UART (`115200` baud)** with automated Python offline root-cause dissection.

---

## 2. System Block Diagram & Signal Interface

| Signal | Physical Parameter | MCU #1 (Virtual 4A-GE Plant) | Conditioning Stage | MCU #2 (Cockpit / FDR Observer) | Diagnostic Role |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`IG-`** | Ignition Coil Tachometer Pulse | `Pin D9` (2 pulses/rev, `2000–7700 RPM`) | Direct Digital + Transistor Audio Stage | `Pin D2` (`INT0` Hardware Interrupt) | Microsecond edge-to-edge RPM calculation |
| **`VTA`** | Throttle Position Sensor (TPS) | `Pin D10` (`PWM Out`) | $4.7\text{ k}\Omega + 10\text{ }\mu\text{F}$ RC LPF ($\tau = 47\text{ ms}$) | `Pin A0` (`10-bit ADC`) | Load tracking, T-VIS gating, Code 7 detection |
| **`THW`** | NTC Coolant Thermistor | `Pin D11` (`PWM Out`, `160°F–230°F`) | $4.7\text{ k}\Omega + 10\text{ }\mu\text{F}$ RC LPF + SPST Fault Switch | `Pin A1` (`10-bit ADC`) | Thermal tracking, Fan control, Code 4 failsafe |
| **`OX`** | Emulated Narrowband O2 Sensor | `Pin D6` (`PWM Out`, `0.0V–1.0V`) | $4.7\text{ k}\Omega + 10\text{ }\mu\text{F}$ RC LPF | `Pin A2` (`10-bit ADC`) | Stoichiometric switching vs. Vacuum Leak lean pin |
| **`VF`** | TCCS Closed-Loop Fuel Trim | `Pin D5` (`PWM Out`, `0.0V–5.0V`) | $4.7\text{ k}\Omega + 10\text{ }\mu\text{F}$ RC LPF | `Pin A3` (`10-bit ADC`) | Internal ECU Fuel Trim integrator (`-20%` to `+20%`) |
| **`T-VIS`** | Intake Runner Butterfly VSV | `Pin D8` (`Feedback In`) | Closed-Loop Actuator Line | `Pin D8` (`Digital Out`) | Opens secondary intake runners $\ge 4400\text{ RPM}$ (`+200 RPM/tick` surge) |
| **`FAN`** | Radiator Cooling Fan Relay | `Pin D7` (`Feedback In`) | Closed-Loop Thermal Line | `Pin D9` (`Digital Out`) | Hysteresis cooling (`0.22°F/tick`) + Code 4 100% ON Failsafe |
| **`SPI SD`** | Onboard FAT16 Flight Recorder | — | 4-Wire Hardware SPI Bus | `D10 (CS), D11 (MOSI), D12 (MISO), D13 (SCK)` | Writes `20 Hz` CSV telemetry to `32 MB` FAT16 `.img` |

---

## 3. Iterative Engineering Progression (Directory Index)

### Phase 1: Signal Generation, Interrupt Acquisition & Closed-Loop Plant Dynamics (Lessons 1–5)
* **Hardware Timer & Interrupt Tachometer:** Microsecond `IG-` ignition pulse generation on MCU #1 and `FALLING` edge interrupt capture (`INT0`) on MCU #2.
* **Analog RC Low-Pass Filtering:** Converting 8-bit PWM plant outputs into smooth analog sensor voltages using $4.7\text{ k}\Omega + 10\text{ }\mu\text{F}$ RC filters ($f_c \approx 3.4\text{ Hz}$) and eliminating boot-up capacitor charge false triggers.
* **Closed-Loop Actuators & Hysteresis:** Bidirectional hardware feedback between MCU #2 and MCU #1 for **T-VIS intake runner actuation** (`4400 RPM` with `150 RPM` hysteresis) and **Radiator Fan thermal regulation** (`12°F` natural thermal hysteresis loop).
* **Cockpit HMI:** Custom 5x8 CGRAM sub-pixel characters driving a 40-step real-time RPM bar graph and telemetry readout on a 20x4 `AIP31068` I2C LCD (`Addr 39`).

### Phase 2: Hard Fault Injection & Asymmetric EEPROM Freeze-Frame Recorder (Lessons 6–8)
* **Automated UART Fault Injection:** Live ASCII command injection (`'F'` = THW Code 4, `'T'` = VTA Code 7, `'B'` = Dual Fault, `'N'` = Heal) into MCU #1 to simulate wire breaks and shorts-to-ground.
* **Slew-Rate vs. Instant Fault Discrimination:** Distinguishing a physical harness short (`< 50 ms` instant step to `0 ADC`) from an upstream sensor failure slewing through the $47\text{ ms}$ RC filter.
* **Asymmetric 2/3 Pre : 1/3 Post EEPROM Black-Box (Lesson 8):**
  * Implements a `60-slot @ 2 Hz` (`30.0s` total window: `20.0s Pre-Trigger` + `10.0s Post-Trigger`) circular RAM ring buffer on MCU #2.
  * Commits the freeze-frame to non-volatile on-chip `EEPROM` (`367 bytes`) using non-blocking background `EEPROM.update()` trickle writes (`1 slot per 50ms loop`) to preserve deterministic `20 Hz` control timing.

### Phase 3: 20Hz UAV-Style Flight Data Recorder & Soft Closed-Loop Diagnostics (Lessons 9–10)
* **Decoupled 20Hz Flight Data Recorder (Lesson 9):**
  * Separates sealed-ECU EEPROM freeze-frames from R&D continuous data logging.
  * Streams 12 channels of `20 Hz` (`50 ms`) telemetry simultaneously to a **Hardware SPI FAT16 SD Card (`HACHI10.CSV` inside `hachi_sd_fat16.img`)** and a **Python Virtual SD Card Daemon (`sd_card_daemon.py`)**.
* **Soft Mechanical Fault & Closed-Loop Fuel Trim Dissection (Lesson 10):**
  * Models the real-world **AE86 Brake Booster Vacuum Leak** diagnostic scenario (`'V'` fault command).
  * Demonstrates how unmetered intake air under high manifold vacuum (`< 3800 RPM`) pins the wideband's emulated narrowband `OX` signal lean (`~100 mV`), tricking the 1986 TCCS ECU into commanding maximum rich fuel trim (`VF = 4.90V / +20%`) **without triggering an OBD-I Check Engine Light (`Live_DTC = 0`)**.
  * Includes an automated **Offline Post-Processor (`sd_post_processor.py`)** and **FAT16 Binary Image Extractor (`hachi_read_SDimg_create_csv.py`)** that sanitize power-down sector boundaries, partition fuel trims by engine load regime, verify radiator fan failsafe response, and automatically diagnose a manifold vacuum leak vs. high-load fuel starvation.

---

## 4. Repository Utilities & Quick-Start Workflow

1. **Generate Virtual FAT16 SPI SD Card Image (Optional for SimulIDE SPI Component):**
   ```bash
   python create_fat16_sd_img.py
