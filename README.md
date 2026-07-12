<div align="center">

# 🚀 PRAGYAN – STM32 Rocket Flight Computer

### Project EKALAVYA

**> Embedded avionics flight computer built on **STM32F401CCU6**, featuring barometric altitude estimation, 9-axis IMU motion tracking, high-G sensing, onboard data logging, and autonomous recovery deployment.**

<br>

![STM32](https://img.shields.io/badge/STM32-BlackPill%20F401CCU6-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![Embedded C](https://img.shields.io/badge/Embedded%20C-Firmware-00599C?style=for-the-badge&logo=c&logoColor=white)
![STM32CubeIDE](https://img.shields.io/badge/STM32CubeIDE-Development-0C5DA5?style=for-the-badge)
![Flight Computer](https://img.shields.io/badge/Rocket-Flight%20Computer-E53935?style=for-the-badge)
![Data Logging](https://img.shields.io/badge/MicroSD-Data%20Logging-2E7D32?style=for-the-badge)
![LoRa](https://img.shields.io/badge/LoRa-Wireless%20Communication-8E24AA?style=for-the-badge)

</div>
---

## Overview

**PRAGYAN** is the Electronics Bay (E-Bay) flight computer at the heart of **Project EKALAVYA**. It serves as the central avionics unit, integrating flight sensors, data logging systems, and deployment control logic into a single reliable embedded platform.

Its design prioritizes **mission safety**, **fault tolerance**, and **modularity** to ensure accurate flight data capture and dependable recovery system activation.

---

## Objectives

- Accurate measurement of flight parameters — altitude, pressure, and acceleration
- Reliable detection of apogee
- Safe and timely deployment of the recovery system
- Robust onboard data logging for post-flight analysis

---

## Features

- **Barometric Altitude Estimation** — DPS310 high-resolution pressure sensor for precise altitude tracking during ascent and descent
- **9-Axis IMU Motion Tracking** — ICM20948 captures rocket orientation, angular velocity, and low-G flight dynamics
- **High-G Sensing** — ADXL375 accelerometer for extreme acceleration measurement during the boost phase
- **Autonomous Recovery Deployment** — Transistor-based deployment circuit controlled by the flight computer for parachute activation
- **Onboard Data Logging** — Micro-SD card module for time-stamped flight data recording
- **Remote Ignition System** — LoRa-based wireless ignition via Launch Control Box (Transmitter) and Ignition Unit (Receiver)

---

## System Requirements

**STM32CubeIDE:**
Download and install from the official ST website:

    https://www.st.com/en/development-tools/stm32cubeide.html

**ST-Link V2:**
Required for flashing and debugging the STM32F401CCU6.

**Hardware:**
The firmware requires the following components for full functionality. Without the deployment circuit and pyro hardware, the system can still be used for sensor data acquisition and logging.

---

## Hardware Specifications

| Component | Specification & Function |
|-----------|--------------------------|
| **STM32F401CCU6 Blackpill** | ARM Cortex-M4 based 32-bit microcontroller. Handles sensor interfacing, real-time data processing, event sequencing, and deployment control. |
| **DPS310** | High-resolution barometric sensor for atmospheric pressure measurement and precise altitude estimation. |
| **ICM20948 IMU** | 9-axis inertial measurement unit — captures rocket orientation, angular velocity, and low-G flight dynamics. |
| **ADXL375 Accelerometer** | High-range accelerometer for measuring extreme acceleration during boost phase; complements IMU data for reliable high-G event detection. |
| **Micro-SD Card Module** | Non-volatile onboard storage for logging time-stamped sensor data for post-flight analysis. |
| **Transistor-Based Deployment Circuit** | Electronic switching interface controlled by the flight computer to activate the parachute deployment mechanism. |

---

## MCU Pin Configuration

| Pin | Function |
|-----|----------|
| PA1 | TIM2_CH2 (Timer / PWM) |
| PA2 | USART2_TX |
| PA3 | USART2_RX |
| PA4 | GPIO Output (Deployment / Pyro channel) |
| PA5 | SPI1_SCK (SD Card) |
| PA6 | SPI1_MISO (SD Card) |
| PA7 | SPI1_MOSI (SD Card) |
| PA8 | I2C3_SCL (Sensor bus) |
| PA9 | GPIO Output |
| PA13 | SWD IO (Debug) |
| PA14 | SWD CLK (Debug) |
| PB4 | I2C3_SDA (Sensor bus) |
| PB6 | I2C1_SCL (Sensor bus) |
| PB7 | I2C1_SDA (Sensor bus) |
| PB12 | GPIO Output (Pyro channel) |
| PC13 | GPIO Output |

---

## Peripheral Configuration

| Peripheral | Mode | Purpose |
|------------|------|---------|
| I2C1 | Fast Mode (400 kHz) | DPS310 / ICM20948 / ADXL375 communication |
| I2C3 | Fast Mode (400 kHz) | Secondary sensor bus |
| SPI1 | Full-Duplex Master, 8 Mbps | Micro-SD card interface |
| USART2 | Asynchronous | Serial debug / telemetry output |
| TIM2 | Internal clock, CH2 | Precise timing and event generation |
| FATFS | User-defined, LFN enabled, 4096B sector | SD card file system |
| SYS | SysTick | HAL timebase |

---

## Flight State Machine

```
[PAD / IDLE] → [POWERED ASCENT] → [COASTING] → [APOGEE] → [DESCENT] → [LANDED]
```

**State Transitions:**

- **ADXL375** detects high-G event at motor ignition → transitions to `POWERED ASCENT`
- **ICM20948** monitors angular velocity and acceleration → detects motor burnout → `COASTING`
- **DPS310** continuously tracks altitude → detects pressure plateau/reversal → `APOGEE`
- Apogee triggers deployment circuit → parachute ejection → `DESCENT`
- All state transitions and sensor data are time-stamped and logged to SD card

---

## Data Logging

All flight data is written to a FAT-formatted Micro-SD card via SPI. Logged parameters include:

- Timestamp (ms since boot)
- Altitude (m) and Pressure (Pa) — from DPS310
- Temperature (°C) — from DPS310
- 3-axis acceleration (low-G) — from ICM20948
- 3-axis acceleration (high-G) — from ADXL375
- Angular velocity (X/Y/Z) — from ICM20948
- Flight state / event flags

Files are stored in **CSV format** for easy post-flight analysis with Python, MATLAB, or Excel.

---

## Project Structure

```
stm32-rocket-flight-computer/
├── Core/
│   ├── Src/          # main.c, sensor drivers (DPS310, ICM20948, ADXL375), flight logic
│   └── Inc/          # Header files
├── Drivers/          # STM32 HAL and CMSIS drivers
├── FATFS/            # FatFS file system middleware
├── Middlewares/
│   └── Third_Party/FatFs/src/   # FatFs source
├── data.ioc          # STM32CubeMX project configuration
├── STM32F401CCUX_FLASH.ld       # Linker script
├── .cproject         # STM32CubeIDE project file
└── .mxproject        # CubeMX metadata
```

---

## Installation

### Build and Flash

**1. Clone the repository:**

    $ git clone https://github.com/YashTatode/stm32-rocket-flight-computer.git

**2. Open in STM32CubeIDE:**

    File → Open Projects from File System

Navigate to the cloned directory and import.

**3. Build the project:**

    Ctrl + B

**4. Flash via ST-Link:**

    Run → Debug

### Reconfiguring Peripherals

Open `data.ioc` in STM32CubeMX to visually reconfigure pins, clocks, or peripherals. After making changes, regenerate the HAL initialization code and rebuild.

---

## Testing Your Hardware

After wiring your sensors and deployment circuit, use the USART2 serial output (baud: 115200) to verify sensor readings before a flight attempt.

**Check sensor communication:**

Confirm I2C ACK responses from DPS310 (addr: `0x77`), ICM20948 (addr: `0x68`), and ADXL375 (addr: `0x53`) during initialization.

**Verify SD card logging:**

After a test power cycle, remove the SD card and inspect the generated CSV file to confirm all sensor fields are populated correctly.

**Test deployment circuit:**

Use a dummy load (LED or 10Ω resistor) in place of the pyro charge on PA4 / PB12 to verify the transistor switching logic fires at the expected apogee event.

---

## Tech Stack

| Category | Details |
|----------|---------|
| **Language** | C |
| **Firmware** | STM32 HAL (STM32Cube FW_F4 V1.28.3) |
| **Toolchain** | GCC / STM32CubeIDE |
| **File System** | FatFs middleware |
| **Sensors** | DPS310 (I2C), ICM20948 (I2C), ADXL375 (I2C) |
| **Wireless** | LoRa (ignition system) |

---

## Safety Notice

This project involves **pyrotechnic recovery systems** and a **remote ignition system**. Always follow local regulations and rocketry safety codes (e.g., NAR, TRA).

> **Never connect pyro channels or the ignition system to live charges during bench testing.**
> Use dummy loads (LEDs or resistors) during all development and verification work.

---

## License

Open source. Feel free to fork, modify, and build upon for your own rocketry projects. Attribution appreciated.

---

## Author

**Yash Tatode**
[github.com/YashTatode](https://github.com/YashTatode)

---

*Built for Project EKALAVYA — pushing the boundaries of amateur rocketry avionics.*
