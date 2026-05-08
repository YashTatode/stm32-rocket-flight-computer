# 🚀 STM32 Rocket Flight Computer — Project EKALAVYA

**PRAGYAN** — Embedded avionics flight computer built on STM32F401CCU6, featuring DPS310 barometric altitude estimation, ICM20948 9-axis IMU motion tracking, ADXL375 high-G sensing, onboard data logging, and autonomous recovery deployment.

---

## 📋 Overview

**PRAGYAN** is the Electronics Bay (E-Bay) flight computer at the heart of **Project EKALAVYA**. It serves as the central avionics unit, integrating flight sensors, data logging systems, and deployment control logic into a single reliable embedded platform.

Its design prioritizes **mission safety**, **fault tolerance**, and **modularity** to ensure accurate flight data capture and dependable recovery system activation.

---

## 🎯 Objectives

- Accurate measurement of flight parameters — altitude, pressure, and acceleration
- Reliable detection of apogee
- Safe and timely deployment of the recovery system
- Robust onboard data logging for post-flight analysis

---

## ✨ Features

- **Barometric Altitude Estimation** — DPS310 high-resolution pressure sensor for precise altitude tracking during ascent and descent
- **9-Axis IMU Motion Tracking** — ICM20948 captures rocket orientation, angular velocity, and low-G flight dynamics
- **High-G Sensing** — ADXL375 accelerometer for extreme acceleration measurement during the boost phase
- **Autonomous Recovery Deployment** — Transistor-based deployment circuit controlled by the flight computer for parachute activation
- **Onboard Data Logging** — Micro-SD card module for time-stamped flight data recording
- **Remote Ignition System** — LoRa-based wireless ignition via Launch Control Box (Transmitter) and Ignition Unit (Receiver)

---

## 🛠️ Hardware Specifications

| Component | Specification & Function |
|-----------|--------------------------|
| **STM32F401CCU6 Blackpill** | ARM Cortex-M4 based 32-bit microcontroller. Primary flight computer — handles sensor interfacing, real-time data processing, event sequencing, and deployment control. |
| **DPS310** | High-resolution barometric sensor for atmospheric pressure measurement and precise altitude estimation during ascent and descent phases. |
| **ICM20948 IMU** | 9-axis inertial measurement unit — captures rocket orientation, angular velocity, and low-G flight dynamics for accurate motion tracking. |
| **ADXL375 Accelerometer** | High-range accelerometer for measuring extreme acceleration levels during the boost phase; complements IMU data for reliable high-G event detection. |
| **Micro-SD Card Module** | Non-volatile onboard storage for logging time-stamped altitude, acceleration, and inertial data for post-flight analysis. |
| **Transistor-Based Deployment Circuit** | Electronic switching interface controlled by the flight computer to activate the parachute deployment mechanism. |

---

## 📌 MCU Pin Configuration

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

## ⚙️ Peripheral Configuration

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

## 🚦 Flight State Machine

```
[PAD / IDLE] → [POWERED ASCENT] → [COASTING] → [APOGEE] → [DESCENT] → [LANDED]
```

- **ADXL375** detects high-G event at motor ignition → transitions to POWERED ASCENT
- **ICM20948** monitors angular velocity and acceleration → detects motor burnout → COASTING
- **DPS310** continuously tracks altitude → detects pressure plateau/reversal → **APOGEE**
- Apogee triggers deployment circuit → parachute ejection → DESCENT
- All state transitions and sensor data are time-stamped and logged to SD card

---

## 🔥 Ignition System

The **communication-based ignition system** enables safe and remote rocket ignition using **LoRa wireless communication**.

The system consists of two units:

- **Launch Control Box (Transmitter)** — Ground-side unit with push-button interface to send the ignition command wirelessly
- **Ignition Unit (Receiver)** — On-board unit that receives the LoRa command and initiates the ignition sequence

Ignition activates the rocket's propulsion system by initiating the combustion of the propellant, generating the thrust required for ascent.

---

## 🗂️ Project Structure

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

## 📊 Data Logging

All flight data is written to a FAT-formatted Micro-SD card via SPI. Logged parameters include:

- Timestamp (ms since boot)
- Altitude (m) and Pressure (Pa) — from DPS310
- Temperature (°C)
- 3-axis acceleration (low-G) — from ICM20948
- 3-axis acceleration (high-G) — from ADXL375
- Angular velocity (X/Y/Z) — from ICM20948
- Flight state / event flags

Files are stored in CSV format for easy post-flight analysis with Python, MATLAB, or Excel.

---

## 🔧 Getting Started

### Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (GCC toolchain)
- ST-Link V2 programmer / debugger
- STM32F401CCU6 Blackpill board

### Build & Flash

1. Clone the repository:
   ```bash
   git clone https://github.com/YashTatode/stm32-rocket-flight-computer.git
   ```
2. Open in **STM32CubeIDE** via `File → Open Projects from File System`.
3. Build: `Ctrl+B`
4. Flash via ST-Link: `Run → Debug`

### Reconfiguring Peripherals

Open `data.ioc` in STM32CubeMX to visually reconfigure pins, clocks, or peripherals, then regenerate HAL initialization code.

---

## ⚠️ Safety Notice

This project involves **pyrotechnic recovery systems** and a **remote ignition system**. Always follow local regulations and rocketry safety codes (e.g., NAR, TRA). Never connect pyro channels or the ignition system to live charges during bench testing. Use dummy loads (LEDs or resistors) during development.

---

## 🧰 Tech Stack

- **Language:** C
- **Firmware:** STM32 HAL (STM32Cube FW_F4 V1.28.3)
- **Toolchain:** GCC / STM32CubeIDE
- **File System:** FatFs middleware
- **Sensors:** DPS310 (I2C), ICM20948 (I2C), ADXL375 (I2C/SPI)
- **Wireless:** LoRa (ignition system)

---

## 📄 License

Open source. Feel free to fork, modify, and build upon for your own rocketry projects. Attribution appreciated!

---

## 🙋 Author

**Yash Tatode**  
[GitHub Profile](https://github.com/YashTatode)

---

*Built for Project EKALAVYA — pushing the boundaries of amateur rocketry avionics. 🚀*
