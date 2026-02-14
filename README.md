# Industrial IoT Seismic Sensor Node

![Language](https://img.shields.io/badge/Language-C-blue.svg)
![Platform](https://img.shields.io/badge/Platform-STM32_Nucleo-green.svg)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-orange.svg)
![Stack](https://img.shields.io/badge/Network-LwIP_Ethernet-blueviolet.svg)

## 🚀 Project Overview
A robust, real-time vibration monitoring system designed for industrial data acquisition. This device samples analog seismic data, timestamps it with high precision, buffers it into non-volatile FRAM, and streams telemetry via Ethernet (TCP/UDP).

The firmware architecture prioritizes **concurrency safety** and **determinism** by utilizing **Gatekeeper Tasks** to manage hardware resources (I2C/SPI) and **DMA** for zero-CPU signal acquisition.

## 🛠 Hardware Architecture
The system integrates a mix of analog signal processing and digital bus communication:

* **MCU:** STM32 Nucleo-F767ZI (ARM Cortex-M7, 216 MHz)
* **Sensor:** Analog 3-Axis Accelerometer (sampled via ADC1)
* **Storage:** External FRAM (Ferroelectric RAM)
    * *Interface:* **SPI** (Via SPI Gatekeeper Task)
    * *Role:* High-endurance persistence for peak vibration events and remote node data.
* **Timing:** External RTC (DS3231 or similar)
    * *Interface:* **I2C** (Via I2C Gatekeeper Task)
    * *Role:* Independent hardware timestamping, synchronized via NTP.

## 🧩 Firmware Architecture
The firmware uses **STM32CubeIDE** and **FreeRTOS** with a modular "Gatekeeper" pattern to ensure thread safety.

### 1. High-Performance Acquisition (DMA)
* **Zero-Copy Sampling:** The ADC operates in Circular Mode via **DMA**, transferring conversion data directly to RAM without CPU intervention.
* **Signal Processing:** A dedicated `AdcConvert` task waits for the `ADC_BUF_READY` signal, calculates the Root Mean Square (RMS) magnitude, and detects peak events.

### 2. The "Gatekeeper" Pattern (Crucial for Reliability)
To prevent bus contention (race conditions) between multiple RTOS tasks, direct HAL calls are forbidden. Instead:
* **I2C & SPI Gatekeepers:** Dedicated tasks (`StartI2cGatekeeperTask`, `StartSpiGatekeeperTask`) own the hardware handles (`hi2c2`, `hspi3`).
* **Mailbox Queues:** Client tasks (e.g., Logging, Storage) send "Job Requests" via FreeRTOS Mail Queues (`osMailPut`). The Gatekeeper executes the transaction and signals completion.

### 3. Networking Stack (LwIP)
* **Telemetry Server:** A TCP Server task handles incoming connections for real-time data visualization.
* **UDP Heartbeat:** Broadcasts device presence and status to the subnet for auto-discovery.
* **NTP Sync:** A dedicated task synchronizes the local RTC with `be.pool.ntp.org` to ensure timestamps match UTC standards.

## 📂 Project Structure
```text
/Core/Src
  ├── main.c              # Hardware init & Task creation
  ├── app_peripherals.c   # Gatekeeper implementations (I2C/SPI) & ISRs
  ├── app_network.c       # LwIP Socket tasks (TCP Server/Client, UDP, NTP)
  ├── app_processing.c    # Signal analysis & JSON parsing
/Middlewares              # FreeRTOS & LwIP Source
