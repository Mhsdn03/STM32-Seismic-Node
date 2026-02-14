# Industrial IoT Seismic Sensor Node

## 🚀 Project Overview
Real-time vibration monitoring system . This device acquires seismic data using an accelerometer, processes it using **FreeRTOS** tasks, and streams telemetry via Ethernet (TCP/UDP) using the **LwIP** stack.

**Key Features:**
* **Real-Time OS:** Deterministic task scheduling with FreeRTOS.
* **High-Speed Comms:** 100 Mbps Ethernet integration.
* **Data Acquisition:** MEMS Accelerometer integration via SPI/I2C.

## 🛠 Hardware & Software Stack
* **MCU:** STM32 Nucleo-F767ZI (ARM Cortex-M7)
* **Sensor:** ADXL345
* **Middleware:** FreeRTOS, LwIP (Lightweight IP)
* **Tools:** STM32CubeIDE, Wireshark (Packet Analysis)

## 🧩 Architecture
The firmware is architected into three primary concurrent tasks:
1.  **Acquisition Task (High Priority):** Handles SPI interrupts and fills the circular buffer.
2.  **Processing Task:** Filters raw data and prepares packets.
3.  **Network Task:** Manages the LwIP stack and handles UDP/TCP transmission.

## ⚡ How to Build
1.  Clone the repository.
2.  Import into STM32CubeIDE.
3.  Build for "Nucleo-F767ZI".
