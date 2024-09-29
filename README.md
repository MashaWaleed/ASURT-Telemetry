# ASU Racing Team - STM32 NRF Telemetry System (FSUK24)

## Project Overview

This project forms part of the telemetry system used in the ASU Racing Team's student Formula Racing car for Formula Student UK 2024 (FSUK24). The system is designed to facilitate real-time data transmission from the car to a remote station using NRF24L01 transceivers and CAN bus for in-vehicle communication, with tasks scheduled by the FreeRTOS real-time operating system.

The system features two key operational modes:

- **Transmitter Mode**: Collects data from the CAN bus and transmits it wirelessly.
- **Receiver Mode**: Receives data wirelessly from the car's telemetry system and displays/logs it using UART.

## System Components

- **MCU**: STM32F103C8T6 (Blue Pill)
- **Wireless Communication**: NRF24L01 for low-power, short-range wireless data transmission
- **Communication Protocols**:
  - **CAN (Controller Area Network)**: For in-vehicle communication between the telemetry system and the car's ECU.
  - **UART**: For receiving telemetry data on the remote station side.
- **Real-Time Operating System**: FreeRTOS for task scheduling.

## Features

### 1. NRF24L01 Wireless Communication

- **Transmitter and Receiver Mode**: You can configure the system as either a transmitter or receiver based on a build directive.
- **2-Way Communication**: Optionally, the system supports two-way communication for ACK responses.

### 2. CAN Bus Communication

- Integrated CAN support via STM32 HAL and CubeMX configuration.
- Configurable CAN filters for specific message reception.

### 3. FreeRTOS Multitasking

- Task-based architecture with FreeRTOS handling task scheduling.
- Modular tasks for CAN bus data handling, NRF transmission, and system state management.

### 4. Scalability & Flexibility

- Easily configurable to handle different data payloads.
- Adjustable wireless channel, payload size, and retry mechanisms for NRF communication.

## Hardware Setup

- **MCU**: STM32F103C8T6 (Blue Pill)
- **NRF24L01**: Connected via SPI
- **CAN Transceiver**: MCP2551 or similar connected to CAN RX/TX lines
- **UART**: For communication with the receiving station (optional two-way communication)

## Software Architecture

### Transmitter Mode (`#define TRANSMITTER`)

- **NRF Transmit Task**: Sends telemetry data collected from CAN bus over NRF24L01.
- **CAN Receive Task**: Listens for messages on the CAN bus, packages the data, and queues it for transmission.
- **NRF Init Task**: Configures the NRF24L01 module at system startup.
- **NRF State Task**: Continuously monitors the NRF24L01 for transmission failures and resets if necessary.

### Receiver Mode (`#define RECEIVER`)

- **NRF Receive Task**: Receives wireless data and processes it for display or logging via UART.
- **NRF Init Task**: Configures the NRF24L01 module for receiving at system startup.
- **NRF State Task**: Monitors the NRF24L01's status and handles re-initialization if communication errors occur.

## Building and Flashing

This project is built using STM32CubeIDE. To switch between Transmitter and Receiver modes, uncomment the corresponding `#define` in the **main.c** file:

- For **Transmitter**: `#define TRANSMITTER`
- For **Receiver**: `#define RECEIVER`

1. Open the project in **STM32CubeIDE**.
2. Modify the build directive (`TRANSMITTER` or `RECEIVER`) in **main.c**.
3. Build the project.
4. Flash the binary using ST-Link or USB-to-serial.

## NRF Configuration

You can modify the NRF24L01 configurations in the initialization tasks:

- **Channel**: Default is 100.
- **Data Rate**: 1 Mbps.
- **Payload Length**: 10 bytes.
- **Address Width**: 5 bytes.

These settings can be changed depending on the requirements of the application.

## CAN Bus Filtering

The CAN configuration allows filtering to focus on specific message IDs:

- Filters can be customized in the **can.c** file.
- The system handles both standard CAN messages and extended IDs depending on the vehicle subsystems.

## Usage Notes

- Ensure that the **NRF24L01** modules are properly connected to the **SPI1** interface on the STM32.
- The **CAN bus** setup requires proper termination resistors and transceiver wiring.
- The system can be expanded to handle more complex data packets by adjusting the **TX/RX Message** structures.

## Further Development

Possible improvements include:

- Integrating GPS or other sensor data.
- Expanding the two-way communication feature for real-time feedback.
- Enhancing error handling and diagnostics for field use.

## License

This project is released under an open-source license by **STMicroelectronics**. You can find more details in the LICENSE file.
