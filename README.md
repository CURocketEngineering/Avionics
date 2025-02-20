# Avionics for Clemson University Rocket Engineering

**Avionics** is an Arduino-Core based system designed for high reliability and efficiency, serving as the backbone for multiple flight computers in the Clemson University Rocket Engineering program.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Dependent Systems](#dependent-systems)
- [Testing](#testing)
  - [Unit Testing](#unit-testing)
  - [Flight Tests](#flight-tests)

## Introduction

This repository contains the avionics system used across various flight computers within the Clemson University Rocket Engineering program. Designed to be modular and highly configurable, the system supports critical functions such as state detection, data logging, and robust communication.

## Features

### State Detection

- **Launch Detection:**  
  The [Launch Predictor](include/data_handling/LaunchPredictor.h) uses a rolling median of acceleration data to accurately detect launches. It is highly configurable to accommodate different launch profiles and includes stringent data rate and error-handling mechanisms.

- **Kalman Filter:**  
  The [Kalman Filter](include/kf-2d.h) module fuses accelerometer and altimeter data to provide precise estimates of vertical velocity.

### Data Logging

- **SPI Flash-based Logging:**  
  [DataSaverSPI](include/data_handling/DataSaverSPI.h) logs sensor data directly to any W25 SPI flash chip. Each sensor logs data at its own rate using the "Byte5" format, which is optimized for space efficiency while allowing for unordered data. Data is stored in a circular buffer until post-launch mode is activated.

- **Serial-based Logging:**  
  [DataSaverSDSerial](include/data_handling/DataSaverSDSerial.h) provides an alternative logging mechanism by streaming binary data to a serial interface. Although less space-efficient than "Byte5", it is designed for applications with large SD cards. Refer to the [Serial-Logger-Decoding](https://github.com/CURocketEngineering/Serial-Logger-Decoding) repository for decoding instructions.

### Communication

- **Serial Command Handler:**  
  The [UARTCommandHandler](include/UARTCommandHandler.h) offers a fully configurable command interface for runtime configuration, debugging, and data retrieval, using function pointers for flexibility.

- **CC1125 Transceiver:**  
  The [CC1125](include/CC1125.h) module enables direct communication via an embedded CC1125 transceiver over SPI.

## Dependent Systems

The following systems integrate Avionics as a submodule:

- [MARTHA 1.3](https://github.com/CURocketEngineering/MARTHA-1.3)
- [Active-Aero](https://github.com/CURocketEngineering/Active-Aero)
- [MARTHA 1.1](https://github.com/CURocketEngineering/MARTHA-1.1)
- [Native](https://github.com/CURocketEngineering/Native)

## Testing

### Unit Testing

Unit tests are managed by the [Native](https://github.com/CURocketEngineering/Native) repository, allowing for module testing without requiring embedded hardware.

### Flight Tests

The following rocket flights have successfully utilized this avionics system (most recent first):

- **[MARTHA 1.3 Feb 8th 2025 Test Vehicle Launch Attempt](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.0)**
 MARTHA was mounted to the Active Aero test vehicle, but it never launched. Data was collected for 14.18 hours before the 9V battery died. The data was successfully dumped, and the last hour of data before shutdown was recorded. A consistent 100Hz loop speed was achieved. During a rough assembly, the LaunchPredictor never had a false positive, even though it detected large acceleration values.


- **[MARTHA 1.1 Nov 9th L1 Cert Launch](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.1.0):**  
  Launch detection and SD card data logging performed as expected. Note: No altitude data was recorded due to issues with sensor drivers or hardware damage.

- **[MARTHA 1.1 Spaceport 2024](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.0.0):**  
  Although data was captured via the serial logger, the battery failure on the pad resulted in no launch data being recorded.

## License

This project is licensed under the [MIT License](LICENSE). Please review the LICENSE file for complete details.