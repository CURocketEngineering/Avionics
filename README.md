# Avionics for Clemson University Rocket Engineering

Avionics is a modular, Arduino-core based system powering the flight computers of Clemson University Rocket Engineering. Designed for high reliability and efficiency, it supports real-time state estimation, data logging, and robust communication.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Dependent Systems](#dependent-systems)
- [Testing](#testing)
  - [Unit Testing](#unit-testing)
  - [Flight Tests](#flight-tests)

## Introduction

This repository contains the avionics system used across various flight computers within the Clemson University Rocket Engineering organization. Designed to be modular and highly configurable, the system supports critical functions such as state detection, data logging, and robust communication.

## Flight Tests

See a comprehensive list of all flights using this software at ***🚀[Flight Tests](docs/FlightTests.md)🚀***.

## Features

### State Detection Tools

- **Launch Detection:**  
  The [Launch Detector](include/data_handling/LaunchDetector.h) uses a rolling median of acceleration data to accurately detect launches. It is highly configurable to accommodate different launch profiles and includes stringent data rate and error-handling mechanisms.

- **Vertical Velocity Estimation:**  
  The [Vertical Velocity Estimator](include/state_estimation/VerticalVelocityEstimator.h) employs a 1D Kalman filter to estimate vertical velocity from acceleration and altitude data.

- **Apogee Detection:**  
  The [Apogee Detector](include/state_estimation/ApogeeDetector.h) utilizes the vertical velocity estimator to detect when the rocket begins to fall. Also checks if the altitude has dropped by a certain threshold from the highest recorded altitude.

- **Apogee Prediction:**  
  The [Apogee Predictor](include/state_estimation/ApogeePredictor.h) utilizes the vertical velocity estimator to predict the time until apogee and the maximum altitude.

### Data Logging

- **SPI Embedded Flash-based Logging:**  
  [DataSaverSPI](include/data_handling/DataSaverSPI.h) logs sensor data directly to any W25 SPI flash chip. Each sensor logs data at its own rate using the "Byte5" format, which is optimized for space efficiency while allowing for unordered data. Data is stored in a circular buffer until post-launch mode is activated.

- **SPI SD Card-based Logging:**  
  [DataSaverBigSD](include/data_handling/DataSaverBigSD.h) logs data to a large capacity SD card. Data is saved in a stream with less regard for space efficiency. Saves data in a stream format rather than a CSV. 

- **Serial-based Logging:**  
  [DataSaverSDSerial](include/data_handling/DataSaverSDSerial.h) provides an alternative logging mechanism by streaming binary data to a serial interface. Although less space-efficient than "Byte5", it is designed for applications with large SD cards. Refer to the [Serial-Logger-Decoding](https://github.com/CURocketEngineering/Serial-Logger-Decoding) repository for decoding instructions.

### Communication

- **Serial Command Handler:**  
  The [UARTCommandHandler](include/UARTCommandHandler.h) offers a fully configurable command interface for runtime configuration, debugging, and data retrieval, using function pointers for flexibility.

## Dependent Systems

The following systems integrate Avionics as a submodule:

- [MARTHA 1.3](https://github.com/CURocketEngineering/MARTHA-1.3)
- [Active-Aero](https://github.com/CURocketEngineering/Active-Aero)
- [MARTHA 1.1](https://github.com/CURocketEngineering/MARTHA-1.1)
- [Native](https://github.com/CURocketEngineering/Native)

## Unit Testing

Unit tests are managed by the [Native](https://github.com/CURocketEngineering/Native) repository, allowing for module testing without requiring embedded hardware.

## License

This project is licensed under the [MIT License](LICENSE). Please review the LICENSE file for complete details.