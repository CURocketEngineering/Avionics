# Avionics for Clemson University Rocket Engineering

Avionics is a modular, Arduino-core based system powering the flight computers of Clemson University Rocket Engineering. Designed for high reliability and efficiency, it supports real-time state estimation, data logging, and robust communication across multiple rockets.

**Documentation Website: 🚀[Avionics Docs](https://curocketengineering.github.io/Avionics/)🚀**

## Table of Contents

- [Introduction](#introduction)
- [Project Goals](#project-goals)
- [Architecture Overview](#architecture-overview)
- [Features](#features)
- [Dependent Systems](#dependent-systems)
- [Documentation](#documentation)
- [Testing](#testing)
  - [Unit Testing](#unit-testing)
  - [Flight Tests](#flight-tests)

## Introduction

This repository contains the avionics system used across various flight computers within the Clemson University Rocket Engineering organization. Designed to be modular and highly configurable, the system supports critical functions such as state detection, data logging, and robust communication.

## Project Goals

- Deliver a reusable avionics toolbox that can be utilized across multiple rocket platforms.
- Prioritize flight safety through redundant sensing, conservative state transitions, and explicit error handling.
- Maintain traceable telemetry and post-flight data for analysis and certification.
- Enable rapid iteration with simulation hooks, configurable command interfaces, and host-side unit testing.

## Architecture Overview

- **State estimation:** Launch/apogee detection, burnout/landing state machines, and vertical velocity filtering to gate deployment logic.
- **Data handling:** SPI flash/SD logging, serial streaming, and documented Byte5 binary format for efficient on-vehicle storage (see `docs/FlashDataSaving.md`).
- **Communication:** UART command handler for runtime configuration, telemetry, and debugging without reflashing firmware.
- **Power and platform:** Power management utilities plus HAL abstractions to keep sensor/MCU specifics isolated from flight logic.
- **Simulation:** Serial-driven sensor simulators (e.g., LSM6DSOX, LIS3MDL, BMP390) to inject prerecorded data for hardware-in-the-loop testing.

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

## Documentation

- Doxygen: run `doxygen Doxyfile` from the repository root. HTML output is placed in `build/doxygen/index.html` (README is used as the main page). You can then use Python to serve the files locally: `python -m http.server --directory build/doxygen 8000` and navigate to `http://localhost:8000` in your browser. We have a workflow that will autogenerate and deploy the docs to GitHub Pages on each push to main.
- High-level notes: data logging constraints and Byte5 format are summarized in `docs/FlashDataSaving.md`.
- Flight history: see `docs/FlightTests.md` for vehicles and dates that have flown with this codebase.
- Target Audience: The documentation is written for compsci students with very little background in C++ or embedded systems. 

## Unit Testing

Unit tests are managed by the [Native](https://github.com/CURocketEngineering/Native) repository, allowing for module testing without requiring embedded hardware.

Never in the Avionics repo should you `#include <Arduino.h>` or any other Arduino-specific headers. Instead, always include `ArduinoHAL.h` from the `hal` directory, which will either pull in the real Arduino core (when compiling for an Arduino target) or a mock implementation (when compiling for Native). This ensures that Avionics can be tested via the Native repo on a laptop without any Arduino dependencies.

## License

This project is licensed under the [MIT License](LICENSE). Please review the LICENSE file for complete details.
