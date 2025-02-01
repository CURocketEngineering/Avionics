# Avionics
 
Arduino-Core based avionics systems to be used across multiple flight computers throughout Clemson University Rocket Engineering

## Dependent Systems

The following systems use Avionics as a submodule:
- [MARTHA 1.3](https://github.com/CURocketEngineering/MARTHA-1.3)
- [Active-Aero](https://github.com/CURocketEngineering/Active-Aero)
- [MARTHA 1.1](https://github.com/CURocketEngineering/MARTHA-1.1)
- [Native](https://github.com/CURocketEngineering/Native)

## 1. Features

### 1.1. State Detection
- [**Launch Detection**](include/data_handling/LaunchPredictor.h):
Uses a rolling median of acceleration data to detect launch. Highly configurable to account for different launch profiles. Enforces strict
data rate requirements to ensure accurate detection. Contains error-handling.

- [**Kalman Filter**](include/kf-2d.h):
Fuses accelerometer and altimeter data to predict vertical velocity.

### 1.2. Data Logging
- [**SPI Flash-based logging (W25)**](include/data_handling/DataSaverSPI.h):
Automatically logs data to any W25 SPI flash chip.
Each sensor can log data at different rates.
Saves in our "Byte5" format for maximum space efficiency, 
while allowing total data order flexibility. Data is 
continuously overwritten circularly until "post-launch" mode is triggered.

- [**Serial-based logging**](include/data_handling/DataSaverSDSerial.h):
Automatically logs data to a serial stream.
Each sensor can log data at different rates.
Saved in a binary format for space efficiency.
See [Serial-Logger-Decoding](https://github.com/CURocketEngineering/Serial-Logger-Decoding) for decoding scripts. 
Not as space-efficient as our "Byte5" format, but was
designed with large SD cards in mind.

### 1.3. Communication
- [**Serial Command Handler**](include/UARTCommandHandler.h):
Can host a serial-based command handler on device which
can be used for runtime configuration and debugging as well as
data retrieval. Completely configurable using function pointers.

- [**CC1125 Transceiver**](include/CC1125.h):
Can communicate via an embedded CC1125 transceiver directly with SPI.




## Testing

### 1. Unit Testing

Unit testing is handled by the [Native](https://github.com/CURocketEngineering/Native) repository. It allows us
to test these modules without embedded hardware.

### 2. Flight Tests

Below is a list of all rocket flights which used this system.
The top is the most recent flight.

- [MARTHA 1.1 Nov 9th L1 Cert Launch](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.1.0)
    Successfully detected launch and recorded flight data to the SD card. No altitude data was recorded due to either faulty drivers or the MPL3115A2 sensor being damaged.

- [MARTHA 1.1 Spaceport 2024](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.0.0)

    Battery died while on the pad. Data was recorded via the serial logger, but nothing interesting was recorded. No altitude data was recorded due to either faulty drivers or the MPL3115A2 sensor being damaged.