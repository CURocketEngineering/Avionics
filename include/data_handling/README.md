# data_handling Directory

Tools for collecting, rate-limiting, persisting, and downlinking sensor data.

## Files
- `CircularArray.h`: Fixed-size circular buffer for recent samples with quickselect-based median support.
- `DataNames.h`: List of 8-bit integer constants that identify each data channel for both data logging and telemetry purposes. This must stay in sync with the ground station's data names YAML file. 
- `DataPoint.h`: Lightweight class that holds a single float with a timestamp. Instead of throwing raw floats around, we use `DataPoint` to keep track of when samples were taken which allows for better filters to be used in the `state_estimation` side of tools. If you have a list of float's you don't know when they were take, a list of `DataPoint`'s is preferred.
- `DataSaver.h`: Abstract `IDataSaver` interface plus convenience overloads and hooks for initialization and launch events.
- `DataSaverBigSD.h`: Buffered CSV logger to large SD cards via SdFat, batching writes and managing stream file paths.
- `DataSaverPrint.h`: `IDataSaver` that prints channel/timestamp/value to stdout for debugging and tests.
- `DataSaverSDSerial.h`: Streams CSV-formatted samples over UART to an external serial data logger.
- `DataSaverSPI.h`: SPI flash logger with timestamp compression, post-launch write protection, and dump/erase utilities. Use this to write to an onboard flash chip with very little storage space. This is the most space-efficient data saver we have, but it is also the most complex to use.
- `SensorDataHandler.h`: Buffers sensor samples, enforces minimum save intervals, and forwards data to an `IDataSaver`.
- `Telemetry.h`: Builds fixed-size packets from `SensorDataHandler` streams and transmits them over UART at set frequencies.
