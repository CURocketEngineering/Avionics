# hal Directory

The HAL directory defines a hardware abstraction layer (HAL) which sits between Avionics and the Arduino core platform. This layer allows Avionics to be tested in the host-native test environment on a laptop without any Arduino dependencies.

## The RULE

Avionics must never `#include <Arduino.h>` or any other Arduino-specific headers. Instead you must include `ArduinoHAL.h` from this directory which will either pull in the real Arduino core (when compiling for an Arduino target) or a mock implementation (when compiling for host-native tests).

If you ever try to include an Arduino header directly in Avionics, then host-native tests will fail to compile because they do not have access to Arduino headers.

## How it works

When you include `ArduinoHAL.h`, it checks if the `ARDUINO` macro is defined. If it is, then it includes the real Arduino core headers. If not, then it includes mock implementations of the Arduino functions and classes that Avionics uses. It is meant to be a drop-in replacement for the Arduino core. 
Things have been added to the ArduinoHAL mock as needed to support Avionics host-native unit tests. If you find that a function or class is missing from the mock, you can add it to the mock implementation files in this directory.

For example, the `serial_mock.h` defines a `Serial` object that mimics the Arduino `Serial` object. It has the same methods (e.g., `begin`, `print`, `println`, etc.) but they just print to the stdout instead of a real serial port.

`ArduinoHAL.h` is the entry point and will include other headers in this directory as needed.
