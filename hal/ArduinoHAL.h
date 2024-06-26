#ifndef ARDUINOHAL_H
#define ARDUINOHAL_H

#include <string>

#ifdef ARDUINO
#include "Arduino.h"
#else // Everything below will only be compiled if we are not on an Arduino

#include <chrono>

// Within here we must define the mock functions for the Arduino functions
#include "serial_mock.h"

// millis mock which still gives us the time since the program started in milliseconds
static auto program_start = std::chrono::high_resolution_clock::now();
unsigned long millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - program_start).count();
}


#endif // ARDUINO
#endif // ARDUINOHAL_H