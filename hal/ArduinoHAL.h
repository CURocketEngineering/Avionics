#ifndef ARDUINOHAL_H
#define ARDUINOHAL_H

#ifdef ARDUINO
#include "Arduino.h"

#include <Adafruit_SPIFlash.h>
#include <Adafruit_Sensor.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SPI.h>

#else // Everything below will only be compiled if we are not on an Arduino

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "Adafruit_SPIFlash_mock.h"
#include "serial_mock.h"
#include "spi_mock.h"

using String = std::string;

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

inline void pinMode(int pin, int mode) { //NOLINT
    // Do nothing
}

inline void digitalWrite(int pin, int value) { //NOLINT
    // Do nothing
}

inline void analogReadResolution(int bits) {
    // Do nothing, we will just return a 12-bit value in analogRead
}

inline uint32_t analogRead(int pin) {
    // Return a dummy 12 bit value for the voltage pin, and 0 for other pins. 
    // This allows us to test the battery voltage reading functionality without needing a real ADC.
    if (pin == 192) {
        return 4090;
    }
    if (pin == 193) { // fake pin for testing the low voltage case
        return 0;
    }
    return 0; // Default dummy value for other pins
}

// millis mock which still gives us the time since the program started in milliseconds
static auto program_start = std::chrono::high_resolution_clock::now();
inline unsigned long millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - program_start).count();
}

inline void delay(unsigned long ms) { // NOLINT
    // Wait using the real time clock
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


#endif // ARDUINO
#endif // ARDUINOHAL_H
