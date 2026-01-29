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
