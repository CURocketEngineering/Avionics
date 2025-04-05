#ifndef ARDUINOHAL_H
#define ARDUINOHAL_H

#ifdef ARDUINO
#include "Arduino.h"
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_Sensor.h>
#include <FS.h>
#include <SD.h>

#ifdef FILE_READ
#undef FILE_READ
#endif
#ifdef FILE_WRITE
#undef FILE_WRITE
#endif

#include <SdFat.h>
#else // Everything below will only be compiled if we are not on an Arduino

#include <string>
using String = std::string;

#include <iostream>

#include <chrono>

#include "spi_mock.h"

// Within here we must define the mock functions for the Arduino functions
#include "serial_mock.h"
#include "Adafruit_SPIFlash_mock.h"

#include <thread>

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

inline void pinMode(int pin, int mode) {
    // Do nothing
}

inline void digitalWrite(int pin, int value) {
    // Do nothing
}


// millis mock which still gives us the time since the program started in milliseconds
static auto program_start = std::chrono::high_resolution_clock::now();
inline unsigned long millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - program_start).count();
}

inline void delay(unsigned long ms) {
    // Wait using the real time clock
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


#endif // ARDUINO
#endif // ARDUINOHAL_H