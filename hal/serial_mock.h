// Mocks for all serial related Arduino functions

#pragma once

#include <iostream>
#include "stdio.h"
#include <cstdarg>
#include <gmock/gmock.h>

class MockSerial {
  public:
    
    template<typename T>
    void print(const T& message) {
        std::cout << message;
    }

    template<typename T>
    void println(const T& message) {
        std::cout << message << std::endl;
    }

    // printf
    void printf(const char *fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }

};

MockSerial Serial;

class MockHardwareSerial : public MockSerial{
    
};

typedef MockHardwareSerial HardwareSerial;