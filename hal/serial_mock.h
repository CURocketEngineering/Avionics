// Mocks for all serial related Arduino functions

#pragma once

#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <cstdarg>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class MockSerial {
public:
    // Store the calls and their parameters
    std::vector<std::string> printCalls;
    std::vector<std::string> printlnCalls;
    std::vector<std::string> printfCalls;

    template<typename T>
    void print(const T& message) {
        std::ostringstream oss;
        oss << message;
        printCalls.push_back(oss.str());
        std::cout << message;
    }

    template<typename T>
    void println(const T& message) {
        std::ostringstream oss;
        oss << message;
        printlnCalls.push_back(oss.str());
        std::cout << message << std::endl;
    }

    // printf
    void printf(const char *fmt, ...) {
        char buffer[256];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        printfCalls.push_back(buffer);
        std::cout << buffer;
    }

    // Clear the stored calls
    void clear() {
        printCalls.clear();
        printlnCalls.clear();
        printfCalls.clear();
    }

    // write
    size_t write(uint8_t) { return 0; }
    size_t write(const uint8_t *buffer, size_t size) { return size; }
};

extern MockSerial Serial;
extern MockSerial Serial1;

typedef MockSerial HardwareSerial;


// Mock Stream class
class Stream {
public:
    std::vector<uint8_t> writeCalls;

    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
    virtual size_t write(uint8_t byte) {
        this->writeCalls.push_back(byte);
        return 0;
    }
    virtual size_t write(const char *str) { return 0; }   
    virtual size_t write(const char* buffer, size_t size) { return size; } // for string literals
    virtual size_t write(const uint8_t *buffer, size_t size) { return size; }
    void clearWriteCalls() {
        writeCalls.clear();
    }

    // Read a string until a newline character
    std::string readStringUntil(char terminator) {
        std::string result;
        char c;
        while ((c = read()) != terminator && c != -1) {
            result += c;
        }
        return result;
    }

    template<typename T>
    void print(const T& message) {
        std::cout << message;
    }

    template<typename T>
    void println(const T& message) {
        std::cout << message << std::endl;
    }

    void println() {
        std::cout << std::endl;
    }

    // Read a string
    std::string readString() {
        std::string result;
        char c;
        while ((c = read()) != -1) {
            result += c;
        }
        return result;
    }

    void println(const char* message) {
        std::cout << message << std::endl;
    }

    size_t availableForWrite() {
        return 64; // Simulate a TX buffer with 64 bytes of available space
    }
};