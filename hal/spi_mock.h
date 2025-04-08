#ifndef SPI_MOCK_H
#define SPI_MOCK_H

#include <array>
#include <cstdint>
#include <cstddef>

#define MSBFIRST 0
#define SPI_MODE0 0

class SPISettings{
    public:
        SPISettings(int a, int b, int c){}
};

// TODO: Add vectors to keep track of what data is being sent for better testing
class MockSPIClass{
    public:
        MockSPIClass(uint32_t& mosi, uint32_t& miso, uint32_t& sclk, uint32_t& ssel){} // Do nothing constructor
        MockSPIClass(){} // Do nothing constructor
        static void begin(){}
        static void end(){}
        static void beginTransaction(SPISettings s){}
        static bool transfer(uint8_t& data, int length) { return true; } // Do nothing transfer
        static bool transfer(uint8_t data) { return true; } // Do nothing transfer
        static bool transfer(uint8_t* data, std::nullptr_t np, int length) { return true; } // Do nothing transfer
        static bool transfer(uint8_t* a, uint8_t*& b, size_t& c) { return true; } // Do nothing transfer
        static bool transfer(const uint8_t* data, std::nullptr_t np, int length) { return true; } // Do nothing transfer
        static bool transfer(uint8_t* a, uint8_t b, int c) { return true; } // New overload for int
        static bool transfer(uint8_t* a, uint8_t* b, int c) { return true; } // New overload for int
        static bool transfer(uint8_t* a, uint8_t* b, int* c) { return true; } // New overload for int
        static bool writeBytes(const uint8_t* data, int length) { return true; } // Do nothing transfer
        static bool writeBytes(uint8_t* data, uint8_t* b, int length) { return true; } // Do nothing transfer
        static bool writeBytes(uint8_t* data, uint8_t* b, int* length) { return true; } // Do nothing transfer
        static bool transferBytes(uint8_t* data, uint8_t* b, int length) { return true; } // Do nothing transfer
        static bool transferBytes(uint8_t* data, uint8_t* b, int* length) { return true; } // Do nothing transfer
};

// To allow for the use of the SPI class with a line like `SPIClass flashSpi;`
class SPIClass : public MockSPIClass {
public:
    SPIClass(uint32_t& mosi, uint32_t& miso, uint32_t& sclk, uint32_t& ssel)
        : MockSPIClass(mosi, miso, sclk, ssel) {} // Forwarding constructor
    SPIClass() : MockSPIClass() {} // Forwarding constructor
};

extern SPIClass SPI; // This is the mock SPI object


// Define SdFile_t 
// This is a mock definition of the SdFile_t type
// In the actual code, this would be defined in the SdFat library
// In this mock, we will just use a simple struct to represent the file
struct File32 {
    // Add any necessary members or methods for the mock
    bool open(const char* path, int mode) {
        // Mock implementation of opening a file
        return true; // Assume success
    }
    void close() {
        // Mock implementation of closing a file
    }
    bool exists(const char* path) {
        // Mock implementation of checking if a file exists
        return false; // Assume file does not exist
    }

    // Override ! operator bool() to simulate file open success
    operator bool() const {
        return true; // Assume file is open
    }
    void print(int value) {
        // Mock implementation of printing an integer
    }
    void print(const char* str) {
        // Mock implementation of printing a string
    }
    void println(float value) {
        // Mock implementation of printing a float
    }
    void println(const char* str) {
        // Mock implementation of printing a string
    }

};

using SdFile_t = File32; // Use the mock File32 as the SdFile_t type

// Define the SdFat class
// This is a mock definition of the SdFat class
// In the actual code, this would be defined in the SdFat library
// In this mock, we will just use a simple struct to represent the SdFat class
struct SdFat {
    // Add any necessary members or methods for the mock
    bool begin(uint8_t csPin, int speed) {
        // Mock implementation of beginning the SD card
        return true; // Assume success
    }
    bool exists(const char* path) {
        // Mock implementation of checking if a file exists
        return false; // Assume file does not exist
    }

    SdFile_t open(const char* path, int mode) {
        // Mock implementation of opening a file
        return SdFile_t(); // Return a mock file object
    }
};


// Define O_WRITE, O_CREAT, and O_APPEND
// These are mock definitions of the file open modes
#define O_WRITE 0x01
#define O_CREAT 0x02
#define O_APPEND 0x04
#define SD_SCK_MHZ(x) (x) // Mock definition for SD_SCK_MHZ

#endif