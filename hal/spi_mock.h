#ifndef SPI_MOCK_H
#define SPI_MOCK_H

#include <array>
#include <cstdint>

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
};

// To allow for the use of the SPI class with a line like `SPIClass flashSpi;`
class SPIClass : public MockSPIClass {
public:
    SPIClass(uint32_t& mosi, uint32_t& miso, uint32_t& sclk, uint32_t& ssel)
        : MockSPIClass(mosi, miso, sclk, ssel) {} // Forwarding constructor
    SPIClass() : MockSPIClass() {} // Forwarding constructor
};

extern SPIClass SPI; // This is the mock SPI object

#endif