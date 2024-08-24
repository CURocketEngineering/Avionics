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

// Define all the SPI function to do nothing
class MockSPI{
    public:
        static void begin(){}
        static void end(){}
        static void beginTransaction(SPISettings s){}
        static void transfer(int a){}
};

MockSPI SPI; // This is the mock SPI object



#endif