#ifndef ADAFRUIT_SPIFLASH_MOCK_H
#define ADAFRUIT_SPIFLASH_MOCK_H

#include <stddef.h>
#include <stdint.h>

#define FAKE_MEMORY_SIZE_BYTES 16777216 // 16MB
#define SFLASH_SECTOR_SIZE 4096
#define SFLASH_BLOCK_SIZE 65536
#define SFLASH_PAGE_SIZE 256

class Adafruit_SPIFlash {
    public:

    Adafruit_SPIFlash() {};

    bool begin() {
        return true;
    }

    bool size() {
        return FAKE_MEMORY_SIZE_BYTES;
    }

    bool writeBuffer(uint32_t address, const uint8_t* buffer, size_t length) {
        // Write to fake memory
        for (size_t i = 0; i < length; i++) {
            fakeMemory[address + i] = buffer[i];
        }
        // if out of bounds, return false
        if (address + length > FAKE_MEMORY_SIZE_BYTES) {
            return false;
        }

        return true;
    }

    bool readBuffer(uint32_t address, uint8_t* buffer, size_t length) {
        // Read from fake memory
        for (size_t i = 0; i < length; i++) {
            buffer[i] = fakeMemory[address + i];
        }

        // if out of bounds, return false
        if (address + length > FAKE_MEMORY_SIZE_BYTES) {
            return false;
        }

        return true;
    }

    bool eraseSector(uint32_t sectorNumber) {
        // Erase the sector in fake memory
        for (size_t i = 0; i < 4096; i++) {
            fakeMemory[sectorNumber * 4096 + i] = 0xFF;
        }
        return true;
    }

    bool eraseChip() {
        // Erase the entire chip in fake memory
        for (size_t i = 0; i < FAKE_MEMORY_SIZE_BYTES; i++) {
            fakeMemory[i] = 0xFF;
        }
        return true;
    }

    uint8_t fakeMemory[FAKE_MEMORY_SIZE_BYTES];
};

#endif // ADAFRUIT_SPIFLASH_MOCK_H