#include "FlashDriver.h"

 // FLASH_CS is the Chip Select pin
 // 100000 is the clock speed (100kHz)
 // SPI_BITORDER_MSBFIRST sets the bit order
 // SPI_MODE1 sets the SPI mode
FlashDriver::FlashDriver() : flashSpi(FLASH_CS, 100000, SPI_BITORDER_MSBFIRST, SPI_MODE1) {}

FlashDriver::~FlashDriver() {}

FlashStatus FlashDriver::initFlash() {

    if (!flashSpi.begin()) {
        return FAILURE;
    }

    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(FLASH_ID_ADDR);  

    // Read Manufacturer ID and Device ID
    uint8_t manufacturerID = flashSpi.transfer(0x00);  // Read Manufacturer ID
    uint8_t memoryType = flashSpi.transfer(0x00);      // Read Memory Type (should be 0x40 for W25Q128JV)
    uint8_t capacity = flashSpi.transfer(0x00);        // Read Memory Capacity (should be 0x18 for W25Q128JV)


    flashSpi.endTransaction();
    digitalWrite(FLASH_CS, HIGH);

    if (manufacturerID == MANUFACTURER_ID && memoryType == MEMORY_TYPE && capacity == MEMORY_CAPACITY) {
        return SUCCESS;
    } else {
        return FAILURE;
    }
}

FlashStatus FlashDriver::readFlash(uint32_t address, uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) {
        return INVALID;
    }

    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(READ_FLASH);                    // Send the Read command (0x03)    
    flashSpi.transfer((address >> 16) & 0xFF);
    flashSpi.transfer((address >> 8) & 0xFF);
    flashSpi.transfer(address & 0xFF);

    for (size_t i = 0; i < length; i++) {
        buffer[i] = flashSpi.transfer(0x00);
    }

    flashSpi.endTransaction();
    digitalWrite(FLASH_CS, HIGH);

    return SUCCESS;
}


FlashStatus FlashDriver::writeFlash(uint32_t address, const uint8_t* data, size_t length) {
    if (!data || length == 0) {
        return INVALID;
    }

    while(length > 0){

        uint32_t currentPage = address / PAGE_SIZE_BYTES;
        uint32_t offsetInPage = address % PAGE_SIZE_BYTES;
        uint32_t bytesToEndOfPage = PAGE_SIZE_BYTES - offsetInPage;
        uint32_t bytesToWrite = (length < bytesToEndOfPage) ? length : bytesToEndOfPage;

        digitalWrite(FLASH_CS, LOW);
        flashSpi.beginTransaction();
        flashSpi.transfer(WRITE_ENABLE_FLASH);                      // Send the Write command (0x06)
        flashSpi.transfer((address >> 16) & 0xFF);
        flashSpi.transfer((address >> 8) & 0xFF);
        flashSpi.transfer(address & 0xFF);

        for (size_t i = 0; i < bytesToWrite; i++) {
            flashSpi.transfer(data[i]);
        }

        flashSpi.endTransaction();
        digitalWrite(FLASH_CS, HIGH);

        // Move to the next page
        address += bytesToWrite;
        data += bytesToWrite;
        length -= bytesToWrite;

        writeDisable();

    }

    return SUCCESS;
}


uint32_t getPageAddress(uint32_t page) {
    if (isValidPage(page) != SUCCESS) {

        return 0xFFFFFF; 
    }
    return PAGE_ADDRESS(page);
}


uint32_t getSectorAddress(uint32_t sector) {
    if (isValidSector(sector) != SUCCESS) {

        return 0xFFFFFF; 
    }
    return SECTOR_ADDRESS(sector);
}


uint32_t getBlockAddress(uint32_t block) {
    if (isValidBlock(block) != SUCCESS) {

        return 0xFFFFFF; 
    }
    return BLOCK_ADDRESS(block);
}


void FlashDriver::eraseSector(uint32_t address){

    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(SECTOR_ERASE);                      
    flashSpi.transfer((address >> 16) & 0xFF);
    flashSpi.transfer((address >> 8) & 0xFF);
    flashSpi.transfer(address & 0xFF);
    flashSpi.endTransaction();
    digitalWrite(FLASH_CS, HIGH);

}

void FlashDriver::eraseFlash(uint32_t address){

    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(ERASE_CHIP);                      
    flashSpi.endTransaction();
    digitalWrite(FLASH_CS, HIGH);

}

void FlashDriver::resetFlash(){

    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(ENABLE_RESET);                      
    flashSpi.endTransaction();
    digitalWrite(FLASH_CS, HIGH);

    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(RESET_FLASH);                      
    flashSpi.endTransaction();
    digitalWrite(FLASH_CS, HIGH);

}


void FlashDriver::writeDisable() {
    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(WRITE_DISABLE_FLASH);     // Send the Write Diable command (0x04)
    flashSpi.endTransaction();
    digitalWrite(FLASH_CS, HIGH);
}

FlashStatus FlashDriver::isValidPage(uint32_t page) {
    return (page < (BLOCK_COUNT * SECTOR_COUNT_BLOCK)) ? SUCCESS : INVALID; 
}

FlashStatus FlashDriver::isValidSector(uint32_t sector) {
    return sector < SECTOR_COUNT ? SUCCESS : INVALID; 
}

FlashStatus FlashDriver::isValidBlock(uint32_t block) {
   return block < BLOCK_COUNT ? SUCCESS : INVALID; 
}







