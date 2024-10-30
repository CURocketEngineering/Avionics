#include "FlashDriver.h"


SPIClass flashSpi(PB5, PB4, PB3, PA4);
FlashDriver::FlashDriver() {}

FlashDriver::~FlashDriver() {}

FlashStatus FlashDriver::initFlash() {

    if (!flashSpi.begin()) {
        return FLASH_FAILURE;
    }

    digitalWrite(PA4, LOW);


    flashSpi.transfer(tx_buf, 1);
    rx_buf[0] = flashSpi.transfer(0x00);
    rx_buf[1] = flashSpi.transfer(0x00);
    rx_buf[2] = flashSpi.transfer(0x00);
 

    digitalWrite(PA4, HIGH);

    // Extract Manufacturer ID, Memory Type, and Capacity from rx_buf
    uint8_t manufacturerID = rx_buf[0];  
    uint8_t memoryType = rx_buf[1];      
    uint8_t capacity = rx_buf[2];        

    if (manufacturerID == MANUFACTURER_ID && memoryType == MEMORY_TYPE && capacity == MEMORY_CAPACITY) {
        return FLASH_SUCCESS;
    } else {
        return FLASH_FAILURE;
    }
}

FlashStatus FlashDriver::readFlash(uint32_t address, uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) {
        return FLASH_INVALID;
    }

    uint8_t tx_buf[4] = {
        READ_FLASH,
        static_cast<uint8_t>((address >> 16) & 0xFF),
        static_cast<uint8_t>((address >> 8) & 0xFF),
        static_cast<uint8_t>(address & 0xFF)
    };


    digitalWrite(PA4, LOW);

    flashSpi.transfer(tx_buf, nullptr, sizeof(tx_buf));
    flashSpi.transfer(nullptr, buffer, length);

    flashSpi.endTransaction();
    digitalWrite(PA4, HIGH);

    return FLASH_SUCCESS;
}



FlashStatus FlashDriver::writeFlash(uint32_t address, const uint8_t* data, size_t length) {
    if (!data || length == 0) {
        return FLASH_INVALID;
    }

    while (length > 0) {
        uint32_t currentPage = address / PAGE_SIZE_BYTES;
        uint32_t offsetInPage = address % PAGE_SIZE_BYTES;
        uint32_t bytesToEndOfPage = PAGE_SIZE_BYTES - offsetInPage;
        uint32_t bytesToWrite = (length < bytesToEndOfPage) ? length : bytesToEndOfPage;

        uint8_t tx_buf[4] = {
            WRITE_ENABLE_FLASH,
            static_cast<uint8_t>((address >> 16) & 0xFF),
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF)
        };


        digitalWrite(PA4, LOW);

        flashSpi.transfer(tx_buf, nullptr, sizeof(tx_buf));
        flashSpi.transfer(data, nullptr, bytesToWrite);

        flashSpi.endTransaction();
        digitalWrite(PA4, HIGH);

        address += bytesToWrite;
        data += bytesToWrite;
        length -= bytesToWrite;

        writeDisable();
    }

    return FLASH_SUCCESS;
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


void FlashDriver::eraseSector(uint32_t address) {
    uint8_t tx_buf[4] = {
        SECTOR_ERASE,
        static_cast<uint8_t>((address >> 16) & 0xFF),
        static_cast<uint8_t>((address >> 8) & 0xFF),
        static_cast<uint8_t>(address & 0xFF)
    };


    digitalWrite(PA4, LOW);

    flashSpi.transfer(tx_buf, nullptr, sizeof(tx_buf));

    flashSpi.endTransaction();
    digitalWrite(PA4, HIGH);
}


void FlashDriver::eraseFlash(uint32_t address){

    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(ERASE_CHIP);                      
    flashSpi.endTransaction();
    digitalWrite(PA4, HIGH);
}


void FlashDriver::resetFlash() {
    uint8_t tx_buf[1] = {ENABLE_RESET};

    digitalWrite(PA4, LOW);

    flashSpi.transfer(tx_buf, nullptr, sizeof(tx_buf));

    flashSpi.endTransaction();
    digitalWrite(PA4, HIGH);

    tx_buf[0] = RESET_DEVICE;


    digitalWrite(FLASH_CS, LOW);
    flashSpi.beginTransaction();
    flashSpi.transfer(RESET_FLASH);                      
    flashSpi.endTransaction();
    digitalWrite(PA4, HIGH);
}


void FlashDriver::writeDisable() {
    uint8_t tx_buf[1] = {WRITE_DISABLE_FLASH};


    digitalWrite(PA4, LOW);

    flashSpi.transfer(tx_buf, nullptr, sizeof(tx_buf));

    flashSpi.endTransaction();
    digitalWrite(PA4, HIGH);
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