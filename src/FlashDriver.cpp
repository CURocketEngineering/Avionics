#include "FlashDriver.h"


FlashDriver::FlashDriver() {}

FlashDriver::~FlashDriver() {}

FlashStatus FlashDriver::initFlash() {

    uint8_t rx_buf[3] = {0}; 
    uint8_t tx_buf = FLASH_JEDEC_ID;   

    _spi->begin();

    digitalWrite(PB1, LOW);

    _spi->transfer(tx_buf, 1);
    rx_buf[0] = _spi->transfer(0x00);
    rx_buf[1] = _spi->transfer(0x00);
    rx_buf[2] = _spi->transfer(0x00);
 

    digitalWrite(PB1, HIGH);

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

    
    digitalWrite(PB1, LOW);
    _spi->transfer(tx_buf, nullptr, sizeof(tx_buf));
    _spi->transfer(nullptr, buffer, length);
    digitalWrite(PB1, HIGH);


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
            PAGE_PROGRAM,
            static_cast<uint8_t>((address >> 16) & 0xFF),
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF)
        };

        digitalWrite(PB1, LOW);
        _spi->transfer(WRITE_ENABLE_FLASH);
        digitalWrite(PB1, HIGH);

        digitalWrite(PB1, LOW);
        _spi->transfer(tx_buf, nullptr, sizeof(tx_buf));
        _spi->transfer(data, nullptr, bytesToWrite);
        digitalWrite(PB1, HIGH);

        if (!waitUntilNotBusy()) { 
            return FLASH_FAILURE;
        }


        address += bytesToWrite;
        data += bytesToWrite;
        length -= bytesToWrite;

        writeDisable();
    }
    

    return FLASH_SUCCESS;
}


uint8_t FlashDriver::readStatusReg1() {
    
    digitalWrite(PB1, LOW);
    _spi->transfer(READ_STATUS_REG1); 
    uint8_t status = _spi->transfer(0x00); 
    digitalWrite(PB1, HIGH);

    return status;
}

void FlashDriver::writeStatusReg1(uint8_t status) {
    
    digitalWrite(PB1, LOW);
    _spi->transfer(WRITE_ENABLE_FLASH);
    digitalWrite(PB1, HIGH);

    digitalWrite(PB1, LOW);
    _spi->transfer(WRITE_STATUS_REG1); 
    _spi->transfer(status);
    digitalWrite(PB1, HIGH);
    
}


uint32_t FlashDriver::getPageAddress(uint32_t page) {
    if (isValidPage(page) != FLASH_SUCCESS) {

        return 0xFFFFFF; 
    }
    return PAGE_ADDRESS(page);
}


uint32_t FlashDriver::getSectorAddress(uint32_t sector) {
    if (isValidSector(sector) != FLASH_SUCCESS) {

        return 0xFFFFFF; 
    }
    return SECTOR_ADDRESS(sector);
}


uint32_t FlashDriver::getBlockAddress(uint32_t block) {
    if (isValidBlock(block) != FLASH_SUCCESS) {

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

    
    digitalWrite(PB1, LOW);
    _spi->transfer(WRITE_ENABLE_FLASH);
    digitalWrite(PB1, HIGH);


    digitalWrite(PB1, LOW);
    _spi->transfer(tx_buf, nullptr, sizeof(tx_buf));
    digitalWrite(PB1, HIGH);

    writeDisable();
    
}


void FlashDriver::eraseFlash() {
    uint8_t tx_buf[1] = {CHIP_ERASE};

    
    digitalWrite(PB1, LOW);
    _spi->transfer(tx_buf, nullptr, sizeof(tx_buf));
    digitalWrite(PB1, HIGH);
    
}


void FlashDriver::resetFlash() {
    uint8_t tx_buf[1] = {ENABLE_RESET};

    
    digitalWrite(PB1, LOW);
    _spi->transfer(tx_buf, nullptr, sizeof(tx_buf));
    digitalWrite(PB1, HIGH);

    tx_buf[0] = RESET_DEVICE;

    digitalWrite(PB1, LOW);
    _spi->transfer(tx_buf, nullptr, sizeof(tx_buf));
    digitalWrite(PB1, HIGH);
    
}

void FlashDriver::sendUnlockCommand() {
    // Send the Global Block/Sector Unlock command
    
    digitalWrite(PB1, LOW);
    _spi->transfer(GLOBAL_UNLOCK_CMD);
    digitalWrite(PB1, HIGH);
    
}

void FlashDriver::writeDisable() {
    uint8_t tx_buf[1] = {WRITE_DISABLE_FLASH};

    digitalWrite(PB1, LOW);
    _spi->transfer(tx_buf, nullptr, sizeof(tx_buf));
    digitalWrite(PB1, HIGH);
}

bool FlashDriver::checkWriteEnable() {
    uint8_t status = readStatusReg1();

    // Check if WEL (bit 1) is set
    return (status & 0x02) != 0;
}

bool FlashDriver::waitUntilNotBusy() {
    uint8_t status;

    // Keep polling until BUSY (bit 0) is cleared
    do {
        status = readStatusReg1();
    } while (status & 0x01); 

    return true;  
}

FlashStatus FlashDriver::isValidPage(uint32_t page) {
    return (page < (BLOCK_COUNT * SECTOR_COUNT_BLOCK)) ? FLASH_SUCCESS : FLASH_INVALID; 
}

FlashStatus FlashDriver::isValidSector(uint32_t sector) {
    return (sector < SECTOR_COUNT) ? FLASH_SUCCESS : FLASH_INVALID; 
}

FlashStatus FlashDriver::isValidBlock(uint32_t block) {
   return (block < BLOCK_COUNT) ? FLASH_SUCCESS : FLASH_INVALID; 
}