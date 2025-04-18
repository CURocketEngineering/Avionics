#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include "ArduinoHAL.h" // Includes SPI.h when running on real hardware

#define FLASH_JEDEC_ID          0x9F
#define MANUFACTURER_ID         0xEF
#define MEMORY_TYPE             0x40
#define MEMORY_CAPACITY         0x18
#define SECTOR_SIZE             0x1000
#define SECTOR_COUNT_BLOCK      16
#define SECTOR_COUNT            4096
#define BLOCK_SIZE              0x10000
#define BLOCK_COUNT             256
#define PAGE_SIZE_BYTES         256
#define BUSY_STATUS_REG         0x05
#define FLASH_BUSY              1
#define SECTOR_ERASE            0x20  
#define CHIP_ERASE              0x60
#define ENABLE_RESET            0x66
#define RESET_DEVICE            0x99
#define WRITE_DISABLE_FLASH     0x04
#define WRITE_ENABLE_FLASH      0x06 
#define READ_FLASH              0x03 
#define PAGE_PROGRAM            0x02
#define GLOBAL_UNLOCK_CMD       0x98 
#define READ_STATUS_REG1        0x05
#define WRITE_STATUS_REG1       0x01

// Calculate the address of a specific page
#define PAGE_ADDRESS(page)     ((page) * PAGE_SIZE_BYTES)

// Calculate the address of a specific sector
#define SECTOR_ADDRESS(sector)  ((sector) * SECTOR_SIZE)

// Calculate the address of a specific block
#define BLOCK_ADDRESS(block)    ((block) * BLOCK_SIZE)



enum FlashStatus {
    FLASH_SUCCESS,
    FLASH_FAILURE,
    FLASH_INVALID
};

class FlashDriver {

public:
    FlashDriver(uint32_t cs);  // Constructor
    ~FlashDriver(); // Destructor

    FlashStatus initFlash();
    FlashStatus readFlash(uint32_t address, uint8_t* buffer, size_t length);
    FlashStatus writeFlash(uint32_t address, const uint8_t* data, size_t length);
    void writeStatusReg1(uint8_t status);
    uint8_t readStatusReg1();
    uint32_t getPageAddress(uint32_t page);
    uint32_t getSectorAddress(uint32_t sector);
    uint32_t getBlockAddress(uint32_t block);
    void eraseSector(uint32_t address);
    void eraseFlash();
    void resetFlash();
    void sendUnlockCommand();

    
private:

    SPIClass *_spi = &SPI;
    uint32_t _cs;

    void writeDisable();
    bool checkWriteEnable();
    bool waitUntilNotBusy();
    FlashStatus isValidPage(uint32_t page);
    FlashStatus isValidSector(uint32_t sector);
    FlashStatus isValidBlock(uint32_t block);

};

#endif // FLASH_DRIVER_H