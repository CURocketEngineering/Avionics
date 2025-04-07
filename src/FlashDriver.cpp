#include "FlashDriver.h"

/* ------------------------------------------------------------------ */
/*  Init ‑‑ single‑byte writes are still fine with transfer(uint8_t)  */
FlashStatus FlashDriver::initFlash()
{
    uint8_t rx_buf[3] = {0};
    uint8_t cmd = FLASH_JEDEC_ID;

    _spi->begin();

    digitalWrite(_cs, LOW);
    _spi->transfer(cmd);                 // send command
    rx_buf[0] = _spi->transfer(0x00);    // read 3‑byte ID
    rx_buf[1] = _spi->transfer(0x00);
    rx_buf[2] = _spi->transfer(0x00);
    digitalWrite(_cs, HIGH);

    if (rx_buf[0] == MANUFACTURER_ID &&
        rx_buf[1] == MEMORY_TYPE     &&
        rx_buf[2] == MEMORY_CAPACITY)
        return FLASH_SUCCESS;

    return FLASH_FAILURE;
}

/* ------------------------------------------------------------------ */
FlashStatus FlashDriver::readFlash(uint32_t address,
                                   uint8_t * buffer,
                                   size_t    length)
{
    if (!buffer || !length) return FLASH_INVALID;

    uint8_t hdr[4] = {
        READ_FLASH,
        uint8_t(address >> 16),
        uint8_t(address >> 8),
        uint8_t(address)
    };

    digitalWrite(_cs, LOW);
    _spi->writeBytes(hdr, sizeof(hdr));                // TX header
    _spi->transferBytes(nullptr, buffer, length);      // read payload
    digitalWrite(_cs, HIGH);

    return FLASH_SUCCESS;
}

/* ------------------------------------------------------------------ */
FlashStatus FlashDriver::writeFlash(uint32_t      address,
                                    const uint8_t *data,
                                    size_t         length)
{
    if (!data || !length) return FLASH_INVALID;

    while (length) {
        uint32_t offset      = address % PAGE_SIZE_BYTES;
        uint32_t space       = PAGE_SIZE_BYTES - offset;
        uint32_t chunk       = (length < space) ? length : space;

        /* --- Write‑enable --- */
        digitalWrite(_cs, LOW);
        _spi->transfer(WRITE_ENABLE_FLASH);
        digitalWrite(_cs, HIGH);

        /* --- PAGE PROGRAM header --- */
        uint8_t hdr[4] = {
            PAGE_PROGRAM,
            uint8_t(address >> 16),
            uint8_t(address >> 8),
            uint8_t(address)
        };

        digitalWrite(_cs, LOW);
        _spi->writeBytes(hdr, sizeof(hdr));            // header
        _spi->writeBytes(data, chunk);                 // payload
        digitalWrite(_cs, HIGH);

        if (!waitUntilNotBusy()) return FLASH_FAILURE;

        address += chunk;
        data    += chunk;
        length  -= chunk;

        writeDisable();
    }
    return FLASH_SUCCESS;
}

/* ------------------------------------------------------------------ */
void FlashDriver::eraseSector(uint32_t address)
{
    uint8_t hdr[4] = {
        SECTOR_ERASE,
        uint8_t(address >> 16),
        uint8_t(address >> 8),
        uint8_t(address)
    };

    digitalWrite(_cs, LOW);
    _spi->transfer(WRITE_ENABLE_FLASH);
    digitalWrite(_cs, HIGH);

    digitalWrite(_cs, LOW);
    _spi->writeBytes(hdr, sizeof(hdr));
    digitalWrite(_cs, HIGH);

    writeDisable();
}

/* ------------------------------------------------------------------ */
void FlashDriver::eraseFlash()
{
    const uint8_t cmd = CHIP_ERASE;

    digitalWrite(_cs, LOW);
    _spi->writeBytes(&cmd, 1);
    digitalWrite(_cs, HIGH);
}

/* ------------------------------------------------------------------ */
void FlashDriver::resetFlash()
{
    uint8_t cmd = ENABLE_RESET;

    digitalWrite(_cs, LOW);
    _spi->writeBytes(&cmd, 1);
    digitalWrite(_cs, HIGH);

    cmd = RESET_DEVICE;
    digitalWrite(_cs, LOW);
    _spi->writeBytes(&cmd, 1);
    digitalWrite(_cs, HIGH);
}

/* ------------------------------------------------------------------ */
void FlashDriver::writeDisable()
{
    const uint8_t cmd = WRITE_DISABLE_FLASH;

    digitalWrite(_cs, LOW);
    _spi->writeBytes(&cmd, 1);
    digitalWrite(_cs, HIGH);
}
