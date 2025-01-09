#ifndef DATACOMPRESSOR_H
#define DATACOMPRESSOR_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include "FlashDriver.h"
#include <array>
#include <cstdlib>

class DataSaverSPI : public IDataSaver {
public:
    /**
     * @brief Construct a new DataSaverSPI object
     * 
     * @param timestampInterval_ms  Interval in ms at which timestamps are stored
     * @param mosi                  SPI MOSI pin
     * @param miso                  SPI MISO pin
     * @param sck                   SPI SCK  pin
     * @param cs                    SPI CS   pin
     *
     * @note timestampInterval_ms must be < 65535 (about 1 minute)
     */
    DataSaverSPI(uint16_t timestampInterval_ms, int mosi, int miso, int sck, int cs);

    /**
     * @brief Saves a DataPoint to SPI flash
     * 
     * If the new data point’s timestamp differs from the last written
     * timestamp by more than timestampInterval_ms, the new timestamp is also
     * written to flash. Otherwise, the timestamp is omitted (to save space).
     * 
     * @param dp   The data point to save
     * @param name An 8-bit “identifier” for the data point (could be a sensor ID)
     * @return int 0 on success; non-zero on error
     */
    virtual int saveDataPoint(DataPoint dp, uint8_t name) override;

    virtual bool begin() override;

    /**
     * @brief Dump all stored data points to some output
     * 
     * Reads data back from flash starting at address 0 until all valid data
     * has been read. You can adjust this method to print to Serial, or copy
     * into a buffer, etc.
     */
    void dumpData();

    /**
     * @brief Clears/erases the entire flash chip to start fresh
     */
    void eraseAllData();

    /**
     * @brief Returns the last timestamp that was actually written to flash
     */
    uint32_t getLastTimestamp() const {
        return lastTimestamp_ms;
    }

    /**
     * @brief Returns the last DataPoint that was written (not necessarily
     *        including timestamp, just the data chunk).
     */
    DataPoint getLastDataPoint() const {
        return lastDataPoint;
    }

private:
    // Interval at which to store the timestamp in flash
    uint16_t timestampInterval_ms;

    // The last timestamp we actually wrote to flash
    uint32_t lastTimestamp_ms;

    // The last data point written
    DataPoint lastDataPoint;

    // SPI pins
    int mosiPin;
    int misoPin;
    int sckPin;
    int csPin;

    // The underlying flash driver
    FlashDriver flash;

    // Next address in flash at which to write
    uint32_t nextWriteAddress;

private:
    /**
     * @brief Helper to write a block of bytes to flash at the current
     *        nextWriteAddress and advance that pointer.
     */
    bool writeToFlash(const void* data, size_t length);

    /**
     * @brief Helper to read a block of bytes from flash (updates read pointer externally).
     */
    bool readFromFlash(uint32_t& readAddress, void* buffer, size_t length);
};

#endif // DATA_SAVER_SPI_H