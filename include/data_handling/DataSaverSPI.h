#ifndef DATASAVERSPI_H
#define DATASAVERSPI_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include "FlashDriver.h"
#include <array>
#include <cstdlib>
#include <Adafruit_SPIFlash.h>

class DataSaverSPI : public IDataSaver {
public:
    /**
     * @brief Construct a new DataSaverSPI object
     * 
     * @param timestampInterval_ms  Interval in ms at which timestamps are stored
     *
     * @note timestampInterval_ms must be < 65535 (about 1 minute)
     */
    DataSaverSPI(uint16_t timestampInterval_ms, Adafruit_SPIFlash *flash);

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
     * @brief Returns whether the metadata from the flash chip
     *        indicates that it contains post-launch data that hasn't been 
     *        dumped yet. Call `clearPostLaunchMode()` to clear this flag.
     *        once you are confident that you have the data you want dumped
     *        to a more permanent storage or another machine. 
     * 
     * Will also update the postLaunchMode flag.
     * 
     * @return true if post-launch mode is active
     * @return false if post-launch mode is not active
     */
    bool isPostLaunchMode();

    /**
     * @brief Clears the post-launch mode flag on the flash chip
     *        **WARNING: This will allow the data to be overwritten
     *        Only call this after you have dumped the data to a more
     *        permanent storage or another machine.
     */
    void clearPostLaunchMode();

    /**
     * @brief Call this when launch is detected to set the post-launch flag and 
     *        prevent any data from being overwritten until the flag is cleared.
     *        All data written from this point on is "post-launch" data which is
     *        sacred and should not be overwritten until it has been dumped.
     * 
     * Sets postLaunchMode flag to true on the flash chip.
     * Records the address at which the launch was detected and ensures that it's
     * not overwritten
     * 
     * Data is saved in a circular fashion but once the address where the launch detected
     * is reached this will stop saving data entirely. Also keeps 1 minute of data from before
     * the launch was detected (b/c launch can be detected late and we have extra room)
     * 
     * The rocket may not be recovered for several hours, this prevents the cool launch data
     * from being overwitten with boring laying-on-the-ground data.
     */
    void launchDetected(uint32_t launchTimestamp_ms);

    /**
     * @brief Dumps all data from flash to Serial
     * 
     */
    void dumpData(Stream &serial);

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

    uint32_t getLaunchWriteAddress() const {
        return launchWriteAddress;
    }

    uint32_t getNextWriteAddress() const {
        return nextWriteAddress;
    }

    /**
     * @brief Resets the timestamp to 0. Can be used to start a new flight
     *        during runtime. Useful for testing.
     */
    void resetTimestamp() {
        lastTimestamp_ms = 0;
    }

    /**
     * @brief Returns whether the flash chip is in post-launch mode
     *       without updating the postLaunchMode flag or reading from flash.
     */
    bool quickGetPostLaunchMode() {
        return this->postLaunchMode;
    }

private:
    // Interval at which to store the timestamp in flash
    uint16_t timestampInterval_ms;

    // The last timestamp we actually wrote to flash
    uint32_t lastTimestamp_ms;

    // The last data point written
    DataPoint lastDataPoint;

    // Flash driver
    Adafruit_SPIFlash *flash;

    // Next address in flash at which to write
    uint32_t nextWriteAddress;

    // Address at which launch was detected
    uint32_t launchWriteAddress;

    bool postLaunchMode;

private:
    /**
     * @brief Helper to write a block of bytes to flash at the current
     *        nextWriteAddress and advance that pointer.
     */
    bool writeToFlash(const uint8_t* data, size_t length);

    /**
     * @brief Helper to read a block of bytes from flash (updates read pointer externally).
     */
    bool readFromFlash(uint32_t& readAddress, uint8_t* buffer, size_t length);
};

#endif // DATA_SAVER_SPI_H