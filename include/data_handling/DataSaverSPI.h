#ifndef DATASAVERSPI_H
#define DATASAVERSPI_H

#include "ArduinoHAL.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include <array>
#include <cstdlib>


#define METADATA_START_ADDRESS 0x000000  // Start writing metadata at the beginning of the flash
#define DATA_START_ADDRESS 0x001000  // Start writing data after 1 sector (4kB) of metadata
#define POST_LAUNCH_FLAG_ADDRESS 0x000000  // Address of the post-launch flag
#define LAUNCH_START_ADDRESS_ADDRESS 0x000001  // Address of the launch start address (32 bits)

#define POST_LAUNCH_FLAG_TRUE 0x00 // Flag to indicate post-launch mode is active
#define POST_LAUNCH_FLAG_FALSE 0x01 // Flag to indicate post-launch mode is not active


#pragma pack(push, 1)  // Pack the struct to avoid padding between the name and datas
typedef struct {
    uint8_t name;
    float data;
} Record_t; 

typedef struct {
    uint8_t name;
    uint32_t timestamp_ms;
} TimestampRecord_t;
#pragma pack(pop)  // Stop packing from here on out

class DataSaverSPI : public IDataSaver {
public:

    static constexpr size_t BUFFER_SIZE = 256; 

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

    int saveTimestamp(uint32_t timestamp_ms, uint8_t name);

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
    void dumpData(Stream &serial, bool ignoreEmptyPages);

    /**
     * @brief Resets all internal state values (buffer, lastDataPoint, nextWriteAddress, lastTimestamp_ms)
     * Does not erase the flash chip
     */
    void clearInternalState();

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

    // Write buffer to improve write performance
    uint8_t buffer[BUFFER_SIZE];
    size_t bufferIndex = 0;
    uint32_t bufferFlushes = 0; // Keep track of how many times the buffer has been flushed

public:
    /**
     * @brief Returns the current buffer index
     * Useful for testing
     */
    size_t getBufferIndex() const {
        return bufferIndex;
    }

    /**
     * @brief Returns the current buffer flush count
     * Useful for testing
     */
    uint32_t getBufferFlushes() const {
        return bufferFlushes;
    }

    bool getIsChipFullDueToPostLaunchProtection() const {
        return isChipFullDueToPostLaunchProtection;
    }

    bool getRebootedInPostLaunchMode() const {
        return rebootedInPostLaunchMode;
    }

private:

    /**
     * @brief Flushes the buffer to flash
     * 
     * Returns 0 on success
     * Returns 1 if the buffer is empty
     * Returns -1 on error
     */
    int flushBuffer();

    /**
     * @brief Adds data to the buffer
     * 
     * @param data   The data to add
     * @param length The length of the data
     * @return int   0 on success; non-zero on error
     */
    int addDataToBuffer(const uint8_t* data, size_t length);


    // Overloaded functions to add data to the buffer from a Record_t or TimestampRecord_t
    // More efficient than callling addDataToBuffer with each part of the record
    int addRecordToBuffer(Record_t * record) {
        return addDataToBuffer(reinterpret_cast<const uint8_t*>(record), 5);
    }

    int addRecordToBuffer(TimestampRecord_t * record) {
        return addDataToBuffer(reinterpret_cast<const uint8_t*>(record), 5);
    }

    // The chip will keep overwriting data forever unless post launch data is being protected.
    // Once it wraps back around to the launchWriteAddress, it will stop writing data.
    bool isChipFullDueToPostLaunchProtection;

    // If the fc boots and is already in post launch mode, then do not write to flash
    // calling clearPostLaunchMode() will allow writing to flash again after a reboot
    bool rebootedInPostLaunchMode = false;
};

#endif // DATA_SAVER_SPI_H