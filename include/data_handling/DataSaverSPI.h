#ifndef DATASAVERSPI_H
#define DATASAVERSPI_H

#include "ArduinoHAL.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include <array>
#include <cstdlib>
#include <limits>


constexpr uint32_t kMetadataStartAddress = 0x000000;  // Start writing metadata at the beginning of flash
constexpr uint32_t kDataStartAddress = 0x001000;  // Start writing data after 1 sector (4kB) of metadata
constexpr uint32_t kPostLaunchFlagAddress = 0x000000;  // Address of the post-launch flag
constexpr uint32_t kLaunchStartAddressAddress = 0x000001;  // Address of the launch start address (32 bits)

constexpr uint8_t kPostLaunchFlagTrue = 0x00; // Flag to indicate post-launch mode is active
constexpr uint8_t kPostLaunchFlagFalse = 0x01; // Flag to indicate post-launch mode is not active

constexpr uint8_t kEmptyPageValue = 0xFF;


#pragma pack(push, 1)  // Pack the struct to avoid padding between the name and datas
typedef struct { // NOLINT(altera-struct-pack-align)
    uint8_t name;
    float data;
} Record_t; 

typedef struct { // NOLINT(altera-struct-pack-align)
    uint8_t name;
    uint32_t timestamp_ms;
} TimestampRecord_t;
#pragma pack(pop)  // Stop packing from here on out

/**
 * @brief SPI flash implementation of IDataSaver with timestamp compression.
 * @note When to use: onboard non-volatile logging where SD cards are
 *       impractical and data may need to survive power loss or long recovery.
 */
class DataSaverSPI : public IDataSaver {
public:

    static constexpr size_t kBufferSize_bytes = 256;

    /**
     * @brief Construct a new DataSaverSPI object
     * 
     * @param timestampInterval_ms   Interval in ms at which timestamps are stored
     * @param flash                  Pointer to an initialized Adafruit_SPIFlash object
     *
     * @note timestampInterval_ms must be < 65535 (about 1 minute)
     */
    DataSaverSPI(uint16_t timestampInterval_ms, Adafruit_SPIFlash *flash);

    /**
     * @brief Saves a DataPoint to SPI flash.
     * 
     * If the new data point’s timestamp differs from the last written
     * timestamp by more than timestampInterval_ms_, the new timestamp is also
     * written to flash. Otherwise, the timestamp is omitted (to save space).
     * 
     * @param dataPoint The data point to save
     * @param name An 8-bit “identifier” for the data point (could be a sensor ID)
     * @return int 0 on success; non-zero on error
     */
    int saveDataPoint(const DataPoint& dataPoint, uint8_t name) override;

    /**
     * @brief Persist a bare timestamp entry to flash.
     * @param timestamp_ms Timestamp in milliseconds to record.
     * @note When to use: emitted internally when gaps exceed
     *       timestampInterval_ms_, or explicitly in tests.
     */
    int saveTimestamp(uint32_t timestamp_ms);

    /**
     * @brief Initialize the flash chip and metadata.
     * @note When to use: call during setup before any saveDataPoint usage.
     */
    virtual bool begin() override;

    /**
     * @brief Returns whether the metadata from the flash chip
     *        indicates that it contains post-launch data that hasn't been 
     *        dumped yet. Call `clearPostLaunchMode()` to clear this flag.
     *        once you are confident that you have the data you want dumped
     *        to a more permanent storage or another machine. 
     * 
     * Will also update the post-launch mode flag.
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
     * Sets the post-launch mode flag to true on the flash chip.
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
     * @brief Stream all recorded data to a serial connection.
     * @param serial            Output stream.
     * @param ignoreEmptyPages  Skip pages that appear unwritten.
     * @note When to use: post-flight data retrieval before erasing or
     *       redeploying the flash chip.
     */
    void dumpData(Stream &serial, bool ignoreEmptyPages);

    /**
     * @brief Reset in-memory pointers without erasing flash contents.
     * @note When to use: restart logging logic while preserving prior data on
     *       the chip.
     */
    void clearInternalState();

    /**
     * @brief Erase the entire flash chip to start fresh.
     * @note When to use: Never needs to be used in normal operation. After
     *       about 1 hour of runtime, the entire chip is overwritten anyways. 
     *       Can be used in testing to reset the chip to a known state and then 
     *       test which bytes are written
     */
    void eraseAllData();

    /**
     * @brief Returns the last timestamp that was actually written to flash.
     */
    uint32_t getLastTimestamp() const {
        return lastTimestamp_ms_;
    }

    /**
     * @brief Returns the last DataPoint that was written (not necessarily
     *        including timestamp, just the data chunk).
     */
    DataPoint getLastDataPoint() const {
        return lastDataPoint_;
    }

    uint32_t getLaunchWriteAddress() const {
        return launchWriteAddress_;
    }

    uint32_t getNextWriteAddress() const {
        return nextWriteAddress_;
    }

    /**
     * @brief Resets the timestamp to 0. Can be used to start a new flight
     *        during runtime. Useful for testing.
     */
    void resetTimestamp() {
        lastTimestamp_ms_ = 0;
    }

    /**
     * @brief Returns whether the flash chip is in post-launch mode
     *       without updating the post-launch mode flag or reading from flash.
     */
    bool quickGetPostLaunchMode() {
        return this->postLaunchMode_;
    }

private:
    // Interval at which to store the timestamp in flash.
    uint16_t timestampInterval_ms_;

    // The last timestamp we actually wrote to flash.
    uint32_t lastTimestamp_ms_;

    // The timestamp this module was given for launch
    uint32_t launchTimestamp_ms_; 

    // The last data point written
    DataPoint lastDataPoint_;

    // Flash driver
    Adafruit_SPIFlash *flash_;

    // Next address in flash at which to write.
    uint32_t nextWriteAddress_;

    // Address at which launch was detected
    uint32_t launchWriteAddress_;

    bool postLaunchMode_;

private:
    /**
     * @brief Helper to write a block of bytes to flash at the current
     *        write address and advance that pointer.
     */
    bool writeToFlash(const uint8_t* data, size_t length);

    /**
     * @brief Helper to read a block of bytes from flash (updates read pointer externally).
     */
    bool readFromFlash(uint32_t& readAddress, uint8_t* buffer, size_t length);

    // Write buffer to improve write performance
    uint8_t buffer_[kBufferSize_bytes] = {};
    size_t bufferIndex_ = 0;
    uint32_t bufferFlushes_ = 0; // Keep track of how many times the buffer has been flushed

public:
    /**
     * @brief Returns the current buffer index
     * Useful for testing
     */
    size_t getBufferIndex() const {
        return bufferIndex_;
    }

    /**
     * @brief Returns the current buffer flush count
     * Useful for testing
     */
    uint32_t getBufferFlushes() const {
        return bufferFlushes_;
    }

    bool getIsChipFullDueToPostLaunchProtection() const {
        return isChipFullDueToPostLaunchProtection_;
    }

    bool getRebootedInPostLaunchMode() const {
        return rebootedInPostLaunchMode_;
    }

private:

    /**
     * @brief Flushes the buffer to flash.
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
    // Once it wraps back around to the launch write address, it will stop writing data.
    bool isChipFullDueToPostLaunchProtection_;

    // If the flight computer boots and is already in post launch mode, do not write to flash.
    // Calling clearPostLaunchMode() will allow writing to flash again after a reboot.
    bool rebootedInPostLaunchMode_ = false;

    // Tracks which sector has already been pre-erased for the next boundary write.
    // UINT32_MAX means "no prepared sector".
    uint32_t preparedSectorNumber_ = std::numeric_limits<uint32_t>::max();
};

#endif // DATA_SAVER_SPI_H
