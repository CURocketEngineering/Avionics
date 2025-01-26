#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"

#include <cstring>



DataSaverSPI::DataSaverSPI(uint16_t timestampInterval_ms, Adafruit_SPIFlash *flash)
    : timestampInterval_ms(timestampInterval_ms),
      flash(flash), nextWriteAddress(DATA_START_ADDRESS), bufferIndex(0),
      lastTimestamp_ms(0),
      postLaunchMode(false),
      launchWriteAddress(0)
    {}

int DataSaverSPI::saveDataPoint(DataPoint dp, uint8_t name) {
    if (!flash || !flash->begin()) return -1;

    // Stop saving only if we wrapped back and hit the sacred address
    if (postLaunchMode && nextWriteAddress <= launchWriteAddress && nextWriteAddress + sizeof(DataPoint) > launchWriteAddress) {
        return 1; // Indicate no write due to post-launch protection
    }

    // Write timestamp if enough time has passed since the last one
    uint32_t timestamp = dp.timestamp_ms;
    if (timestamp - lastTimestamp_ms > timestampInterval_ms) {
        TimestampRecord_t tr = {TIMESTAMP, timestamp};
        if (!addRecordToBuffer(&tr) == 0) return -1;

        lastTimestamp_ms = timestamp;  // Everything after this timestamp until the next timestamp will use this timestamp when reconstructed
    }

    Record_t record = {name, dp.data};
    if (addRecordToBuffer(&record) < 0) return -1;

    lastDataPoint = dp;
    return 0;
}

int DataSaverSPI::addDataToBuffer(const uint8_t* data, size_t length) {
    if (bufferIndex + length > BUFFER_SIZE) {
        // Flush the buffer
        if (flushBuffer() < 0) return -1;
    }

    // Copy the data into the buffer
    memcpy(buffer + bufferIndex, data, length);
    bufferIndex += length;
    return 0;
}

// Write the entire buffer to flash
int DataSaverSPI::flushBuffer() {
    if (bufferIndex == 0) return 1; // Nothing to flush

    // Check if we need to wrap around
    if (nextWriteAddress + bufferIndex > flash->size()) {
        // Wrap around
        nextWriteAddress = DATA_START_ADDRESS;
    }   

    if (!flash->writeBuffer(nextWriteAddress, buffer, BUFFER_SIZE)) {
        return -1;
    }

    nextWriteAddress += BUFFER_SIZE;  // keep it aligned to the buffer size or page size
    bufferIndex = 0; // Reset the buffer
    // Fill the buffer with 0s
    memset(buffer, 0, BUFFER_SIZE);
    bufferFlushes++;
    return 0;
}


bool DataSaverSPI::begin() {
    if (!flash) return false;
    if (!flash->begin()) return false;

    this->postLaunchMode = isPostLaunchMode();
    return true;
}

bool DataSaverSPI::isPostLaunchMode() {
    uint8_t flag;
    flash->readBuffer(POST_LAUNCH_FLAG_ADDRESS, &flag, sizeof(flag));
    this->postLaunchMode = (flag == POST_LAUNCH_FLAG_TRUE);
    return this->postLaunchMode;
}

void DataSaverSPI::clearPostLaunchMode() {
    flash->eraseSector(METADATA_START_ADDRESS / SFLASH_SECTOR_SIZE);
    
    uint8_t flag = POST_LAUNCH_FLAG_FALSE;
    flash->writeBuffer(POST_LAUNCH_FLAG_ADDRESS, &flag, sizeof(flag));
    postLaunchMode = false;
}

void DataSaverSPI::dumpData(Stream &serial) {
    uint32_t readAddress = 1; // Start reading after metadata
    // Write each byte to serial
    while (readAddress < flash->size()) {
        uint8_t byte;
        if (!readFromFlash(readAddress, &byte, sizeof(byte))) {
            serial.println("Error reading from flash");
            return;
        }
        serial.write(byte);
    }
}

void DataSaverSPI::clearInternalState() {
    bufferIndex = 0;
    memset(buffer, 0, BUFFER_SIZE);
    lastDataPoint = {0, 0};
    nextWriteAddress = DATA_START_ADDRESS;
    lastTimestamp_ms = 0;
    postLaunchMode = false;
    launchWriteAddress = 0;
    bufferFlushes = 0;
}

void DataSaverSPI::eraseAllData() {
    flash->eraseChip();
    clearPostLaunchMode();

    clearInternalState();

    // Clear the launchWriteAddress
    launchWriteAddress = 0;

}

void DataSaverSPI::launchDetected(uint32_t launchTimestamp_ms) {
    // 1) Set the post-launch flag in metadata so we don't overwrite post-launch data.
    uint8_t flag = POST_LAUNCH_FLAG_TRUE;
    flash->writeBuffer(POST_LAUNCH_FLAG_ADDRESS, &flag, sizeof(flag));
    postLaunchMode = true;

    // 2) Compute how many bytes we want to roll back to capture ~1 minute of pre-launch data.
    //    If you always store timestamps + name + DataPoint, factor that in:
    //
    //    struct DataRecord {
    //        uint8_t  name;
    //        DataPoint data;
    //        // occasionally 4 more bytes for a timestamp if we cross the interval threshold
    //    };
    //
    //    For simplicity, let's assume worst-case all data points have the 4-byte timestamp:
    //      size_t recordSize = sizeof(uint32_t) // timestamp
    //                        + sizeof(uint8_t)  // name
    //                        + sizeof(DataPoint); 
    //
    //    If you only occasionally store the timestamp, you might want a more nuanced approach.
    // 
    size_t recordSize = sizeof(uint32_t) + sizeof(uint8_t) + sizeof(DataPoint);
    uint32_t oneMinuteInMs = 60000;
    uint32_t dataPointsPerMinute = oneMinuteInMs / timestampInterval_ms; 
    uint32_t rollbackBytes       = dataPointsPerMinute * recordSize;

    // 3) Clamp rollbackBytes to something reasonable. We must not exceed
    //    the usable flash region (from address=1 to address=flash->size()-1).
    uint32_t maxUsable = flash->size() - DATA_START_ADDRESS;
    if (rollbackBytes > maxUsable) {
        // If we can't keep an entire minute, just keep as much as we can
        rollbackBytes = maxUsable;
    }

    // 4) Next, we do ring-buffer math to find our new launchWriteAddress
    //    which is "1 minute's worth of data behind nextWriteAddress" *in a circular sense*.

    //    Because nextWriteAddress can be anywhere in [1, flash->size()-1],
    //    let’s do a safe modular subtraction:
    //
    //       newAddr = ( nextWriteAddress + flash->size() - rollbackBytes ) 
    //                                % flash->size()
    //
    //    Then we ensure it’s never 0 because 0 is used for metadata.

    uint32_t sizeOfFlash = flash->size();
    
    // Make sure we aren't in the metadata region
    if (nextWriteAddress < DATA_START_ADDRESS) {
        nextWriteAddress = DATA_START_ADDRESS;
    }

    // Use 64-bit to avoid any negative wrap during the subtraction.
    int64_t potentialAddr = static_cast<int64_t>(nextWriteAddress)
                          + static_cast<int64_t>(sizeOfFlash)  // ensure positivity
                          - static_cast<int64_t>(rollbackBytes);

    // Modulo by sizeOfFlash to bring it back into [0, sizeOfFlash-1].
    potentialAddr = potentialAddr % sizeOfFlash;

    // If result is 0 or negative after mod, or less than DATA_START_ADDRESS, add sizeOfFlash
    if (potentialAddr <= 0 || potentialAddr < DATA_START_ADDRESS) {
        potentialAddr += sizeOfFlash;
    }


    launchWriteAddress = static_cast<uint32_t>(potentialAddr);

    // Write the launchWriteAddress to the metadata
    flash->writeBuffer(LAUNCH_START_ADDRESS_ADDRESS, reinterpret_cast<uint8_t*>(&launchWriteAddress),
                                                     sizeof(launchWriteAddress));
}

bool DataSaverSPI::writeToFlash(const uint8_t* data, size_t length) {
    if (!flash->writeBuffer(nextWriteAddress, data, length)) return false;
    nextWriteAddress += length;
    return true;
}

bool DataSaverSPI::readFromFlash(uint32_t& readAddress, uint8_t* buffer, size_t length) {
    if (!flash->readBuffer(readAddress, buffer, length)) return false;
    readAddress += length;
    return true;
}
