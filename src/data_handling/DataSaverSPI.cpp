#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"

#include <cstring>

DataSaverSPI::DataSaverSPI(uint16_t timestampInterval_ms,
                           Adafruit_SPIFlash* flash)
    : timestampInterval_ms(timestampInterval_ms),
      flash(flash),
      nextWriteAddress(DATA_START_ADDRESS),
      bufferIndex(0),
      lastTimestamp_ms(0),
      postLaunchMode(false),
      launchWriteAddress(0),
      isChipFullDueToPostLaunchProtection(false) {
  clearInternalState();
}

int DataSaverSPI::saveDataPoint(const DataPoint& dp, uint8_t name) {
  if (rebootedInPostLaunchMode || isChipFullDueToPostLaunchProtection) {
    return 1;  // Do not save if we rebooted in post-launch mode
  }

    // Write a timestamp automatically if enough time has passed since the last one
    uint32_t const timestamp = dp.timestamp_ms;
    if (timestamp - lastTimestamp_ms > timestampInterval_ms) {
      if (saveTimestamp(timestamp) < 0) {
        return -1;
      }
    }

    Record_t record = {name, dp.data};
    if (addRecordToBuffer(&record) < 0) {
      return -1;
    }

    lastDataPoint = dp;
    return 0;
}

int DataSaverSPI::saveTimestamp(uint32_t timestamp_ms){
    if (rebootedInPostLaunchMode || isChipFullDueToPostLaunchProtection) {
      return 1;  // Do not save if we rebooted in post-launch mode
    }

    TimestampRecord_t timeStampRecord = {TIMESTAMP, timestamp_ms};
    if (addRecordToBuffer(&timeStampRecord) != 0) {
      return -1;
    }

    lastTimestamp_ms = timestamp_ms; 
    return 0;
}

int DataSaverSPI::addDataToBuffer(const uint8_t* data, size_t length) {
    if (bufferIndex + length > BUFFER_SIZE) {
        // Flush the buffer
        if (flushBuffer() < 0) {
          return -1;
        }
    }

    // Copy the data into the buffer
    memcpy(buffer + bufferIndex, data, length);
    bufferIndex += length;
    return 0;
}

// Write the entire buffer to flash
int DataSaverSPI::flushBuffer() {
    if (bufferIndex == 0) {
        return 1;  // Nothing to flush
    }

    // Check if we need to wrap around
    if (nextWriteAddress + bufferIndex > flash->size()) {
        // Wrap around
        nextWriteAddress = DATA_START_ADDRESS;
    }   

    // Check that we haven't wrapped around to the launch address while in post-launch mode
    if (postLaunchMode && nextWriteAddress <= launchWriteAddress && nextWriteAddress + BUFFER_SIZE * 2 > launchWriteAddress) {
        isChipFullDueToPostLaunchProtection = true;
        return -1; // Indicate no write due to post-launch protection
    }

    if (nextWriteAddress % SFLASH_SECTOR_SIZE == 0) {
        if (!flash->eraseSector(nextWriteAddress / SFLASH_SECTOR_SIZE)) {
            return -1;
        }
    }

    // Write 1 page of data
    if (!flash->writeBuffer(nextWriteAddress, buffer, BUFFER_SIZE)) {
        return -1;
    }

    nextWriteAddress += BUFFER_SIZE;  // keep it aligned to the buffer size or page size
    bufferIndex = 0; // Reset the buffer
    bufferFlushes++;
    return 0;
}


bool DataSaverSPI::begin() {
    if (flash == nullptr) {
        return false;
    }
    if (!flash->begin()) {
        return false;
    }

    this->postLaunchMode = isPostLaunchMode();
    if (postLaunchMode) {
        // If we are already in post-launch mode, then don't write to flash at all
        rebootedInPostLaunchMode = true;
        return false; 
    }

    return true;
}

bool DataSaverSPI::isPostLaunchMode() {
    uint8_t flag = 0;
    flash->readBuffer(POST_LAUNCH_FLAG_ADDRESS, &flag, sizeof(flag));
    this->postLaunchMode = (flag == POST_LAUNCH_FLAG_TRUE);
    return this->postLaunchMode;
}

void DataSaverSPI::clearPostLaunchMode() {
    flash->eraseSector(METADATA_START_ADDRESS / SFLASH_SECTOR_SIZE);
    
    uint8_t flag = POST_LAUNCH_FLAG_FALSE;
    flash->writeBuffer(POST_LAUNCH_FLAG_ADDRESS, &flag, sizeof(flag));
    clearInternalState();
}

void DataSaverSPI::dumpData(Stream &serial, bool ignoreEmptyPages) { //NOLINT(readability-function-cognitive-complexity)
    uint32_t readAddress = DATA_START_ADDRESS; //NOLINT(cppcoreguidelines-init-variables) //NOLINT(misc-const-correctness)
    // For each page write 51 sets of 5 bytes to serial with a newline
    std::array<uint8_t, SFLASH_PAGE_SIZE> buffer; //NOLINT(cppcoreguidelines-init-variables)
    size_t recordSize = sizeof(Record_t); //NOLINT(cppcoreguidelines-init-variables)
    size_t numRecordsPerPage = SFLASH_PAGE_SIZE / recordSize; //NOLINT(cppcoreguidelines-init-variables)

    // If not in post-launch mode, erase the next sector after nextWriteAddress
    // This ensures that we don't accidentally dump old data from previous flights
    // If ignoreEmptyPages is true, then we don't need to erase the next sector
    if (!postLaunchMode && !ignoreEmptyPages) {
        flash->eraseSector(nextWriteAddress / SFLASH_SECTOR_SIZE + 1);
    }

    // To ensure it's lined-up let's set a '\n' , '\r' and a 's' at the start
    serial.write('a');
    serial.write('b');
    serial.write('c');
    serial.write('d');
    serial.write('e');
    serial.write('f');

    bool done = false;
    bool timedOut = false;
    bool stoppedFromEmptyPage = false;
    bool badRead = false;
   
    while (readAddress < flash->size()) { 
        if (!readFromFlash(readAddress, buffer.data(), SFLASH_PAGE_SIZE)) {
            badRead = true;
            return;
        }

        // If the first name of this page is 255 then break
        if (buffer[0] == EMPTY_PAGE) {
            if (ignoreEmptyPages) {
                continue;
            }
            done = true;
            stoppedFromEmptyPage = true;
            break;
        }

        // At the start of each page, write some alignment characters
        std::array<uint8_t, 3> startLine = {'l', 's', 'h'};
        serial.write(startLine.data(), 3);



        for (size_t i = 0; i < numRecordsPerPage; i++) { //NOLINT(cppcoreguidelines-init-variables)

            serial.write(buffer.data() + i * recordSize, recordSize);
            // serial.write('\n');
        }

        // Wait for a 'n' character to be received before continuing (10 second timeout)
        uint32_t const timeout = millis() + 10000;
        while (serial.read() != 'n') {
            if (millis() > timeout) {
                timedOut = true;
                return;
            }
        }

    }

    #pragma unroll
    for (int i = 0; i < BUFFER_SIZE; i++){
        std::array<uint8_t, 3> doneLine = {'E', 'O', 'F'};
        serial.write(doneLine.data(), doneLine.size());
        if (done){
            serial.write('D');
        }
        if (timedOut){
            serial.write('T');
        }
        if (stoppedFromEmptyPage){
            serial.write('P');
        }
        if (badRead){
            serial.write('B');
        }
        if (readAddress >= flash->size()){
            serial.write('F');
        }
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
    isChipFullDueToPostLaunchProtection = false;
}

void DataSaverSPI::eraseAllData() {
    flash->eraseChip();
    clearPostLaunchMode();

    clearInternalState();

    // Clear the launchWriteAddress
    launchWriteAddress = 0;

}

void DataSaverSPI::launchDetected(uint32_t launchTimestamp_ms) {
    this->launchTimestamp_ms = launchTimestamp_ms;

    // 0) Stop if we are already in post-launch mode
    if (postLaunchMode) {
        return;
    }

    // 0.5) Clear the metadata sector to avoid 0 --> 1 inabilites
    flash->eraseSector(METADATA_START_ADDRESS / SFLASH_SECTOR_SIZE);

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
    size_t recordSize = sizeof(uint32_t) + sizeof(uint8_t) + sizeof(DataPoint); //NOLINT(cppcoreguidelines-init-variables)
    uint32_t const oneMinuteInMs = 60000;
    uint32_t const dataPointsPerMinute = oneMinuteInMs / timestampInterval_ms;
    uint32_t rollbackBytes       = dataPointsPerMinute * recordSize;

    // 3) Clamp rollbackBytes to something reasonable. We must not exceed
    //    the usable flash region (from address=1 to address=flash->size()-1).
    uint32_t const maxUsable = flash->size() - DATA_START_ADDRESS;
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

    uint32_t const sizeOfFlash = flash->size();

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

    std::array<uint8_t, sizeof(launchWriteAddress)> bytes;
    std::memcpy(bytes.data(), &launchWriteAddress, sizeof(launchWriteAddress));
    flash->writeBuffer(LAUNCH_START_ADDRESS_ADDRESS, bytes.data(), bytes.size());

}

bool DataSaverSPI::writeToFlash(const uint8_t* data, size_t length) {
    if (!flash->writeBuffer(nextWriteAddress, data, length)) {
        return false;
    }
    nextWriteAddress += length;
    return true;
}

bool DataSaverSPI::readFromFlash(uint32_t& readAddress, uint8_t* buffer, size_t length) {
    if (!flash->readBuffer(readAddress, buffer, length)) {
        return false;
    }
    readAddress += length;
    return true;
}
