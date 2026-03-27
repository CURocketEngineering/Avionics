#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"

#include <cstring>
#include <limits>

DataSaverSPI::DataSaverSPI(uint16_t timestampInterval_ms,
                           Adafruit_SPIFlash* flash)
    : timestampInterval_ms_(timestampInterval_ms),
      lastTimestamp_ms_(0),
      launchTimestamp_ms_(0),
      flash_(flash),
      nextWriteAddress_(kDataStartAddress),
      launchWriteAddress_(0),
      postLaunchMode_(false),
      bufferIndex_(0),
      isChipFullDueToPostLaunchProtection_(false) {
  clearInternalState();
}

int DataSaverSPI::saveDataPoint(const DataPoint& dataPoint, uint8_t name) {
  if (rebootedInPostLaunchMode_ || isChipFullDueToPostLaunchProtection_) {
    return 1;  // Do not save if we rebooted in post-launch mode
  }

    // Write a timestamp automatically if enough time has passed since the last one
    uint32_t const timestamp = dataPoint.timestamp_ms;
    if (timestamp - lastTimestamp_ms_ > timestampInterval_ms_) {
      if (saveTimestamp(timestamp) < 0) {
        return -1;
      }
    }

    Record_t record = {name, dataPoint.data};
    if (addRecordToBuffer(&record) < 0) {
      return -1;
    }

    lastDataPoint_ = dataPoint;
    return 0;
}

int DataSaverSPI::saveTimestamp(uint32_t timestamp_ms){
    if (rebootedInPostLaunchMode_ || isChipFullDueToPostLaunchProtection_) {
      return 1;  // Do not save if we rebooted in post-launch mode
    }

    TimestampRecord_t timeStampRecord = {TIMESTAMP, timestamp_ms};
    if (addRecordToBuffer(&timeStampRecord) != 0) {
      return -1;
    }

    lastTimestamp_ms_ = timestamp_ms; 
    return 0;
}

int DataSaverSPI::addDataToBuffer(const uint8_t* data, size_t length) {
    if (bufferIndex_ + length > kBufferSize_bytes) {
        // Flush the buffer
        if (flushBuffer() < 0) {
          return -1;
        }
    }

    // Copy the data into the buffer
    memcpy(buffer_ + bufferIndex_, data, length);
    bufferIndex_ += length;
    return 0;
}

// Write the entire buffer to flash.
int DataSaverSPI::flushBuffer() {
    if (bufferIndex_ == 0) {
        return 1;  // Nothing to flush
    }

    // Check if we need to wrap around
    if (nextWriteAddress_ + bufferIndex_ > flash_->size()) {
        // Wrap around
        nextWriteAddress_ = kDataStartAddress;
    }   

    // Check that we haven't wrapped around to the launch address while in post-launch mode
    if (postLaunchMode_ && nextWriteAddress_ <= launchWriteAddress_ && nextWriteAddress_ + kBufferSize_bytes * 2 > launchWriteAddress_) {
        isChipFullDueToPostLaunchProtection_ = true;
        return -1; // Indicate no write due to post-launch protection
    }

    if (nextWriteAddress_ % SFLASH_SECTOR_SIZE == 0) {
        if (!flash_->eraseSector(nextWriteAddress_ / SFLASH_SECTOR_SIZE)) {
            return -1;
        }
    }

    // Write 1 page of data
    if (!flash_->writeBuffer(nextWriteAddress_, buffer_, kBufferSize_bytes)) {
        return -1;
    }

    nextWriteAddress_ += kBufferSize_bytes;  // keep it aligned to the buffer size or page size
    bufferIndex_ = 0; // Reset the buffer
    bufferFlushes_++;
    return 0;
}


bool DataSaverSPI::begin() {
    if (flash_ == nullptr) {
        return false;
    }
    if (!flash_->begin()) {
        return false;
    }

    this->postLaunchMode_ = isPostLaunchMode();
    if (postLaunchMode_) {
        // If we are already in post-launch mode, then don't write to flash at all.
        rebootedInPostLaunchMode_ = true;
        return false; 
    }

    return true;
}

bool DataSaverSPI::isPostLaunchMode() {
    uint8_t flag = 0;
    flash_->readBuffer(kPostLaunchFlagAddress, &flag, sizeof(flag));
    this->postLaunchMode_ = (flag == kPostLaunchFlagTrue);
    return this->postLaunchMode_;
}

void DataSaverSPI::clearPostLaunchMode() {
    flash_->eraseSector(kMetadataStartAddress / SFLASH_SECTOR_SIZE);
    
    uint8_t flag = kPostLaunchFlagFalse;
    flash_->writeBuffer(kPostLaunchFlagAddress, &flag, sizeof(flag));

    postLaunchMode_ = false;
}

void DataSaverSPI::dumpData(Stream &serial, bool ignoreEmptyPages) { //NOLINT(readability-function-cognitive-complexity)
    uint32_t readAddress = kDataStartAddress; //NOLINT(cppcoreguidelines-init-variables) //NOLINT(misc-const-correctness)
    // For each page write 51 sets of 5 bytes to serial with a newline
    std::array<uint8_t, SFLASH_PAGE_SIZE> buffer; //NOLINT(cppcoreguidelines-init-variables)
    size_t recordSize = sizeof(Record_t); //NOLINT(cppcoreguidelines-init-variables)
    size_t numRecordsPerPage = SFLASH_PAGE_SIZE / recordSize; //NOLINT(cppcoreguidelines-init-variables)

    // If not in post-launch mode, erase the next sector after the next write address.
    // This ensures that we don't accidentally dump old data from previous flights
    // If ignoreEmptyPages is true, then we don't need to erase the next sector
    if (!postLaunchMode_ && !ignoreEmptyPages) {
        flash_->eraseSector(nextWriteAddress_ / SFLASH_SECTOR_SIZE + 1);
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
   
    while (readAddress < flash_->size()) { 
        if (!readFromFlash(readAddress, buffer.data(), SFLASH_PAGE_SIZE)) {
            badRead = true;
            return;
        }

        // If the first name of this page is 255 then break
        if (buffer[0] == kEmptyPageValue) {
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
        const uint32_t timeout = static_cast<uint32_t>(millis()) + 10000U;
        while (serial.read() != 'n') {
            if (static_cast<uint32_t>(millis()) > timeout) {
                timedOut = true;
                return;
            }
        }

    }

    for (size_t i = 0; i < kBufferSize_bytes; i++){
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
        if (readAddress >= flash_->size()){
            serial.write('F');
        }
    }
}

void DataSaverSPI::clearInternalState() {
    bufferIndex_ = 0;
    memset(buffer_, 0, kBufferSize_bytes);
    lastDataPoint_ = {0, 0};
    nextWriteAddress_ = kDataStartAddress;
    lastTimestamp_ms_ = 0;
    postLaunchMode_ = false;
    launchWriteAddress_ = 0;
    bufferFlushes_ = 0;
    isChipFullDueToPostLaunchProtection_ = false;
}

void DataSaverSPI::eraseAllData() {
    flash_->eraseChip();
    clearPostLaunchMode();

    clearInternalState();

    // Clear the launch write address.
    launchWriteAddress_ = 0;

}

void DataSaverSPI::launchDetected(uint32_t launchTimestamp_ms) {
    this->launchTimestamp_ms_ = launchTimestamp_ms;

    // 0) Stop if we are already in post-launch mode
    if (postLaunchMode_) {
        return;
    }

    // 0.5) Clear the metadata sector to avoid 0 --> 1 inabilities
    flash_->eraseSector(kMetadataStartAddress / SFLASH_SECTOR_SIZE);

    // 1) Set the post-launch flag in metadata so we don't overwrite post-launch data.
    uint8_t flag = kPostLaunchFlagTrue;
    flash_->writeBuffer(kPostLaunchFlagAddress, &flag, sizeof(flag));
    postLaunchMode_ = true;

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
    const size_t recordSize_bytes = sizeof(uint32_t) + sizeof(uint8_t) + sizeof(DataPoint);
    uint32_t const oneMinuteInMs = 60000;
    uint32_t const dataPointsPerMinute = oneMinuteInMs / timestampInterval_ms_;
    uint64_t rollbackSize_bytes = static_cast<uint64_t>(dataPointsPerMinute) * static_cast<uint64_t>(recordSize_bytes);

    // 3) Clamp rollbackSize_bytes to something reasonable. We must not exceed
    //    the usable flash region (from address=1 to address=flash size - 1).
    const size_t flashSize_bytes = flash_->size();
    const size_t maxUsable_bytes = (flashSize_bytes > static_cast<size_t>(kDataStartAddress))
        ? (flashSize_bytes - static_cast<size_t>(kDataStartAddress))
        : 0U;
    const uint64_t maxUsable = static_cast<uint64_t>(maxUsable_bytes);
    if (rollbackSize_bytes > maxUsable) {
        // If we can't keep an entire minute, just keep as much as we can
        rollbackSize_bytes = maxUsable;
    }

    // 4) Next, we do ring-buffer math to find the new launch write address,
    //    which is "1 minute's worth of data behind the next write address" in a circular sense.

    //    Because the next write address can be anywhere in [1, flash size - 1],
    //    let’s do a safe modular subtraction:
    //
    //       newAddr = (next write address + flash size - rollback size)
    //                 % flash size
    //
    //    Then we ensure it’s never 0 because 0 is used for metadata.

    const uint32_t sizeOfFlash = static_cast<uint32_t>(flashSize_bytes);

    // Make sure we aren't in the metadata region
    if (nextWriteAddress_ < kDataStartAddress) {
        nextWriteAddress_ = kDataStartAddress;
    }

    // Use 64-bit to avoid any negative wrap during the subtraction.
    int64_t potentialAddr = static_cast<int64_t>(nextWriteAddress_)
                          + static_cast<int64_t>(sizeOfFlash)  // ensure positivity
                          - static_cast<int64_t>(rollbackSize_bytes);

    // Modulo by sizeOfFlash to bring it back into [0, sizeOfFlash-1].
    potentialAddr = potentialAddr % sizeOfFlash;

    // If result lands in the metadata region, wrap back into the data region.
    if (potentialAddr < static_cast<int64_t>(kDataStartAddress)) {
        potentialAddr += sizeOfFlash;
    }


    launchWriteAddress_ = static_cast<uint32_t>(potentialAddr);

    std::array<uint8_t, sizeof(launchWriteAddress_)> bytes;
    std::memcpy(bytes.data(), &launchWriteAddress_, sizeof(launchWriteAddress_));
    flash_->writeBuffer(kLaunchStartAddressAddress, bytes.data(), bytes.size());

}

bool DataSaverSPI::writeToFlash(const uint8_t* data, size_t length) {
    if (!flash_->writeBuffer(nextWriteAddress_, data, length)) {
        return false;
    }
    nextWriteAddress_ = static_cast<uint32_t>(nextWriteAddress_ + static_cast<uint32_t>(length));
    return true;
}

bool DataSaverSPI::readFromFlash(uint32_t& readAddress, uint8_t* buffer, size_t length) {
    if (!flash_->readBuffer(readAddress, buffer, length)) {
        return false;
    }
    readAddress = static_cast<uint32_t>(readAddress + static_cast<uint32_t>(length));
    return true;
}
