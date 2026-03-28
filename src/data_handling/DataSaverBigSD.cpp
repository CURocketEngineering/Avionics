#include "data_handling/DataSaverBigSD.h"
#include "ArduinoHAL.h"   // for Serial

// one SdFat object for all
/* static */ SdFat DataSaverBigSD::sd_; //NOLINT(readability-identifier-length)      
    
/* ------------------------------------------------------------------------- */
DataSaverBigSD::DataSaverBigSD(uint8_t csPin) : csPin_(csPin) {}

/* ---------------------------  begin()  ----------------------------------- */
bool DataSaverBigSD::begin() {
    Serial.print(F("Init SD… "));
    pinMode(csPin_, OUTPUT);
    if (!sd_.begin(csPin_, SD_SCK_MHZ(40))) {     // 40 MHz on ESP32‑S3
        Serial.println(F("fail"));
        return false;
    }

    filePath_ = nextFreeFilePath();
    if (!file_.open(filePath_.c_str(), O_WRITE | O_CREAT)) {
        Serial.println(F("file create fail"));
        return false;
    }

    // Pre‑allocate 4 MiB so writes stay contiguous (faster & less wear)
    file_.preAllocate(kPreAllocateSize_MiB * kBytesPerMiB_bytes);

    Serial.print(F("Logging to ")); Serial.println(filePath_.c_str());

    bufLen_ = 0;
    linesPending_ = 0;
    lastFlushMs_ = static_cast<uint32_t>(millis());
    lastSyncMs_ = lastFlushMs_;
    ready_ = true;
    return true;
}

/* -----------------------  saveDataPoint()  ------------------------------- */
int DataSaverBigSD::saveDataPoint(const DataPoint& dataPoint, uint8_t name) {
    if (!ready_) {
        return DS_NOT_READY;
    }

    // Reserve space left
    size_t remaining = sizeof(buf_) - bufLen_; // NOLINT(cppcoreguidelines-init-variables)

    // Format the new line
    int numCharsWritten = snprintf(buf_ + bufLen_, remaining, "%lu,%u,%.6f\n", // NOLINT(cppcoreguidelines-init-variables)
                     static_cast<long unsigned int>(dataPoint.timestamp_ms), name, static_cast<double>(dataPoint.data));

    // Check snprintf result
    if (numCharsWritten <= 0 || (size_t)numCharsWritten >= remaining) {
        // Flush current buffer and try again if this line couldn’t fit
        if (file_.write(buf_, bufLen_) != bufLen_) {
          return DS_BUFFER_WRITE_FAILED;  // failed to write current buffer
        }
        file_.sync();  // just to be extra safe during debug
        bufLen_ = 0;
        linesPending_ = 0;

        // Try again (safe now)
        remaining = sizeof(buf_);
        numCharsWritten = snprintf(buf_, remaining, "%lu,%u,%.6f\n", static_cast<long unsigned int>(dataPoint.timestamp_ms), name, static_cast<double>(dataPoint.data));
        if (numCharsWritten <= 0 || (size_t)numCharsWritten >= remaining) {
          return DS_LINE_TOO_LONG;  // can't encode this line even in an empty buffer
        }
    }

    const auto numCharsWritten_u16 = static_cast<uint16_t>(numCharsWritten);
    bufLen_ = static_cast<uint16_t>(bufLen_ + numCharsWritten_u16);
    ++linesPending_;

    const auto now = static_cast<uint32_t>(millis());
    bool const bufFull = (bufLen_ >= kBufSize_bytes);
    bool const manyLines = (linesPending_ >= kFlushLines);
    bool const timeUp = (now - lastFlushMs_ >= kFlushMs);

    if (bufFull || manyLines || timeUp) {
        if (file_.write(buf_, bufLen_) != bufLen_) {
          return DS_FLUSH_FAILED;
        }
        bufLen_ = 0;
        linesPending_ = 0;
        lastFlushMs_ = now;

        if (now - lastSyncMs_ >= kSyncInterval_ms) {
            file_.sync();
            lastSyncMs_ = now;
        }
    }

    return 0;
}


/* -----------------------------  end()  ----------------------------------- */
void DataSaverBigSD::end() {
    if (!ready_) {
        return;
    }
    if (bufLen_ > 0) {
        file_.write(buf_, bufLen_);
        bufLen_ = 0;
    }
    file_.sync();
    file_.close();
    ready_ = false;
}

/* -------------------  nextFreeFilePath()  -------------------------------- */
std::string DataSaverBigSD::nextFreeFilePath() {
    std::array<char, kFilePathBufferSize> path;
    for (uint16_t flightNumber = 0;; ++flightNumber) {
        snprintf(path.data(), sizeof(path), "/stream-%u.csv", flightNumber);
        if (!sd_.exists(path.data())) {
            return std::string(path.data());
        }
    }
}
