#include "data_handling/DataSaverBigSD.h"
#include "ArduinoHAL.h"   // for Serial

// one SdFat object for all
/* static */ SdFat DataSaverBigSD::sd; //NOLINT(readability-identifier-length)      
    
/* ------------------------------------------------------------------------- */
DataSaverBigSD::DataSaverBigSD(uint8_t csPin) : _csPin(csPin) {}

/* ---------------------------  begin()  ----------------------------------- */
bool DataSaverBigSD::begin() {
    Serial.print(F("Init SD… "));
    pinMode(_csPin, OUTPUT);
    if (!sd.begin(_csPin, SD_SCK_MHZ(40))) {     // 40 MHz on ESP32‑S3
        Serial.println(F("fail"));
        return false;
    }

    _filePath = nextFreeFilePath();
    if (!_file.open(_filePath.c_str(), O_WRITE | O_CREAT)) {
        Serial.println(F("file create fail"));
        return false;
    }

    // Pre‑allocate 4 MB so writes stay contiguous (faster & less wear)
    _file.preAllocate(PRE_ALLOCATE_SIZE_MB * BYTES_PER_MB);

    Serial.print(F("Logging to ")); Serial.println(_filePath.c_str());

    _bufLen       = 0;
    _linesPending = 0;
    _lastFlushMs  = millis();
    _lastSyncMs   = _lastFlushMs;
    _ready        = true;
    return true;
}

/* -----------------------  saveDataPoint()  ------------------------------- */
int DataSaverBigSD::saveDataPoint(const DataPoint& dataPoint, uint8_t name) {
    if (!_ready) {
        return DS_NOT_READY;
    }

    // Reserve space left
    size_t remaining = sizeof(_buf) - _bufLen; // NOLINT(cppcoreguidelines-init-variables)

    // Format the new line
    int numCharsWritten = snprintf(_buf + _bufLen, remaining, "%lu,%u,%.6f\n", // NOLINT(cppcoreguidelines-init-variables)
                     static_cast<long unsigned int>(dataPoint.timestamp_ms), name, dataPoint.data);

    // Check snprintf result
    if (numCharsWritten <= 0 || (size_t)numCharsWritten >= remaining) {
        // Flush current buffer and try again if this line couldn’t fit
        if (_file.write(_buf, _bufLen) != _bufLen) {
          return DS_BUFFER_WRITE_FAILED;  // failed to write current buffer
        }
        _file.sync();  // just to be extra safe during debug
        _bufLen = 0;
        _linesPending = 0;

        // Try again (safe now)
        remaining = sizeof(_buf);
        numCharsWritten = snprintf(_buf, remaining, "%lu,%u,%.6f\n", static_cast<long unsigned int>(dataPoint.timestamp_ms), name, dataPoint.data);
        if (numCharsWritten <= 0 || (size_t)numCharsWritten >= remaining) {
          return DS_LINE_TOO_LONG;  // can't encode this line even in an empty buffer
        }
    }

    _bufLen += numCharsWritten;
    ++_linesPending;

    uint32_t const now = millis();
    bool const bufFull = (_bufLen >= kBufBytes);
    bool const manyLines = (_linesPending >= kFlushLines);
    bool const timeUp = (now - _lastFlushMs >= kFlushMs);

    if (bufFull || manyLines || timeUp) {
        if (_file.write(_buf, _bufLen) != _bufLen) {
          return DS_FLUSH_FAILED;
        }
        _bufLen       = 0;
        _linesPending = 0;
        _lastFlushMs  = now;

        if (now - _lastSyncMs >= SYNC_INTERVAL_MS) {
            _file.sync();
            _lastSyncMs = now;
        }
    }

    return 0;
}


/* -----------------------------  end()  ----------------------------------- */
void DataSaverBigSD::end() {
    if (!_ready) {
        return;
    }
    if (_bufLen > 0) {
        _file.write(_buf, _bufLen);
        _bufLen = 0;
    }
    _file.sync();
    _file.close();
    _ready = false;
}

/* -------------------  nextFreeFilePath()  -------------------------------- */
std::string DataSaverBigSD::nextFreeFilePath() {
    std::array<char, FILE_PATH_BUFFER_SIZE> path;
    for (uint16_t flightNumber = 0;; ++flightNumber) {
        snprintf(path.data(), sizeof(path), "/stream-%u.csv", flightNumber);
        if (!sd.exists(path.data())) {
            return std::string(path.data());
        }
    }
}
