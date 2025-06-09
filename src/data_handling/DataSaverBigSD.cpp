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
    _file.preAllocate(4UL * 1024 * 1024);

    Serial.print(F("Logging to ")); Serial.println(_filePath.c_str());

    _bufLen       = 0;
    _linesPending = 0;
    _lastFlushMs  = millis();
    _lastSyncMs   = _lastFlushMs;
    _ready        = true;
    return true;
}

/* -----------------------  saveDataPoint()  ------------------------------- */
int DataSaverBigSD::saveDataPoint(const DataPoint& dp, uint8_t name) {
    if (!_ready) {
        return -1;
    }

    // Reserve space left
    size_t remaining = sizeof(_buf) - _bufLen;

    // Format the new line
    int n = snprintf(_buf + _bufLen, remaining, "%lu,%u,%.6f\n",
                     static_cast<long unsigned int>(dp.timestamp_ms), name, dp.data);

    // Check snprintf result
    if (n <= 0 || (size_t)n >= remaining) {
        // Flush current buffer and try again if this line couldn’t fit
        if (_file.write(_buf, _bufLen) != _bufLen) {
          return -3;
        }
        _file.sync();  // just to be extra safe during debug
        _bufLen = 0;
        _linesPending = 0;

        // Try again (safe now)
        remaining = sizeof(_buf);
        n = snprintf(_buf, remaining, "%lu,%u,%.6f\n", static_cast<long unsigned int>(dp.timestamp_ms), name, dp.data);
        if (n <= 0 || (size_t)n >= remaining) {
          return -4;  // can't encode this line even in an empty buffer
        }
    }

    _bufLen += n;
    ++_linesPending;

    uint32_t const now = millis();
    bool const bufFull = (_bufLen >= kBufBytes);
    bool const manyLines = (_linesPending >= kFlushLines);
    bool const timeUp = (now - _lastFlushMs >= kFlushMs);

    if (bufFull || manyLines || timeUp) {
        if (_file.write(_buf, _bufLen) != _bufLen) {
          return -5;
        }
        _bufLen       = 0;
        _linesPending = 0;
        _lastFlushMs  = now;

        if (now - _lastSyncMs >= 1000) {
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
    if (_bufLen) {
        _file.write(_buf, _bufLen);
        _bufLen = 0;
    }
    _file.sync();
    _file.close();
    _ready = false;
}

/* -------------------  nextFreeFilePath()  -------------------------------- */
std::string DataSaverBigSD::nextFreeFilePath() {
    char path[32];
    for (uint16_t n = 0;; ++n) {
        snprintf(path, sizeof(path), "/stream-%u.csv", n);
        if (!sd.exists(path)) {
            return std::string(path);
        }
    }
}
