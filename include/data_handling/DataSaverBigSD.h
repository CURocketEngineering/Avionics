#pragma once

#include <stdint.h>

#include <string>

#include "ArduinoHAL.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

constexpr size_t PRE_ALLOCATE_SIZE_MB = 4; // Define the size in MB
constexpr size_t BYTES_PER_MB = 1024 * 1024; // Define the number of bytes in 1 MB
constexpr uint32_t SYNC_INTERVAL_MS = 1000;
constexpr size_t FILE_PATH_BUFFER_SIZE = 32; // Buffer size for file path

enum BigSDDataSaverError {
    DS_SUCCESS = 0,            // Operation successful
    DS_NOT_READY = -1,         // DataSaver is not ready
    DS_BUFFER_WRITE_FAILED = -3, // Failed to write buffer to file
    DS_LINE_TOO_LONG = -4,     // Line couldn't fit in an empty buffer
    DS_FLUSH_FAILED = -5       // Failed to flush buffer to file
};

class DataSaverBigSD : public IDataSaver {
public:
    explicit DataSaverBigSD(uint8_t csPin = 5);

    /** Call once from setup(); returns true on success. */
    bool begin();

    /** Buffer a CSV line (timestamp,name,value).  Flushes to SD in bulk. */
    int  saveDataPoint(const DataPoint& dp, uint8_t name) override;

    /** Flush any pending bytes and close the file (call before power‑off). */
    void end();

private:
    static std::string nextFreeFilePath();                    // /stream‑<n>.csv

    uint8_t  _csPin;
    bool     _ready {false};

    /* single shared SdFat instance */
    static SdFat sd;
    using SdFile_t = File32;

    SdFile_t    _file;
    std::string _filePath;

    /* buffering parameters */
    static constexpr uint16_t kBufBytes   = 512;   // one SD sector
    static constexpr uint16_t kFlushLines = 64;    // flush after N lines
    static constexpr uint32_t kFlushMs    = 200;   // or after 200 ms

    /* buffering state */
    char      _buf[kBufBytes] = {};
    uint16_t  _bufLen       = 0;
    uint16_t  _linesPending = 0;
    uint32_t  _lastFlushMs  = 0;
    uint32_t  _lastSyncMs   = 0;
};
