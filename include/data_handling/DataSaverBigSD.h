#pragma once

#include <stdint.h>

#include <string>

#include "ArduinoHAL.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

class DataSaverBigSD : public IDataSaver {
public:
    explicit DataSaverBigSD(uint8_t csPin = 5);

    /** Call once from setup(); returns true on success. */
    bool begin();

    /** Buffer a CSV line (timestamp,name,value).  Flushes to SD in bulk. */
    int  saveDataPoint(DataPoint dp, uint8_t name) override;

    /** Flush any pending bytes and close the file (call before power‑off). */
    void end();

private:
    std::string nextFreeFilePath();                    // /stream‑<n>.csv

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
    char      _buf[kBufBytes];
    uint16_t  _bufLen       = 0;
    uint16_t  _linesPending = 0;
    uint32_t  _lastFlushMs  = 0;
    uint32_t  _lastSyncMs   = 0;
};
