#ifndef DATA_SAVER_BIG_SD_H
#define DATA_SAVER_BIG_SD_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

#include <string>
#include <stdint.h>

class DataSaverBigSD : public IDataSaver
{
public:
    explicit DataSaverBigSD(uint8_t csPin = 5);

    /** Call once from setup(); returns true on success. */
    bool begin();

    /** Append “idx,name,value\n” to the current log file. */
    int  saveDataPoint(DataPoint dp, uint8_t name) override;

private:
    uint8_t     _csPin;
    bool        _ready {false};
    std::string _filePath;

    /** Returns the first unused “/stream‑<n>.csv” path. */
    std::string nextFreeFilePath();
};

#endif  // DATA_SAVER_BIG_SD_H
