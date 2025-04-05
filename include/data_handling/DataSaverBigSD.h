/************************  DataSaverBigSD.h  ************************/
#ifndef DATA_SAVER_BIG_SD_H
#define DATA_SAVER_BIG_SD_H

// Saves DataPoint objects to a FAT32 SD card as a CSV stream:
// idx,name,value\n

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

#include "ArduinoHAL.h"

#include <string>

class DataSaverBigSD : public IDataSaver
{
public:
    explicit DataSaverBigSD(uint8_t csPin = 5);   // default CS pin = 5

    /** Initialise the SD card and create the next free log file. */
    bool begin();

    /** Append “idx,name,value\n” to the current log file. */
    int  saveDataPoint(DataPoint dp, uint8_t name) override;

private:
    uint8_t     _csPin;
    bool        _ready{false};
    std::string _filePath;

    /** Return a free path of the form “/stream‑<n>.csv”. */
    std::string nextFreeFilePath() const;
};

#endif  // DATA_SAVER_BIG_SD_H
