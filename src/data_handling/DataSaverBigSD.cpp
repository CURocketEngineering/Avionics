/************************  DataSaverBigSD.cpp  ************************/
#include "data_handling/DataSaverBigSD.h"
#include "ArduinoHAL.h"      // For Serial on ESP32 / STM32‑Arduino, etc.

DataSaverBigSD::DataSaverBigSD(uint8_t csPin) : _csPin(csPin) {}

/* ------------------------------------------------------------------ */
bool DataSaverBigSD::begin()
{
    Serial.print("Initialising SD card … ");
    pinMode(_csPin, OUTPUT);
    if (!SD.begin(_csPin)) {
        Serial.println("failed!");
        return false;
    }
    Serial.println("done.");

    _filePath = nextFreeFilePath();                 // e.g. “/stream‑3.csv”

    // Create the file (so later FILE_APPEND is guaranteed to succeed).
    File f = SD.open(_filePath.c_str(), FILE_WRITE);
    if (!f) {
        Serial.println("Could not create data file!");
        return false;
    }
    // Optional header row – comment out if you don’t want it.
    // f.println("idx,name,value");
    f.close();

    Serial.print("Logging to ");
    Serial.println(_filePath.c_str());

    _ready = true;
    return true;
}

/* ------------------------------------------------------------------ */
std::string DataSaverBigSD::nextFreeFilePath() const
{
    char path[24];
    uint16_t n = 0;
    do {
        snprintf(path, sizeof(path), "/stream-%u.csv", n++);
    } while (SD.exists(path));
    return std::string(path);
}

/* ------------------------------------------------------------------ */
int DataSaverBigSD::saveDataPoint(DataPoint dp, uint8_t name)
{
    if (!_ready)              return -1;            // not initialised
    File f = SD.open(_filePath.c_str(), FILE_APPEND);
    if (!f)                   return -2;            // open error

    /* ----‑‑ build the CSV line ----‑‑
       Adjust the field names to match your DataPoint definition.        */
    f.print(dp.idx);          f.print(',');
    f.print(name);            f.print(',');
    f.println(dp.value);

    f.close();
    return 0;                                       // success
}
