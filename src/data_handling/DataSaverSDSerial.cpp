#include "ArduinoHAL.h"
#include "data_handling/DataSaverSDSerial.h"
#include <cstdint>

struct SerialData{
    uint32_t timestamp_ms; // 4 bytes
    float data; // 4 bytes
    uint8_t name; // 1 byte
    char dlim [3] = {'\0', '\r', '\n'}; // 3 chars for delimiter
};

/*
* Saves the data to the SD card via serial
* Only uses the first 3 characters of the name, so make sure the first 3 characters are unique
*/
void dataToSDCardSerial(uint8_t name, uint32_t timestamp_ms, float data, HardwareSerial &SD_serial){
    // Optimize for speed
    SerialData serialData;
    serialData.timestamp_ms = timestamp_ms;
    serialData.data = data;
    serialData.name = name;
    SD_serial.write((uint8_t*)&serialData, sizeof(SerialData));
}

DataSaverSDSerial::DataSaverSDSerial(HardwareSerial &SD_serial) : SD_serial(SD_serial) {
    // Nothing to do here
}

int DataSaverSDSerial::saveDataPoint(const DataPoint& dp, uint8_t name){
    dataToSDCardSerial(name, dp.timestamp_ms, dp.data, this->SD_serial);
    return 0;
}