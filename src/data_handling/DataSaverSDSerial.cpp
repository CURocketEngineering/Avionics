#include "ArduinoHAL.h"
#include "data_handling/DataSaverSDSerial.h"
#include <cstdint>

#pragma pack(push, 1)
struct SerialData {
    uint32_t timestamp_ms;
    float data;
    uint8_t name;
    std::array<char, 3> dlim;
};
#pragma pack(pop)


/*
* Saves the data to the SD card via serial
* Only uses the first 3 characters of the name, so make sure the first 3 characters are unique
*/
void dataToSDCardSerial(uint8_t name, uint32_t timestamp_ms, float data, HardwareSerial &SD_serial) { // NOLINT(bugprone-easily-swappable-parameters)
    SerialData serialData = {timestamp_ms, data, name, {{'\0', '\r', '\n'}}};
    SD_serial.write(reinterpret_cast<const uint8_t*>(&serialData), sizeof(SerialData));
}

DataSaverSDSerial::DataSaverSDSerial(HardwareSerial &SD_serial) : SD_serial(SD_serial) {
    // Nothing to do here
}

int DataSaverSDSerial::saveDataPoint(const DataPoint& dataPoint, uint8_t name){
    dataToSDCardSerial(name, dataPoint.timestamp_ms, dataPoint.data, this->SD_serial);
    return 0;
}