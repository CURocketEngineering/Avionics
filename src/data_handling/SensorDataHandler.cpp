// Originally written by Ethan Anderson -- 2/17/2024

#include "data_handling/SensorDataHandler.h"
#include "ArduinoHAL.h"

SensorDataHandler::SensorDataHandler(uint8_t name, IDataSaver* dataSaver)
    : name(name),
      dataSaver(dataSaver),
      saveInterval_ms(0),
      lastSaveTime_ms(0),
      lastDataPointSaved({0, 0})
{}

void SensorDataHandler::restrictSaveSpeed(uint16_t interval_ms){
    this->saveInterval_ms = interval_ms;
}


int SensorDataHandler::addData(DataPoint data){
    // Check if the data is old enough to be saved based on the interval
    if (data.timestamp_ms - lastSaveTime_ms > saveInterval_ms) {
        dataSaver->saveDataPoint(data, name);
        lastSaveTime_ms = data.timestamp_ms;
        lastDataPointSaved = data;
    }

    return 0;
}