// Originally written by Ethan Anderson -- 2/17/2024

#include "data_handling/SensorDataHandler.h"
#include "ArduinoHAL.h"

SensorDataHandler::SensorDataHandler(uint8_t name, IDataSaver* dataSaver_in)
    : dataSaver(dataSaver_in),
      name_(name),
      saveInterval_ms_(0U),
      lastSaveTime_ms_(0UL),
      lastDataPointSaved_({0UL, 0.0F})
{}

void SensorDataHandler::restrictSaveSpeed(uint16_t interval_ms){
    this->saveInterval_ms_ = interval_ms;
}


int SensorDataHandler::addData(DataPoint data){
    // Check if the data is old enough to be saved based on the interval
    if (data.timestamp_ms - lastSaveTime_ms_ >= saveInterval_ms_) {
        dataSaver->saveDataPoint(data, name_);
        lastSaveTime_ms_ = data.timestamp_ms;
        lastDataPointSaved_ = data;
    }

    return 0;
}
