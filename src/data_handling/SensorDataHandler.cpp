// Originally written by Ethan Anderson -- 2/17/2024

#include "data_handling/SensorDataHandler.h"

SensorDataHandler::SensorDataHandler(std::string name, IDataSaver* ds) {
    this->name = name;
    this->dataSaver = ds;
    this->saveInterval_ms = 0;
    this->lastSaveTime_ms = 0;
}

void SensorDataHandler::restrictSaveSpeed(uint16_t interval_ms){
    this->saveInterval_ms = interval_ms;
}


int SensorDataHandler::addData(DataPoint data){
    // Check if the data is old enough to be saved based on the interval
    if (data.timestamp_ms - lastSaveTime_ms > saveInterval_ms){
        dataSaver->saveDataPoint(data, name);
        lastSaveTime_ms = data.timestamp_ms;
    }

    return 0;
}