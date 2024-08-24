// Originally written by Ethan Anderson -- 2/17/2024
// DataHandler.h

#ifndef SensorDataHandler_H
#define SensorDataHandler_H

#include <cstdint>
#include <cstdlib>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

#include "data_handling/DataPoint.h"
#include "data_handling/CircularArray.h"
#include "data_handling/DataSaver.h"

// This class is used to store data from a sensor
// Stores data in a pair of circular arrays
// The temporal array stores data with evenly spaced timestamps
//
// @param temporalInterval_ms: The interval between each data point in the temporal array
// @param temporalSize_ms: The size of the temporal array in milliseconds
// @param name: The name of sensor as an 8 bit unsigned integer
class SensorDataHandler {
public:
    // Constructor
    SensorDataHandler(uint8_t name, IDataSaver* ds);
    
    // Adds a data point to the sensor data handler
    // Will save it if the saveInterval_ms has passed
    // Returns the status
    int addData(DataPoint data);

    // Sets the minimum time between each data point that is saved to the SD card
    void restrictSaveSpeed(uint16_t interval_ms);

    uint8_t getName() {return name;}

 
protected:
    IDataSaver* dataSaver;
private:
    uint8_t name;
    uint16_t saveInterval_ms; // The minimum time between each data point that is saved to the SD card
    uint32_t lastSaveTime_ms; // The last time a data point was saved to the SD card
};

#endif // DATAHANDLER_H