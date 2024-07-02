#ifndef ADVANCED_SENSOR_DATA_HANDLER_H
#define ADVANCED_SENSOR_DATA_HANDLER_H

#include "SensorDataHandler.h"

#define MAX_DATA_POINTS_READ_ARRAY 16
#define MAX_DATA_POINTS_TEMPORAL_ARRAY 255



class AdvancedSensorDataHandler : public SensorDataHandler {
  public:
    AdvancedSensorDataHandler(std::string name,
                              IDataSaver* ds,
                              uint16_t temporalInterval_ms,
                              uint16_t temporalSize_ms
                              ); 

    uint16_t getInterval_ms() {return temporalInterval_ms;}
    uint16_t getTemporalSize_ms() {return temporalSize_ms;}
  private:

    uint16_t temporalInterval_ms;
    uint16_t temporalSize_ms;
};

#endif