#ifndef FLIGHTSTATUS_H
#define FLIGHTSTATUS_H

#include <deque>
#include <vector>
#include <algorithm>
#include "ArduinoHAL.h"
#include "data_handling/AdvancedSensorDataHandler.h"

enum Stage {
    ARMED,
    ASCENT,
    COAST,
    APOGEE,
    DESCENT,
    ONGROUND,
};

class FlightStatus {
  private:
    Stage flightStage;
    double median(std::vector<double> vec);

    // Using SensorDataHandlers to check for launch
    bool checkLaunchSDH(HardwareSerial * SD_serial); 

    bool checkLaunch();
    bool checkCoast();
    bool checkApogee();
    bool checkGround();

    int validatedSDHs = 0;
    AdvancedSensorDataHandler * xac;
    AdvancedSensorDataHandler * yac;
    AdvancedSensorDataHandler * zac;
  public:
    FlightStatus(SensorDataHandler * xac, SensorDataHandler * yac, SensorDataHandler * zac);
    void setupSDHs();
    void update(HardwareSerial * SD_serial);
    Stage getStage();

    
};

#endif