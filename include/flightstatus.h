#ifndef FLIGHTSTATUS_H
#define FLIGHTSTATUS_H

#include <deque>
#include <vector>
#include <algorithm>
#include "SensorDataHandler.h"

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
    SensorData * xac;
    SensorData * yac;
    SensorData * zac;
  public:
    FlightStatus(SensorData * xac, SensorData * yac, SensorData * zac);
    void setupSDHs();
    void update(HardwareSerial * SD_serial);
    Stage getStage();

    
};

#endif