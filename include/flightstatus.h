#ifndef FLIGHTSTATUS_H
#define FLIGHTSTATUS_H

#include <deque>
#include <vector>
#include <algorithm>

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
    int hz;
    int n;
    std::deque<double> accelDeque;
    std::deque<double> altitudeDeque;

    double median(std::vector<double> vec);
    bool checkLaunch();
    bool checkCoast();
    bool checkApogee();
    bool checkGround();
  public:
    FlightStatus(int sensorHz = 32);
    void newTelemetry(double acceleration, double altitude);
    Stage getStage();
};

#endif