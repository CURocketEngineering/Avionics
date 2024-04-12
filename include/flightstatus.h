#ifndef FLIGHTSTATUS_H
#define FLIGHTSTATUS_H

#include <deque>
#include <vector>
#include <algorithm>
#include "SensorDataHandler.h"

//enumeration decleration of rocket positions
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

    Stage flightStage; //holds the stage rocket is in
    int hz; // Frequency calculations use [Hertz]
    int n; //Random Variable for intermediete calculations

    //creates vector of structs which hold data value and time
    std::vector<DataPoint> theData;  //acceleration vector
    std::vector<DataPoint> theDataAlt; //altitude vector
    //continuously updating deque 
    std::deque<double> accelDeque; //acceleration
    std::deque<double> altitudeDeque; //altitude

    double median(std::vector<double> vec); //median function to calculate averages
    bool checkLaunch(); 
    bool checkCoast();
    bool checkApogee();
    bool checkDescent();
    bool checkGround();
    void frequencyCheck(SensorData someData); //error check for frequency

  public:
    FlightStatus(int sensorHz = 32);
    void newTelemetry1(double acceleration, double altitude); //telemetry for flightstatus
    void newTelemetry(SensorData* acceleration, SensorData* altitude); //telemtry for new code (flightStatus2)
    Stage getStage();
};

#endif