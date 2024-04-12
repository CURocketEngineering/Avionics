#include "flightstatus.h"
#include <string>
#include <Arduino.h>
//Going to add globalHistoricalData

FlightStatus::FlightStatus(int sensorHz): altitudeDeque(512, 0), accelDeque(512,0) {
    // Why 128 to 0 when passing in?
    flightStage = ARMED;
    hz = sensorHz;
    n = hz * 2; // revisit for sensitivity, 2 seconds is the duration of apogeee
}

double FlightStatus::median(std::vector<double> vec) {
    // Sorts the vector and returns the median
    std::sort(vec.begin(), vec.end());
    if(vec.size() % 2 == 0) {
        return (vec[vec.size()/2 - 1] + vec[vec.size()/2]) / 2;
    }
    return vec[vec.size()/2];
}

bool FlightStatus::checkLaunch() {
    // If acceleration shoots up, then the rocket has launched
    // Reads in acceleration as a deque
    // If average of last 2 seconds are greater than 11 m/s^2 --> liftoff
    std::vector<double> lm(accelDeque.cend() - n, accelDeque.cend());

    double lmMed = median(lm);

    return lmMed > 11;
}

bool FlightStatus::checkCoast() {
    // If acceleration suddenly drops, then the engines have cut off
    // Reads in acceleration as a deque
    // If average of last 2 seconds suddenly drops --> coast
    // also if rocket is not on ground and prior statement is true --> coast

    std::vector<double> lm(accelDeque.cend() - n, accelDeque.cend());
    std::vector<double> fm(accelDeque.cend() - 3*n, accelDeque.cend() - n);

    double lmMed = median(lm);
    double fmMed = median(fm);
    
    return fmMed > lmMed;
}

bool FlightStatus::checkApogee() {
    // If altitude stops increasing, then the rocket has reached apogee
    // Reads in altitude as a deque
    // If average of last two seconds is lower than recent average --> apogee

    std::vector<double> lm(altitudeDeque.cend() - n, altitudeDeque.cend());
    std::vector<double> fm(altitudeDeque.cend() - 3* n, altitudeDeque.cend() - n);

    double lmMed = median(lm);
    double fmMed = median(fm);

    return lmMed < fmMed;
}

bool FlightStatus::checkDescent() {
    // If acceleration starts to increase and launch is false,
    // then rocket is descending
    // Reads in acceleration as a deque
    // If average of last 2 seconds suddenly drops --> coast
    // also if rocket is not on ground and prior statement is true --> coast

    std::vector<double> lm(accelDeque.cend() - n, accelDeque.cend());
    std::vector<double> fm(accelDeque.cend() - 3*n, accelDeque.cend() - n);

    double lmMed = median(lm);
    double fmMed = median(fm);
    
    return fmMed < lmMed;
}

bool FlightStatus::checkGround() {
    // If altitude is less than 0, then rocket has hit the ground
    // Reads in altitude as a deque
    // If average of last two seconds is lower than 20 --> ground
    std::vector<double> lm(altitudeDeque.cend() - n, altitudeDeque.cend());

    double lmMed = median(lm);

    return lmMed < 20;
}

void FlightStatus::newTelemetry(double acceleration, double altitude) {
    //Creating altitude and acceleration deques
    //ascent -> coast -> apogee -> descent -> on ground must happen in order
    altitudeDeque.pop_front();
    altitudeDeque.push_back(altitude);

    accelDeque.pop_front();
    accelDeque.push_back(acceleration);

    // std::string outStr = ("Acceleration: " + std::to_string(acceleration) + " Altitude: " + std::to_string(altitude));
    // Serial.println(outStr.c_str());

    if(checkLaunch() && flightStage == ARMED) {
        flightStage = ASCENT;
    }
    // Serial.println("Checking coast");
    if(checkCoast() && flightStage == ASCENT) {
        flightStage = COAST;
    }
    // Serial.println("Checking apogee");
    if(checkApogee() && flightStage == COAST)
    {
        flightStage = APOGEE;
    }
    // Serial.println("Checking descent");
    //adding checkdescent for time between apogee and desending
    if(checkDescent() && flightStage == APOGEE) {
        flightStage = DESCENT;
    }
    // Serial.println("Checking ground");
    if(checkGround() && flightStage == DESCENT) {
        flightStage = ONGROUND;
    }
}

Stage FlightStatus::getStage() {
    return flightStage;
}

std::string FlightStatus::getStageString() {
    switch(flightStage) {
        case ARMED:
            return "ARMED";
        case ASCENT:
            return "ASCENT";
        case COAST:
            return "COAST";
        case APOGEE:
            return "APOGEE";
        case DESCENT:
            return "DESCENT";
        case ONGROUND:
            return "ONGROUND";
    }
    return "ERROR";
}