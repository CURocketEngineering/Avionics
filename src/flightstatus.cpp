#include "flightstatus.h"

FlightStatus::FlightStatus(int sensorHz = 32): altitudeDeque(128, 0) {
    flightStage = ARMED;
    currentAccel = 0;
    hz = sensorHz;
}

double FlightStatus::median(std::vector<double> vec){
    int size = vec.size();
    sort(vec.begin(), vec.end());
    if (size % 2 != 0)
        return (double)vec[size/2];
    return (double)(vec[(size-1)/2] + vec[size/2])/2.0;
}

bool FlightStatus::checkLaunch() {
    return false;
}

bool FlightStatus::checkCoast() {
    return false;
}

bool FlightStatus::checkApogee() {
    std::vector<double> lm(altitudeDeque.cend() - 16, altitudeDeque.cend());
    std::vector<double> fm(altitudeDeque.cend() - 48, altitudeDeque.cend() - 16);

    double lmMed = median(lm);
    double fmMed = median(fm);

    return lmMed < fmMed;
}

bool FlightStatus::checkGround() {
    std::vector<double> lm(altitudeDeque.cend() - 16, altitudeDeque.cend());

    double lmMed = median(lm);

    return lmMed < 20;
}

void FlightStatus::newTelemetry(double acceleration, double altitude) {
    altitudeDeque.pop_front();
    altitudeDeque.push_back(altitude);

    currentAccel = acceleration;

    if(checkLaunch() && flightStage == ARMED) {
        flightStage = ASCENT;
    }
    if(checkCoast() && flightStage == ASCENT) {
        flightStage = COAST;
    }
    if(checkApogee() && flightStage == COAST)
    {
        flightStage = APOGEE;
    }
    if(flightStage == APOGEE) {
        flightStage = DESCENT;
    }
    if(checkGround() && flightStage == DESCENT) {
        flightStage = ONGROUND;
    }
}

Stage FlightStatus::getStage() {
    return flightStage;
}


 