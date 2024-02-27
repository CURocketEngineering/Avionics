#include <flightstatus.h>

FlightStatus::FlightStatus(int sensorHz = 32): altitudeDeque(128, 0), accelDeque(128,0) {
    // Why 128 to 0 when passing in?
    flightStage = ARMED;
    hz = sensorHz;
    n = hz * 2; // revisit for sensitivity, 2 seconds is the duration of apogeee
}

double FlightStatus::median(std::vector<double> vec){
    int size = vec.size();
    sort(vec.begin(), vec.end());
    if (size % 2 != 0)
        return (double)vec[size/2];
    return (double)(vec[(size-1)/2] + vec[size/2])/2.0;
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

    // rename lm/fm? what does that mean?
    // smaller intervals?
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

bool FlightStatus::checkGround() {
    // If altitude is less than 0, then rocket has hit the ground
    // Reads in altitude as a deque
    // If average of last two seconds is lower than 20 --> ground
    std::vector<double> lm(altitudeDeque.cend() - n, altitudeDeque.cend());

    double lmMed = median(lm);

    return lmMed < 20;
}

void FlightStatus::newTelemetry(double accelCheck, double altCheck) {
    altitudeDeque.pop_front();
    altitudeDeque.push_back(altCheck);

    accelDeque.pop_front();
    accelDeque.push_back(accelCheck);

    if(checkLaunch() && flightStage == ARMED) {
        flightStage = ASCENT;
    }
    if(checkCoast() && flightStage == ASCENT) {
        flightStage = COAST;
    }
    if(checkApogee() && flightStage == COAST)
    {
        flightStage = APOGEE;
    } // why no pause?
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