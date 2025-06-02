#include "state_estimation/ApogeeDetector.h"
#include <cmath>

// Constructor
ApogeeDetector::ApogeeDetector(float apogeeThreshold_m)
: apogeeThreshold_m(apogeeThreshold_m)
{}

// Initialize the filter state
void ApogeeDetector::init(ApogeeDetectorInitialState initialState) {
    apogee_flag = false;

    maxAltitude = initialState.initialAltitude;
    maxAltitudeTimestamp = initialState.initialTimestamp;
}

// Update the filter with new sensor data.
void ApogeeDetector::update(VerticalVelocityEstimator* verticalVelocityEstimator) {
    
    const float estimated_altitude_meters = verticalVelocityEstimator->getEstimatedAltitude();

    // Update maximum altitude seen so far.
    if (estimated_altitude_meters > maxAltitude) {
        maxAltitude = estimated_altitude_meters;
        maxAltitudeTimestamp = verticalVelocityEstimator->getTimestamp();
    }
    
    // Apogee detection: if the current estimated altitude is at least apogeeThreshold_m
    // below the maximum and the estimated velocity is negative, mark apogee.
    if (!apogee_flag && ((maxAltitude - estimated_altitude_meters) >= apogeeThreshold_m) && (verticalVelocityEstimator->getEstimatedVelocity() < 0)) {
        apogee_flag = true;
        // (Optionally, you could “snap” the altitude to maxAltitude here.)
    }
}

// Return true if apogee has been detected.
bool ApogeeDetector::isApogeeDetected() const {
    return apogee_flag;
}

// Get the detected apogee as a DataPoint.
// If apogee has not been detected, returns a DataPoint with timestamp 0 and altitude 0.
DataPoint ApogeeDetector::getApogee() const {
    if (apogee_flag) {
        return {maxAltitudeTimestamp, maxAltitude};
    }
    return {0, 0.0F};
}
