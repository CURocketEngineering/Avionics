#include "state_estimation/ApogeeDetector.h"
#include <cmath>

// Constructor
ApogeeDetector::ApogeeDetector(float apogeeThreshold_m)
: apogeeThreshold_m(apogeeThreshold_m)
{}

// Initialize the filter state
void ApogeeDetector::init(ApogeeDetectorInitialState initialState) {
    apogeeDetected = false;

    maxAltitude = initialState.initialAltitude;
    maxAltitudeTimestamp = initialState.initialTimestamp;
}

// Update the filter with new sensor data.
void ApogeeDetector::update(VerticalVelocityEstimator* verticalVelocityEstimator) {
    
    const float estimatedAltitude_m = verticalVelocityEstimator->getEstimatedAltitude();

    // Update maximum altitude seen so far.
    if (estimatedAltitude_m > maxAltitude) {
        maxAltitude = estimatedAltitude_m;
        maxAltitudeTimestamp = verticalVelocityEstimator->getTimestamp();
    }
    
    // Apogee detection: if the current estimated altitude is at least apogeeThreshold_m
    // below the maximum and the estimated velocity is negative, mark apogee.
    if (!apogeeDetected && ((maxAltitude - estimatedAltitude_m) >= apogeeThreshold_m) && (verticalVelocityEstimator->getEstimatedVelocity() < 0)) {
        apogeeDetected = true;
        // (Optionally, you could “snap” the altitude to maxAltitude here.)
    }
}

// Return true if apogee has been detected.
bool ApogeeDetector::isApogeeDetected() const {
    return apogeeDetected;
}

// Get the detected apogee as a DataPoint.
// If apogee has not been detected, returns a DataPoint with timestamp 0 and altitude 0.
DataPoint ApogeeDetector::getApogee() const {
    if (apogeeDetected) {
        return {maxAltitudeTimestamp, maxAltitude};
    }
    return {0, 0.0F};
}
