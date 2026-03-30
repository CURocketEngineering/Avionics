#include "state_estimation/ApogeeDetector.h"
#include <cmath>

// Constructor
ApogeeDetector::ApogeeDetector(float apogeeThreshold_m)
: apogeeThreshold_m_(apogeeThreshold_m)
{}

// Initialize the filter state
void ApogeeDetector::init(ApogeeDetectorInitialState initialState) {
    apogeeDetected_ = false;

    maxAltitude_ = initialState.initialAltitude;
    maxAltitudeTimestamp_ = initialState.initialTimestamp;
}

// Update the filter with new sensor data.
void ApogeeDetector::update(VerticalVelocityEstimator* verticalVelocityEstimator) {
    
    const float estimatedAltitude_m = verticalVelocityEstimator->getEstimatedAltitude();

    // Update maximum altitude seen so far.
    if (estimatedAltitude_m > maxAltitude_) {
        maxAltitude_ = estimatedAltitude_m;
        maxAltitudeTimestamp_ = verticalVelocityEstimator->getTimestamp();
    }
    
    // Apogee detection: if the current estimated altitude is at least apogeeThreshold_m
    // below the maximum and the estimated velocity is negative, mark apogee.
    if (!apogeeDetected_ && ((maxAltitude_ - estimatedAltitude_m) >= apogeeThreshold_m_) && (verticalVelocityEstimator->getEstimatedVelocity() < 0)) {
        apogeeDetected_ = true;
        // (Optionally, you could “snap” the altitude to maxAltitude here.)
    }
}

// Return true if apogee has been detected.
bool ApogeeDetector::isApogeeDetected() const {
    return apogeeDetected_;
}

// Get the detected apogee as a DataPoint.
// If apogee has not been detected, returns a DataPoint with timestamp 0 and altitude 0.
DataPoint ApogeeDetector::getApogee() const {
    if (apogeeDetected_) {
        return {maxAltitudeTimestamp_, maxAltitude_};
    }
    return {0, 0.0F};
}
