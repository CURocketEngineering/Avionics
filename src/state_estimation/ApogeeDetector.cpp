#include "state_estimation/ApogeeDetector.h"
#include <cmath>

// Constructor
ApogeeDetector::ApogeeDetector(float apogeeThreshold_m = 1.0f)
: apogee_flag(false),
  apogeeThreshold_m(apogeeThreshold_m),
  maxAltitude(0.0f),
  maxAltitudeTimestamp(0)
{}

// Initialize the filter state
void ApogeeDetector::init(float initialAltitude, uint32_t initialTimestamp) {
    apogee_flag = false;

    maxAltitude = initialAltitude;
    maxAltitudeTimestamp = initialTimestamp;
}

// Update the filter with new sensor data.
void ApogeeDetector::update(VerticalVelocityEstimator* verticalVelocityEstimator) {
    
    float estimated_altitude_meters = verticalVelocityEstimator->getEstimatedAltitude();

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
        return DataPoint(maxAltitudeTimestamp, maxAltitude);
    }
    return DataPoint(0, 0.0f);
}
