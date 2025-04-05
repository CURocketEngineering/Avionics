#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include <cstdint>
#include "data_handling/DataPoint.h"
#include "state_estimation/VerticalVelocityEstimator.h"

/*
 * ApogeeDetector uses a simple Kalman filter for fusing acceleration and altimeter data.
 * It assumes:
 *   - The state is [altitude, vertical velocity] (in SI units).
 *   - The process model uses the vertical acceleration as a control input.
 *   - The accelerometer (LSM6DSOX) outputs about 9.81 m/s² when at rest (i.e. it “measures the normal”)
 *     so that in free fall (or after burnout) it outputs 0 m/s².
 *     To get the inertial acceleration we subtract g (9.81 m/s²) from the measured value.
 *   - The altimeter (BMP390) measurement is used to correct the altitude.
 *
 * Apogee detection is based on tracking the maximum altitude reached and then detecting a drop
 * of at least a configurable threshold (e.g. 1 meter) while the velocity becomes negative.
 */
class ApogeeDetector {
public:

    ApogeeDetector(float apogeeThreshold_m);
    // Default constructor with 1.0 m threshold
    ApogeeDetector() : ApogeeDetector(1.0f) {}

    /**
     * Initialize the filter with an initial altitude and timestamp.
     * @param initialAltitude in meters.
     * @param initialTimestamp in milliseconds.
     */
    void init(float initialAltitude, uint32_t initialTimestamp);

    /**
     * Reads the VerticalVelocityEstimator and checks if apogee has occurred.
     * @param verticalVelocityEstimator the VerticalVelocityEstimator to read velocity and altitude from.
     */
    void update(VerticalVelocityEstimator * verticalVelocityEstimator);

    /// Returns true if apogee has been detected.
    bool isApogeeDetected() const;

    /**
     * Get the detected apogee.
     * @return a DataPoint where the timestamp is the time at maximum altitude and data is the altitude.
     *         If apogee has not been detected yet, returns a DataPoint with timestamp 0 and altitude 0.
     */
    DataPoint getApogee() const;

    /// Get the current estimated altitude (meters) from the Kalman filter.
    float getEstimatedAltitude() const;

    /// Get the current estimated vertical velocity (m/s) from the Kalman filter.
    float getEstimatedVelocity() const;

    float getInertialVerticalAcceleration() const;

    int8_t getVerticalAxis() const;

    int8_t getVerticalDirection() const;

private:
    // Has apogee been detected?
    bool apogee_flag;

    // Apogee detection threshold (meters)
    float apogeeThreshold_m;

    // Maximum altitude reached so far (and its timestamp)
    float maxAltitude;
    uint32_t maxAltitudeTimestamp;
};

#endif // APOGEE_DETECTOR_H
