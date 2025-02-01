#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include <cstdint>
#include "data_handling/DataPoint.h"

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
    /**
     * Constructor.
     * @param accelNoiseVariance Process noise variance due to acceleration noise (default 0.25, e.g. (0.5 m/s²)²).
     * @param altimeterNoiseVariance Measurement noise variance of the altimeter (default 1.0).
     * @param apogeeThreshold Minimum drop (in meters) from the maximum altitude to flag apogee (default 1.0 m).
     */
    ApogeeDetector(float accelNoiseVariance = 0.25f, float altimeterNoiseVariance = 1.0f, float apogeeThreshold = 1.0f);

    /**
     * Initialize the filter with an initial altitude and timestamp.
     * @param initialAltitude in meters.
     * @param initialTimestamp in milliseconds.
     */
    void init(float initialAltitude, uint32_t initialTimestamp);

    /**
     * Update the detector with new sensor data.
     *
     * The three acceleration DataPoints correspond to x, y, and z (vertical) axes.
     * The altimeter DataPoint contains the altitude measurement.
     *
     * @param accelX Accelerometer reading for the x-axis.
     * @param accelY Accelerometer reading for the y-axis.
     * @param accelZ Accelerometer reading for the z-axis (vertical).
     * @param altimeter Altimeter reading.
     */
    void update(const DataPoint &accelX, const DataPoint &accelY, const DataPoint &accelZ, const DataPoint &altimeter);

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

private:
    // Kalman filter state (altitude and vertical velocity)
    float state_alt; // altitude (meters)
    float state_vel; // vertical velocity (m/s)

    // Covariance matrix (2x2)
    float P[2][2];

    // Last update timestamp (ms)
    uint32_t lastTimestamp;

    // Has the filter been initialized?
    bool initialized;

    // Has apogee been detected?
    bool apogee_flag;

    // Noise parameters
    float accelNoiseVariance;    // acceleration noise variance (process noise)
    float altimeterNoiseVariance; // measurement noise variance

    // Apogee detection threshold (meters)
    float apogeeThreshold;

    // Gravitational acceleration constant (m/s²)
    const float g = 9.81f;

    // Maximum altitude reached so far (and its timestamp)
    float maxAltitude;
    uint32_t maxAltitudeTimestamp;
};

#endif // APOGEE_DETECTOR_H
