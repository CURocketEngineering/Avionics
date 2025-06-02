#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

#include <array>
#include <cstdint>

#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"



constexpr float MINIMUM_DELTA_T_S = 0.01f; // Minimum delta time for updates (10ms)
constexpr float MILLISECONDS_TO_SECONDS = 0.001f; // Conversion factor from milliseconds to seconds

struct alignas(8) NoiseVariances {
    float accelNoiseVar;
    float altimeterNoiseVar;
};

struct InitialState {
    float initialAltitude;
    uint32_t initialTimestamp;
};



/**
 * VerticalVelocityEstimator provides a 1D Kalman filter that fuses altimeter and acceleration
 * data to estimate altitude and vertical velocity. It is adapted from the logic in 
 * ApogeeDetector but excludes the apogee detection mechanism. 
 *
 * Assumptions:
 *  - The state is [altitude, vertical velocity] (in SI units).
 *  - The process model uses the vertical acceleration as a control input.
 *  - The accelerometer outputs roughly +9.81 m/s² when at rest (measuring gravity).
 *    In free fall it reads ~0 m/s². Hence we subtract g (9.81 m/s²) from the raw 
 *    accelerometer reading to get the rocket’s inertial acceleration.
 *  - The altimeter measurement is used to correct the altitude.
 *
 * The user of this class may simply call update(...) whenever new sensor readings 
 * arrive and retrieve the current altitude and velocity estimates.
 */
class VerticalVelocityEstimator {
public:
    /**
     * Constructor.
     * @param accelNoiseVariance    Process noise variance due to unknown acceleration 
     *                             changes (e.g. (0.5 m/s²)² = 0.25).
     * @param altimeterNoiseVariance Measurement noise variance of the altimeter 
     *                             (e.g. 1.0 for 1m²).
     */
    VerticalVelocityEstimator(NoiseVariances noise = {0.25f, 1.0f});

    /**
     * Initialize the filter with an initial altitude and timestamp.
     * @param initialAltitude  in meters.
     * @param initialTimestamp in milliseconds.
     */
    void init(InitialState initialState);

    /**
     * Update the estimator with new sensor data.
     *
     * The three acceleration DataPoints correspond to x, y, and z (vertical) axes.
     * The altimeter DataPoint contains the altitude measurement.
     *
     * @param accelX    Accelerometer reading for the x-axis.
     * @param accelY    Accelerometer reading for the y-axis.
     * @param accelZ    Accelerometer reading for the z-axis (vertical).
     * @param altimeter Altimeter reading.
     */
    void update(const AccelerationTriplet accel,
                const DataPoint &altimeter);

    /**
     * @return Current estimated altitude (meters).
     */
    virtual float getEstimatedAltitude() const;

    /**
     * @return Current estimated vertical velocity (m/s).
     */
    virtual float getEstimatedVelocity() const;

    /**
     * @return The timestamp of the last update (milliseconds).
     */
    virtual uint32_t getTimestamp() const;

    /**
     * @return Computed inertial vertical acceleration (m/s²), i.e., raw_accel - g.
     */
    virtual float getInertialVerticalAcceleration() const;

    /**
     * @return The index of the axis determined to be vertical (0 = x, 1 = y, 2 = z).
     */
    virtual int8_t getVerticalAxis() const;

    /**
     * @return The direction along that axis (+1 if positive direction is “up,” -1 if negative).
     */
    virtual int8_t getVerticalDirection() const;

private:
    /**
     * Determine which of the three accelerometer axes is vertical, based on the largest
     * magnitude reading. Also sets the direction (+1 or -1) depending on the sign.
     */
    void determineVerticalAxis(const std::array<float, 3>& rawAcl);

private:
    // Kalman filter state: altitude (m), vertical velocity (m/s).
    float state_alt;
    float state_vel;

    // Covariance matrix (2x2).
    float P[2][2] = {{}, {}};

    // Time of last update (milliseconds).
    uint32_t lastTimestamp_ms;

    // True after init() has been called.
    bool initialized;

    // Noise parameters.
    float accelNoiseVariance;      // Acceleration noise variance (process noise).
    float altimeterNoiseVariance;  // Altimeter noise variance (measurement noise).

    // Gravity constant (m/s²).
    const float g = 9.81f;

    // Which axis is vertical, and in what direction?
    int8_t verticalAxis;  
    int8_t verticalDirection;  
    bool verticalAxisDetermined;

    // Latest computed inertial acceleration along the vertical axis.
    float inertialVerticalAcceleration;
};

#endif // VELOCITY_ESTIMATOR_H
