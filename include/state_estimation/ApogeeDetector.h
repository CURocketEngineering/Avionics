#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include <cstdint>

#include "data_handling/DataPoint.h"
#include "state_estimation/VerticalVelocityEstimator.h"

/**
 * @brief Represents the initial state for initializing the ApogeeDetector.
 */
struct ApogeeDetectorInitialState {
    float initialAltitude;       ///< Initial altitude in meters.
    uint32_t initialTimestamp;   ///< Initial timestamp in milliseconds.
};

/**
 * @brief Detects the apogee (peak altitude) of a rocket flight using estimated altitude and vertical velocity.
 *
 * The detector uses data from a VerticalVelocityEstimator (e.g., a Kalman filter fusing an altimeter and accelerometer).
 * Apogee is detected when:
 *   - The vertical velocity becomes negative.
 *   - The estimated altitude drops by at least a configurable threshold after reaching a maximum.
 *
 * Acceleration assumptions:
 *   - The accelerometer (e.g., LSM6DSOX) measures the "normal force" and outputs ~9.81 m/s² at rest.
 *   - During free fall or coasting, its output approaches 0.
 *   - Therefore, inertial acceleration = measured acceleration − 9.81 m/s².
 */
class ApogeeDetector {
public:
    /**
     * @brief Constructs an ApogeeDetector with a custom drop threshold.
     * @param apogeeThreshold_m Minimum drop in altitude (in meters) to confirm apogee.
     */
    explicit ApogeeDetector(float apogeeThreshold_m);

    /**
     * @brief Constructs an ApogeeDetector with the default threshold of 1.0 meter.
     */
    ApogeeDetector() : ApogeeDetector(1.0F) {}

    /**
     * @brief Initializes the detector with the starting altitude and timestamp.
     * @param initialState Struct containing initial altitude and timestamp.
     */
    void init(ApogeeDetectorInitialState initialState);

    /**
     * @brief Updates the detector using the latest estimated altitude and vertical velocity.
     * @param verticalVelocityEstimator Pointer to the estimator providing current flight state.
     */
    void update(VerticalVelocityEstimator* verticalVelocityEstimator);

    /**
     * @brief Checks if apogee has been detected.
     * @return true if apogee has occurred, false otherwise.
     */
    bool isApogeeDetected() const;

    /**
     * @brief Retrieves the detected apogee.
     * @return A DataPoint containing the timestamp and altitude of apogee.
     *         If not detected, returns {0, 0.0F}.
     */
    DataPoint getApogee() const;

    /**
     * @brief Gets the current estimated altitude.
     * @return Altitude in meters.
     */
    float getEstimatedAltitude() const;

    /**
     * @brief Gets the current estimated vertical velocity.
     * @return Velocity in meters per second.
     */
    float getEstimatedVelocity() const;

    /**
     * @brief Gets the current inertial vertical acceleration.
     * @return Vertical acceleration in m/s², corrected for gravity.
     */
    float getInertialVerticalAcceleration() const;

    /**
     * @brief Gets the configured vertical axis.
     * @return Axis index used as vertical (0=X, 1=Y, 2=Z).
     */
    int8_t getVerticalAxis() const;

    /**
     * @brief Gets the configured vertical direction (+1 or -1).
     * @return 1 if increasing values mean upward, -1 otherwise.
     */
    int8_t getVerticalDirection() const;

private:
    bool apogee_flag = false;              ///< True if apogee has been detected.
    float apogeeThreshold_m = 1.0F;        ///< Minimum drop required to confirm apogee.

    float maxAltitude = 0.0F;              ///< Maximum altitude observed so far.
    uint32_t maxAltitudeTimestamp = 0;     ///< Timestamp when max altitude was reached.
};

#endif // APOGEE_DETECTOR_H
