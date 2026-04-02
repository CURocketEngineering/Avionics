#ifndef ORIENTATION_ESTIMATOR_H
#define ORIENTATION_ESTIMATOR_H

#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/States.h"

/**
 * @brief Orientation estimator interface.
 * 
 * This class defines the interface for orientation estimation algorithms. It provides a method to 
 * update the estimator with new sensor data and a method to retrieve the current orientation estimate.
 * Uses Madgwick's algorithm for sensor fusion, which combines accelerometer and gyroscope data to estimate orientation.
 */
class OrientationEstimator {
public:
    float q0, q1, q2, q3;   // quaternion
    float betaPad;
    float betaFlight;
    float beta; // determines how much the algorithm relies on the accelerometer vs the gyroscope. Higher beta means more reliance on accel.
    float roll, pitch, yaw; // Euler angles in degrees
    bool hasLaunched;
    uint32_t lastUpdateTime;   // timestamp of the last update in milliseconds

    OrientationEstimator(float gainPad = 0.05f, float gainFlight = 0.005f)
        : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
          betaPad(gainPad), betaFlight(gainFlight), lastUpdateTime(0),
          roll(0.0f), pitch(0.0f), yaw(0.0f), hasLaunched(false) {}

    void update(AccelerationTriplet accel, GyroTriplet gyro, MagTriplet mag, uint32_t currentTime);
    void updateFullAHRS(AccelerationTriplet accel, GyroTriplet gyro, MagTriplet mag, uint32_t currentTime);
    void updateIMU(AccelerationTriplet accel, GyroTriplet gyro, uint32_t currentTime);
    void getEuler();

    Quanternion getQuanternion() const{
        Quanternion q = {q0, q1, q2, q3};
        return q;
    }
    void launchDetected() { hasLaunched = true;}
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
};


#endif