#ifndef ORIENTATION_ESTIMATOR_H
#define ORIENTATION_ESTIMATOR_H

#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"

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
    float beta;             // algorithm gain
    float sampleFreq;       // sample frequency in Hz
    float roll, pitch, yaw; // Euler angles in degrees

    OrientationEstimator(float freq, float gain = 0.1f)
        : q0(1), q1(0), q2(0), q3(0),
          beta(gain), sampleFreq(freq) {}

    void update(AccelerationTriplet accel, GyroTriplet gyro, MagTriplet mag, float dt);

    void getEuler();

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
};






#endif