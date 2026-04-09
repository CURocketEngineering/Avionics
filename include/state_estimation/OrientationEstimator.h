#ifndef ORIENTATION_ESTIMATOR_H
#define ORIENTATION_ESTIMATOR_H

#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/States.h"

/**
 * @brief Orientation estimator using Madgwick's algorithm for sensor fusion.
 * 
 * This class estimates the orientation of the rocket in 3D space using data from the accelerometer, 
 * gyroscope, and magnetometer. When not in flight, it uses the full AHRS algorithm to fuse all three sensors. 
 * Once launch is detected, it relies on gyro data only for orientation updates to avoid accelerometer and magnetometer
 * disturbances during flight.
 */
class OrientationEstimator {
public:
    OrientationEstimator(float gainPad = 0.05f, float gainFlight = 0.005f)
        : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
          betaPad(gainPad), betaFlight(gainFlight),
          roll(0.0f), pitch(0.0f), yaw(0.0f), hasLaunched(false), lastUpdateTime(0) {}

    /**
     * @brief Update the orientation estimator with new sensor data.
     * This method should be called whenever new accelerometer, gyroscope, or magnetometer data is available.
     * Calls updateFullAHRS or updateIMU when on pad, directly updates orientation estimate
     * from gyro data only when in flight.
     * @param accel Acceleration triplet (x, y, z) in m/s^2
     * @param gyro Gyroscope triplet (x, y, z) in degrees/s
     * @param mag Magnetometer triplet (x, y, z) in microteslas
     * @param currentTime Current timestamp in milliseconds
     */
    void update(AccelerationTriplet accel, GyroTriplet gyro, MagTriplet mag, uint32_t currentTime);

    Quaternion getQuaternion() const{
        Quaternion q = {q0, q1, q2, q3};
        return q;
    }
    void launchDetected() { hasLaunched = true;}
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    
private:
    float q0, q1, q2, q3;   // quaternion
    float betaPad;
    float betaFlight;
    float beta; // determines how much the algorithm relies on the accelerometer vs the gyroscope. Higher beta means more reliance on accel.
    float roll, pitch, yaw; // Euler angles in degrees
    bool hasLaunched;
    uint32_t lastUpdateTime;   // timestamp of the last update in milliseconds

    /**
     * @brief Update the orientation estimate using the full AHRS algorithm (gyro + accel + mag).
      * This function should never be called directly. Call update() instead, which will route to 
      * this or the IMU-only update as appropriate.
      * @param accel Acceleration triplet (x, y, z) in m/s^2
      * @param gyro Gyroscope triplet (x, y, z) in degrees/s
      * @param mag Magnetometer triplet (x, y, z) in microteslas
      * @param currentTime Current timestamp in milliseconds
     */
    void updateFullAHRS(AccelerationTriplet accel, GyroTriplet gyro, MagTriplet mag, uint32_t currentTime);

    /**
      * @brief Update the orientation estimator using only the IMU (gyro + optional accel). 
      * This is used when magnetometer data is invalid. Never call directly - call update() instead.
      * @param accel Acceleration triplet (x, y, z) in m/s^2
      * @param gyro Gyroscope triplet (x, y, z) in degrees/s
      * @param mag Magnetometer triplet (x, y, z) in microteslas
      * @param currentTime Current timestamp in milliseconds
     */
    void updateIMU(AccelerationTriplet accel, GyroTriplet gyro, uint32_t currentTime);

    /*
    * @brief Compute Euler angles (roll, pitch, yaw) from the current quaternion estimate.
    * This is called every update to keep the Euler angles in sync with the quaternion.
    * The Euler angles are in degrees and follow the aerospace convention (roll around x-axis, pitch around y-axis, yaw around z-axis).
    */
    void getEuler();
};


#endif