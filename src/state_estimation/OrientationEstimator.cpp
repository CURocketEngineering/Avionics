#include "state_estimation/OrientationEstimator.h"
#include <cmath>

void OrientationEstimator::update(AccelerationTriplet accel, GyroTriplet gyro, MagTriplet mag)
{
    // Extract raw sensor data from DataPoints
    // accleration
    float ax = accel.x.data;
    float ay = accel.y.data;
    float az = accel.z.data;

    // gyro
    float gx = gyro.x.data;
    float gy = gyro.y.data;
    float gz = gyro.z.data;

    // magnetometer
    float mx = mag.x.data;
    float my = mag.y.data;
    float mz = mag.z.data;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    // Convert gyro from deg/s to rad/s if needed
    gx *= M_PI / 180.0f;
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    // Normalize accelerometer
    recipNorm = 1.0f / std::sqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalize magnetometer
    recipNorm = 1.0f / std::sqrt(mx*mx + my*my + mz*mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables
    float _2q0mx = 2.0f * q0 * mx;
    float _2q0my = 2.0f * q0 * my;
    float _2q0mz = 2.0f * q0 * mz;
    float _2q1mx = 2.0f * q1 * mx;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float q0q0 = q0*q0;
    float q1q1 = q1*q1;
    float q2q2 = q2*q2;
    float q3q3 = q3*q3;

    // Reference direction of Earth's magnetic field
    float hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
    float hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
    float _2bx = std::sqrt(hx*hx + hy*hy);
    float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;

    // Gradient descent corrective step
    s0 = -_2q2*(2*(q1*q3 - q0*q2) - ax) +
         _2q1*(2*(q0*q1 + q2*q3) - ay) -
         _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1*q3 - q0*q2) - mx);

    s1 = _2q3*(2*(q1*q3 - q0*q2) - ax) +
         _2q0*(2*(q0*q1 + q2*q3) - ay) -
         4*q1*(1 - 2*(q1q1 + q2q2) - az);

    s2 = -_2q0*(2*(q1*q3 - q0*q2) - ax) +
         _2q3*(2*(q0*q1 + q2*q3) - ay) -
         4*q2*(1 - 2*(q1q1 + q2q2) - az);

    s3 = _2q1*(2*(q1*q3 - q0*q2) - ax) +
         _2q2*(2*(q0*q1 + q2*q3) - ay);

    recipNorm = 1.0f / std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Quaternion derivative
    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz) - beta * s0;
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy) - beta * s1;
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx) - beta * s2;
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx) - beta * s3;

    // Integrate
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalize quaternion
    recipNorm = 1.0f / std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
void OrientationEstimator::getEuler(float &roll, float &pitch, float &yaw)
{
    roll  = atan2(2*(q0*q1 + q2*q3),
                  1 - 2*(q1*q1 + q2*q2));

    pitch = asin(2*(q0*q2 - q3*q1));

    yaw   = atan2(2*(q0*q3 + q1*q2),
                  1 - 2*(q2*q2 + q3*q3));

    roll  *= 180.0f / M_PI;
    pitch *= 180.0f / M_PI;
    yaw   *= 180.0f / M_PI;
}