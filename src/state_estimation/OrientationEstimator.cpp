#include "state_estimation/OrientationEstimator.h"
#include <cmath>

#define M_PI 3.14159265358979323846

void OrientationEstimator::update(AccelerationTriplet accel,
                                  GyroTriplet gyro,
                                  MagTriplet mag,
                                  uint32_t currentTime)
{
    const float gyroX = gyro.x.data;
    const float gyroY = gyro.y.data;
    const float gyroZ = gyro.z.data;

    const float magX = mag.x.data;
    const float magY = mag.y.data;
    const float magZ = mag.z.data;

    const float dt = (float) (currentTime - lastUpdateTime) / 1000.0f;

    // --- Sensor usage flags ---
    bool useAccel = false;
    bool useMag   = false;

    if (hasLaunched == false){
		useAccel = true;
		useMag = true;
		beta = betaPad;
	}
	else{
		beta = betaFlight;
	}

    // --- Magnetometer validity check ---
    const float magMag = (float) sqrt(magX*magX + magY*magY + magZ*magZ);
    if (magMag < 0.01f || magMag > 10.0f) {
        useMag = false;
    }

    // --- Route to appropriate update ---
    if (!useMag) {
        // IMU-only path (gyro + optional accel)
        if (useAccel) {
            updateIMU(accel, gyro, currentTime);
        } else {
            // PURE GYRO INTEGRATION (no correction)
            float qDot1 = 0.5f * (-q1 * gyroX - q2 * gyroY - q3 * gyroZ);
            float qDot2 = 0.5f * ( q0 * gyroX + q2 * gyroZ - q3 * gyroY);
            float qDot3 = 0.5f * ( q0 * gyroY - q1 * gyroZ + q3 * gyroX);
            float qDot4 = 0.5f * ( q0 * gyroZ + q1 * gyroY - q2 * gyroX);

            q0 += qDot1 * dt;
            q1 += qDot2 * dt;
            q2 += qDot3 * dt;
            q3 += qDot4 * dt;

            float recipNorm = 1.0f / (float) sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q0 *= recipNorm;
            q1 *= recipNorm;
            q2 *= recipNorm;
            q3 *= recipNorm;

            getEuler();
            lastUpdateTime = currentTime;
        }
        return;
    }

    // used when on pad
    updateFullAHRS(accel, gyro, mag, currentTime);
}

void OrientationEstimator::updateFullAHRS(AccelerationTriplet accel, GyroTriplet gyro, MagTriplet mag, uint32_t currentTime){
	float accelX = accel.x.data;
    float accelY = accel.y.data;
    float accelZ = accel.z.data;

	const float gyroX = gyro.x.data;
	const float gyroY = gyro.y.data;
	const float gyroZ = gyro.z.data;

	float magX = mag.x.data;
	float magY = mag.y.data;
	float magZ = mag.z.data;
	
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0magX, _2q0magY, _2q0magZ, _2q1magX, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;


    const float dt = (float) (currentTime - lastUpdateTime) / 1000.0f; // convert ms to seconds

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((magX == 0.0f) && (magY == 0.0f) && (magZ == 0.0f)) {
		updateIMU(accel, gyro, currentTime);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gyroX - q2 * gyroY - q3 * gyroZ);
	qDot2 = 0.5f * (q0 * gyroX + q2 * gyroZ - q3 * gyroY);
	qDot3 = 0.5f * (q0 * gyroY - q1 * gyroZ + q3 * gyroX);
	qDot4 = 0.5f * (q0 * gyroZ + q1 * gyroY - q2 * gyroX);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((accelX == 0.0f) && (accelY == 0.0f) && (accelZ == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1 / std::sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
		accelX *= recipNorm;
		accelY *= recipNorm;
		accelZ *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = 1 /std::sqrt(magX * magX + magY * magY + magZ * magZ);
		magX *= recipNorm;
		magY *= recipNorm;
		magZ *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0magX = 2.0f * q0 * magX;
		_2q0magY = 2.0f * q0 * magY;
		_2q0magZ = 2.0f * q0 * magZ;
		_2q1magX = 2.0f * q1 * magX;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = magX * q0q0 - _2q0magY * q3 + _2q0magZ * q2 + magX * q1q1 + _2q1 * magY * q2 + _2q1 * magZ * q3 - magX * q2q2 - magX * q3q3;
		hy = _2q0magX * q3 + magY * q0q0 - _2q0magZ * q1 + _2q1magX * q2 - magY * q1q1 + magY * q2q2 + _2q2 * magZ * q3 - magY * q3q3;
		_2bx = (float) sqrt(hx * hx + hy * hy);
		_2bz = -_2q0magX * q2 + _2q0magY * q1 + magZ * q0q0 + _2q1magX * q3 - magZ * q1q1 + _2q2 * magY * q3 - magZ * q2q2 + magZ * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - accelX) + _2q1 * (2.0f * q0q1 + _2q2q3 - accelY) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magZ);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - accelX) + _2q0 * (2.0f * q0q1 + _2q2q3 - accelY) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - accelZ) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magZ);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - accelX) + _2q3 * (2.0f * q0q1 + _2q2q3 - accelY) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - accelZ) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magZ);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - accelX) + _2q2 * (2.0f * q0q1 + _2q2q3 - accelY) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - magZ);
		recipNorm = 1 / std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = 1 / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	getEuler();
	lastUpdateTime = currentTime;
}

void OrientationEstimator::updateIMU(AccelerationTriplet accel, GyroTriplet gyro, uint32_t currentTime){
	float accelX = accel.x.data;
	float accelY = accel.y.data;
	float accelZ = accel.z.data;

	const float gyroX = gyro.x.data;
	const float gyroY = gyro.y.data;
	const float gyroZ = gyro.z.data;

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    float dt = (float) (currentTime - lastUpdateTime) / 1000.0f; // convert ms to seconds

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gyroX - q2 * gyroY - q3 * gyroZ);
	qDot2 = 0.5f * (q0 * gyroX + q2 * gyroZ - q3 * gyroY);
	qDot3 = 0.5f * (q0 * gyroY - q1 * gyroZ + q3 * gyroX);
	qDot4 = 0.5f * (q0 * gyroZ + q1 * gyroY - q2 * gyroX);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((accelX == 0.0f) && (accelY == 0.0f) && (accelZ == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1 / std::sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
		accelX *= recipNorm;
		accelY *= recipNorm;
		accelZ *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * accelX + _4q0 * q1q1 - _2q1 * accelY;
		s1 = _4q1 * q3q3 - _2q3 * accelX + 4.0f * q0q0 * q1 - _2q0 * accelY - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accelZ;
		s2 = 4.0f * q0q0 * q2 + _2q0 * accelX + _4q2 * q3q3 - _2q3 * accelY - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accelZ;
		s3 = 4.0f * q1q1 * q3 - _2q1 * accelX + 4.0f * q2q2 * q3 - _2q2 * accelY;
		recipNorm = 1 / std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = 1 / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	getEuler();
	lastUpdateTime = currentTime;
}

void OrientationEstimator::getEuler()
{
    // Roll (x-accelXis rotation)
    roll = (float) atan2(
        2.0f * (q0 * q1 + q2 * q3),
        1.0f - 2.0f * (q1 * q1 + q2 * q2)
    );

    // Pitch (y-accelXis rotation)
    float t = 2.0f * (q0 * q2 - q3 * q1);
    if (t > 1.0f) t = 1.0f;
	if (t < -1.0f) t = -1.0f;
    pitch = (float) asin(t);

    // Yaw (z-accelXis rotation)
    yaw = (float) atan2(
        2.0f * (q0 * q3 + q1 * q2),
        1.0f - 2.0f * (q2 * q2 + q3 * q3)
    );

    const float rad2deg = 57.29577951308232f;
    roll *= rad2deg;
    pitch *= rad2deg;
    yaw *= rad2deg;
}