// NOLINTBEGIN(readability-identifier-length)
#include "state_estimation/OrientationEstimator.h"
#include <cmath>

constexpr float lowMagThreshold = 0.01F;
constexpr float highMagThreshold = 10.0F;
constexpr float kOne = 1.0F;
constexpr float kTwo = 2.0F;
constexpr float kFour = 4.0F;
constexpr float kEight = 8.0F;
constexpr float kRecipTwo = 0.5F;
constexpr float milliToSec = 1000.0F;


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

    const float dt = static_cast<float>(currentTime - lastUpdateTime) / milliToSec;

    // --- Sensor usage flags ---
    bool useAccel = false;
    bool useMag   = false;

	if (!hasLaunched){
		useAccel = true;
		useMag = true;
		beta = betaPad;
	}
	else{
		beta = betaFlight;
	}

    // --- Magnetometer validity check ---
    const auto magMag = static_cast<float>(std::sqrt(magX*magX + magY*magY + magZ*magZ));
    if (magMag < lowMagThreshold || magMag > highMagThreshold) {
        useMag = false;
    }

    // --- Route to appropriate update ---
    if (!useMag) {
        // IMU-only path (gyro + optional accel)
        if (useAccel) {
            updateIMU(accel, gyro, currentTime);
        } else {
            // PURE GYRO INTEGRATION (no correction)
            const float qDot1 = kRecipTwo * (-q1 * gyroX - q2 * gyroY - q3 * gyroZ);
            const float qDot2 = kRecipTwo * ( q0 * gyroX + q2 * gyroZ - q3 * gyroY);
            const float qDot3 = kRecipTwo * ( q0 * gyroY - q1 * gyroZ + q3 * gyroX);
            const float qDot4 = kRecipTwo * ( q0 * gyroZ + q1 * gyroY - q2 * gyroX);

            q0 += qDot1 * dt;
            q1 += qDot2 * dt;
            q2 += qDot3 * dt;
            q3 += qDot4 * dt;

            const float recipNorm = kOne / static_cast<float>(std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3));
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

	// gradient descent algorithm corrective step variables
	float s0;
	float s1;
	float s2;
	float s3;
	float qDot1;
	float qDot2;
	float qDot3;
	float qDot4;

	float hx;
	float hy;
	float _2q0magX, _2q0magY, _2q0magZ, _2q1magX, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; //NOLINT(readability-isolate-declaration)


    const float dt = static_cast<float>(currentTime - lastUpdateTime) / milliToSec; // convert ms to seconds

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((magX == 0.0F) && (magY == 0.0F) && (magZ == 0.0F)) {
		updateIMU(accel, gyro, currentTime);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = kRecipTwo * (-q1 * gyroX - q2 * gyroY - q3 * gyroZ);
	qDot2 = kRecipTwo * (q0 * gyroX + q2 * gyroZ - q3 * gyroY);
	qDot3 = kRecipTwo * (q0 * gyroY - q1 * gyroZ + q3 * gyroX);
	qDot4 = kRecipTwo * (q0 * gyroZ + q1 * gyroY - q2 * gyroX);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((accelX != 0.0F) || (accelY != 0.0F) || (accelZ != 0.0F)) {

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
		_2q0magX = kTwo * q0 * magX;
		_2q0magY = kTwo * q0 * magY;
		_2q0magZ = kTwo * q0 * magZ;
		_2q1magX = kTwo * q1 * magX;
		_2q0 = kTwo * q0;
		_2q1 = kTwo * q1;
		_2q2 = kTwo * q2;
		_2q3 = kTwo * q3;
		_2q0q2 = kTwo * q0 * q2;
		_2q2q3 = kTwo * q2 * q3;
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
		_2bx = static_cast<float>(std::sqrt(hx * hx + hy * hy));
		_2bz = -_2q0magX * q2 + _2q0magY * q1 + magZ * q0q0 + _2q1magX * q3 - magZ * q1q1 + _2q2 * magY * q3 - magZ * q2q2 + magZ * q3q3;
		_4bx = kTwo * _2bx;
		_4bz = kTwo * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (kTwo * q1q3 - _2q0q2 - accelX) + _2q1 * (kTwo * q0q1 + _2q2q3 - accelY) - _2bz * q2 * (_2bx * (kRecipTwo - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (kRecipTwo - q1q1 - q2q2) - magZ);
		s1 = _2q3 * (kTwo * q1q3 - _2q0q2 - accelX) + _2q0 * (kTwo * q0q1 + _2q2q3 - accelY) - kFour * q1 * (1 - kTwo * q1q1 - kTwo * q2q2 - accelZ) + _2bz * q3 * (_2bx * (kRecipTwo - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (kRecipTwo - q1q1 - q2q2) - magZ);
		s2 = -_2q0 * (kTwo * q1q3 - _2q0q2 - accelX) + _2q3 * (kTwo * q0q1 + _2q2q3 - accelY) - kFour * q2 * (1 - kTwo * q1q1 - kTwo * q2q2 - accelZ) + (-_4bx * q2 - _2bz * q0) * (_2bx * (kRecipTwo - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (kRecipTwo - q1q1 - q2q2) - magZ);
		s3 = _2q1 * (kTwo * q1q3 - _2q0q2 - accelX) + _2q2 * (kTwo * q0q1 + _2q2q3 - accelY) + (-_4bx * q3 + _2bz * q1) * (_2bx * (kRecipTwo - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - magX) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - magY) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (kRecipTwo - q1q1 - q2q2) - magZ);
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

	// gradient descent algorithm corrective step variables
	float s0;
	float s1;
	float s2;
	float s3;
	float qDot1;
	float qDot2;
	float qDot3;
	float qDot4;

	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3; //NOLINT(readability-isolate-declaration)

    const float dt = static_cast<float>(currentTime - lastUpdateTime) / milliToSec; // convert ms to seconds

	// Rate of change of quaternion from gyroscope
	qDot1 = kRecipTwo * (-q1 * gyroX - q2 * gyroY - q3 * gyroZ);
	qDot2 = kRecipTwo * (q0 * gyroX + q2 * gyroZ - q3 * gyroY);
	qDot3 = kRecipTwo * (q0 * gyroY - q1 * gyroZ + q3 * gyroX);
	qDot4 = kRecipTwo * (q0 * gyroZ + q1 * gyroY - q2 * gyroX);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((accelX != 0.0F) || (accelY != 0.0F) || (accelZ != 0.0F)) {

		// Normalise accelerometer measurement
		recipNorm = 1 / std::sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
		accelX *= recipNorm;
		accelY *= recipNorm;
		accelZ *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = kTwo * q0;
		_2q1 = kTwo * q1;
		_2q2 = kTwo * q2;
		_2q3 = kTwo * q3;
		_4q0 = kFour * q0;
		_4q1 = kFour * q1;
		_4q2 = kFour * q2;
		_8q1 = kEight * q1;
		_8q2 = kEight * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * accelX + _4q0 * q1q1 - _2q1 * accelY;
		s1 = _4q1 * q3q3 - _2q3 * accelX + kFour * q0q0 * q1 - _2q0 * accelY - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accelZ;
		s2 = kFour * q0q0 * q2 + _2q0 * accelX + _4q2 * q3q3 - _2q3 * accelY - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accelZ;
		s3 = kFour * q1q1 * q3 - _2q1 * accelX + kFour * q2q2 * q3 - _2q2 * accelY;
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
    roll = static_cast<float>(std::atan2(
        kTwo * (q0 * q1 + q2 * q3),
        kOne - kTwo * (q1 * q1 + q2 * q2)
    ));

    // Pitch (y-accelXis rotation)
    float t = kTwo * (q0 * q2 - q3 * q1);
    if (t > kOne){
		t = kOne;
	}
	if (t < -kOne){
		t = -kOne;
	}
    pitch = static_cast<float>(std::asin(t));

    // Yaw (z-accelXis rotation)
    yaw = static_cast<float>(std::atan2(
        kTwo * (q0 * q3 + q1 * q2),
        kOne - kTwo * (q2 * q2 + q3 * q3)
    ));

    const float rad2deg = 57.29577951308232F;
    roll *= rad2deg;
    pitch *= rad2deg;
    yaw *= rad2deg;
}
// NOLINTEND(readability-identifier-length)