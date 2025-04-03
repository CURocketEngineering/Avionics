#include "state_estimation/VelocityEstimator.h"
#include <cmath>

VerticalVelocityEstimator::VerticalVelocityEstimator(float accelNoiseVar, float altimeterNoiseVar)
    : state_alt(0.0f),
      state_vel(0.0f),
      lastTimestamp(0),
      initialized(false),
      accelNoiseVariance(accelNoiseVar),
      altimeterNoiseVariance(altimeterNoiseVar),
      verticalAxis(0),
      verticalDirection(0),
      verticalAxisDetermined(false),
      inertialVerticalAcceleration(0.0f)
{
    // Initialize the covariance matrix P with moderate initial uncertainty.
    P[0][0] = 1.0f;  P[0][1] = 0.0f;
    P[1][0] = 0.0f;  P[1][1] = 1.0f;
}

void VerticalVelocityEstimator::init(float initialAltitude, uint32_t initialTimestamp) {
    state_alt = initialAltitude;
    state_vel = 0.0f;
    lastTimestamp = initialTimestamp;
    initialized = true;

    // Reset vertical axis determination.
    verticalAxisDetermined = false;
    verticalAxis = 0;
    verticalDirection = 0;

    // Reset covariance matrix.
    P[0][0] = 1.0f;  P[0][1] = 0.0f;
    P[1][0] = 0.0f;  P[1][1] = 1.0f;
}

void VerticalVelocityEstimator::determineVerticalAxis(const float rawAcl[3]) {
    // Check the magnitude of each axis reading.
    float mag[3] = { std::fabs(rawAcl[0]), std::fabs(rawAcl[1]), std::fabs(rawAcl[2]) };

    // Find the index of the largest magnitude.
    verticalAxis = 0; // Start with X
    if (mag[1] > mag[verticalAxis]) {
        verticalAxis = 1; // Y
    }
    if (mag[2] > mag[verticalAxis]) {
        verticalAxis = 2; // Z
    }

    // Determine if it's positive or negative relative to 'up'.
    verticalDirection = (rawAcl[verticalAxis] > 0.0f) ? 1 : -1;
}

void VerticalVelocityEstimator::update(const DataPoint &accelX, const DataPoint &accelY, 
                               const DataPoint &accelZ, const DataPoint &altimeter) 
{
    // Use the altimeter timestamp as the reference for this update.
    uint32_t currentTimestamp = altimeter.timestamp_ms;

    // If not initialized, do so with the altimeter reading.
    if (!initialized) {
        init(altimeter.data, currentTimestamp);
        return;
    }

    // Determine which axis is vertical if not done yet.
    float rawAcl[3] = { accelX.data, accelY.data, accelZ.data };
    if (!verticalAxisDetermined) {
        determineVerticalAxis(rawAcl);
        verticalAxisDetermined = true;
    }

    // Compute time step in seconds (dt). Use a small default if timestamps are identical.
    float dt = (currentTimestamp > lastTimestamp)
               ? (currentTimestamp - lastTimestamp) / 1000.0f
               : 0.01f;

    // Subtract gravity from the measured acceleration on the identified vertical axis.
    inertialVerticalAcceleration = (rawAcl[verticalAxis] * verticalDirection) - g;

    // --- Prediction Step ---
    // State prediction:
    //     predicted_alt = alt + vel * dt + 0.5 * a * dt^2
    //     predicted_vel = vel + a * dt
    float predicted_alt = state_alt + state_vel * dt + 0.5f * inertialVerticalAcceleration * dt * dt;
    float predicted_vel = state_vel + inertialVerticalAcceleration * dt;

    // Process noise covariance Q (derived from accelNoiseVariance).
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    float Q00 = 0.25f * dt4 * accelNoiseVariance; // var in position
    float Q01 = 0.5f  * dt3 * accelNoiseVariance; // covar pos-vel
    float Q10 = 0.5f  * dt3 * accelNoiseVariance; // covar vel-pos
    float Q11 =        dt2 * accelNoiseVariance;  // var in velocity

    // Predicted covariance: P' = F P F^T + Q
    // F = [ [1, dt], [0, 1] ]
    // so:
    // P'[0][0] = P[0][0] + 2*dt*P[0][1] + dt^2*P[1][1] + Q00
    // P'[0][1] = P[0][1] + dt*P[1][1] + Q01
    // P'[1][0] = P[1][0] + dt*P[1][1] + Q10  (should be symmetric to P'[0][1])
    // P'[1][1] = P[1][1] + Q11
    float P00 = P[0][0] + 2.0f * dt * P[0][1] + dt2 * P[1][1] + Q00;
    float P01 = P[0][1] + dt * P[1][1] + Q01;
    float P10 = P[1][0] + dt * P[1][1] + Q10;  // note: P[1][0] == P[0][1] if always kept symmetric
    float P11 = P[1][1] + Q11;

    // --- Measurement Update (using altimeter reading) ---
    // Measurement z = altimeter altitude
    float z = altimeter.data;
    // Innovation (residual): y = z - predicted_alt
    float y = z - predicted_alt;
    // Innovation covariance: S = H P' H^T + R
    // H = [1, 0], so S = P'[0][0] + altimeterNoiseVariance
    float S = P00 + altimeterNoiseVariance;

    // Kalman gain: K = P' H^T / S = [ P'[0][0], P'[1][0] ] / S
    float K0 = P00 / S;
    float K1 = P10 / S;

    // Update state with measurement
    state_alt = predicted_alt + K0 * y;
    state_vel = predicted_vel + K1 * y;

    // Update covariance: P = (I - K H) P'
    // (I - K H) = [ [1 - K0, 0], [-K1, 1] ]
    // So:
    // P[0][0] = (1 - K0)*P00
    // P[0][1] = (1 - K0)*P01
    // P[1][0] = P10 - K1*P00
    // P[1][1] = P11 - K1*P01
    P[0][0] = (1.0f - K0) * P00;
    P[0][1] = (1.0f - K0) * P01;
    P[1][0] = P10 - K1 * P00;
    P[1][1] = P11 - K1 * P01;

    // Update time
    lastTimestamp = currentTimestamp;
}

float VerticalVelocityEstimator::getEstimatedAltitude() const {
    return state_alt;
}

float VerticalVelocityEstimator::getEstimatedVelocity() const {
    return state_vel;
}

float VerticalVelocityEstimator::getInertialVerticalAcceleration() const {
    return inertialVerticalAcceleration;
}

int8_t VerticalVelocityEstimator::getVerticalAxis() const {
    return verticalAxis;
}

int8_t VerticalVelocityEstimator::getVerticalDirection() const {
    return verticalDirection;
}
