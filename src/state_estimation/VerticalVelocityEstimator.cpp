#include <array>
#include <cmath>
#include <cstdio>

#include "state_estimation/VerticalVelocityEstimator.h"


VerticalVelocityEstimator::VerticalVelocityEstimator(NoiseVariances noise)
    : state_alt(0.0F),
      state_vel(0.0F),
      lastTimestamp_ms(0),
      initialized(false),
      accelNoiseVariance(noise.accelNoiseVar),
      altimeterNoiseVariance(noise.altimeterNoiseVar),
      verticalAxis(0),
      verticalDirection(0),
      verticalAxisDetermined(false),
      inertialVerticalAcceleration(0.0F)
{
    // Initialize the covariance matrix P with moderate initial uncertainty.
    P[0][0] = 1.0F;  P[0][1] = 0.0F;
    P[1][0] = 0.0F;  P[1][1] = 1.0F;
}

void VerticalVelocityEstimator::init(InitialState initialState) {
    state_alt = initialState.initialAltitude;
    state_vel = 0.0F;
    lastTimestamp_ms = initialState.initialTimestamp;
    initialized = true;

    // Reset vertical axis determination.
    verticalAxisDetermined = false;
    verticalAxis = 0;
    verticalDirection = 0;

    // Reset covariance matrix.
    P[0][0] = 1.0F;  P[0][1] = 0.0F;
    P[1][0] = 0.0F;  P[1][1] = 1.0F;
}

void VerticalVelocityEstimator::determineVerticalAxis(const std::array<float, 3>& rawAcl) {
    // Check the magnitude of each axis reading.
    std::array<float, 3> mag = { std::fabs(rawAcl[0]), std::fabs(rawAcl[1]), std::fabs(rawAcl[2]) };

    // Find the index of the largest magnitude.
    verticalAxis = 0; // Start with X
    if (mag[1] > mag[verticalAxis]) {
        verticalAxis = 1; // Y
    }
    if (mag[2] > mag[verticalAxis]) {
        verticalAxis = 2; // Z
    }

    // Determine if it's positive or negative relative to 'up'.
    verticalDirection = (rawAcl[verticalAxis] > 0.0F) ? 1 : -1;
}

// NOLINTBEGIN(readability-identifier-length)
void VerticalVelocityEstimator::update(const AccelerationTriplet &accel, const DataPoint &altimeter) 
{
    // Use the altimeter timestamp as the reference for this update.
    const uint32_t currentTimestamp_ms = altimeter.timestamp_ms;

    // If not initialized, do so with the altimeter reading.
    if (!initialized) {
        const InitialState initialState = { altimeter.data, currentTimestamp_ms };
        init(initialState);
        return;
    }

    // Determine which axis is vertical if not done yet.
    std::array<float, 3> rawAcl = { accel.x.data, accel.y.data, accel.z.data};
    if (!verticalAxisDetermined) {
        determineVerticalAxis(rawAcl);
        verticalAxisDetermined = true;
    }

    // Compute time step in seconds (dt). Use a small default if timestamps are identical.
    const float dt = (currentTimestamp_ms > lastTimestamp_ms)
               ? (static_cast<float>(currentTimestamp_ms - lastTimestamp_ms)) * MILLISECONDS_TO_SECONDS
               : MINIMUM_DELTA_T_S;

    // Subtract gravity from the measured acceleration on the identified vertical axis.
    inertialVerticalAcceleration = (rawAcl[verticalAxis] * static_cast<float>(verticalDirection)) - g;

    // --- Prediction Step ---
    // State prediction:
    //     predicted_alt = alt + vel * dt + 0.5 * a * dt^2
    //     predicted_vel = vel + a * dt
    const float predicted_alt = state_alt + state_vel * dt + 0.5F * inertialVerticalAcceleration * dt * dt;
    const float predicted_vel = state_vel + inertialVerticalAcceleration * dt;

    // Process noise covariance Q (derived from accelNoiseVariance).
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    const float Q00 = 0.25F * dt4 * accelNoiseVariance; // var in position
    const float Q01 = 0.5F  * dt3 * accelNoiseVariance; // covar pos-vel
    const float Q10 = 0.5F  * dt3 * accelNoiseVariance; // covar vel-pos
    const float Q11 =        dt2 * accelNoiseVariance;  // var in velocity

    // Predicted covariance: P' = F P F^T + Q
    // F = [ [1, dt], [0, 1] ]
    // so:
    // P'[0][0] = P[0][0] + 2*dt*P[0][1] + dt^2*P[1][1] + Q00
    // P'[0][1] = P[0][1] + dt*P[1][1] + Q01
    // P'[1][0] = P[1][0] + dt*P[1][1] + Q10  (should be symmetric to P'[0][1])
    // P'[1][1] = P[1][1] + Q11

    // 2.0F comes from the derivative of the position state equation
    const float P00 = P[0][0] + 2.0F * dt * P[0][1] + dt2 * P[1][1] + Q00; //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    const float P01 = P[0][1] + dt * P[1][1] + Q01;
    const float P10 = P[1][0] + dt * P[1][1] + Q10;  // note: P[1][0] == P[0][1] if always kept symmetric
    const float P11 = P[1][1] + Q11;

    // --- Measurement Update (using altimeter reading) ---
    // Measurement z = altimeter altitude
    const float z = altimeter.data;
    // Innovation (residual): y = z - predicted_alt
    const float y = z - predicted_alt;
    // Innovation covariance: S = H P' H^T + R
    // H = [1, 0], so S = P'[0][0] + altimeterNoiseVariance
    const float S = P00 + altimeterNoiseVariance;

    // Kalman gain: K = P' H^T / S = [ P'[0][0], P'[1][0] ] / S
    const float K0 = P00 / S; 
    const float K1 = P10 / S; 

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
    P[0][0] = (1.0F - K0) * P00;
    P[0][1] = (1.0F - K0) * P01;
    P[1][0] = P10 - K1 * P00;
    P[1][1] = P11 - K1 * P01;

    // Update time
    lastTimestamp_ms = currentTimestamp_ms;
}
// NOLINTEND(readability-identifier-length)

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

uint32_t VerticalVelocityEstimator::getTimestamp() const {
    return lastTimestamp_ms;
}
