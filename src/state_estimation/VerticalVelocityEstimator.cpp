#include <array>
#include <cmath>
#include <cstdio>

#include "state_estimation/VerticalVelocityEstimator.h"

constexpr float VerticalVelocityEstimator::kGravity_mps2;


VerticalVelocityEstimator::VerticalVelocityEstimator(NoiseVariances noise)
    : stateAltitude_m_(0.0F),
      stateVelocity_mps_(0.0F),
      lastTimestamp_ms_(0),
      initialized_(false),
      accelNoiseVariance_(noise.accelNoiseVar),
      altimeterNoiseVariance_(noise.altimeterNoiseVar),
      verticalAxis_(0),
      verticalDirection_(0),
      verticalAxisDetermined_(false),
      inertialVerticalAcceleration_(0.0F)
{
    // Initialize the covariance matrix P with moderate initial uncertainty.
    P_[0][0] = 1.0F;  P_[0][1] = 0.0F;
    P_[1][0] = 0.0F;  P_[1][1] = 1.0F;
}

void VerticalVelocityEstimator::init(InitialState initialState) {
    stateAltitude_m_ = initialState.initialAltitude;
    stateVelocity_mps_ = 0.0F;
    lastTimestamp_ms_ = initialState.initialTimestamp;
    initialized_ = true;

    // Reset vertical axis determination.
    verticalAxisDetermined_ = false;
    verticalAxis_ = 0;
    verticalDirection_ = 0;

    // Reset covariance matrix.
    P_[0][0] = 1.0F;  P_[0][1] = 0.0F;
    P_[1][0] = 0.0F;  P_[1][1] = 1.0F;
}

void VerticalVelocityEstimator::determineVerticalAxis(const std::array<float, 3>& rawAcl) {
    // Check the magnitude of each axis reading.
    std::array<float, 3> mag = { std::fabs(rawAcl[0]), std::fabs(rawAcl[1]), std::fabs(rawAcl[2]) };

    // Find the index of the largest magnitude.
    const std::size_t yAxis = 1U;
    const std::size_t zAxis = 2U;
    verticalAxis_ = 0; // Start with X
    auto verticalAxisIndex = static_cast<std::size_t>(static_cast<uint8_t>(verticalAxis_));
    if (mag[yAxis] > mag[verticalAxisIndex]) {
        verticalAxis_ = 1; // Y
        verticalAxisIndex = yAxis;
    }
    if (mag[zAxis] > mag[verticalAxisIndex]) {
        verticalAxis_ = 2; // Z
        verticalAxisIndex = zAxis;
    }

    // Determine if it's positive or negative relative to 'up'.
    verticalDirection_ = (rawAcl[verticalAxisIndex] > 0.0F) ? 1 : -1;
}

// NOLINTBEGIN(readability-identifier-length)
void VerticalVelocityEstimator::update(const AccelerationTriplet &accel, const DataPoint &altitude) 
{
    // Use the altimeter timestamp as the reference for this update.
    const uint32_t currentTimestamp_ms = altitude.timestamp_ms;

    // If not initialized, do so with the altimeter reading.
    if (!initialized_) {
        const InitialState initialState = { altitude.data, currentTimestamp_ms };
        init(initialState);
        return;
    }

    // Determine which axis is vertical if not done yet.
    std::array<float, 3> rawAcl = { accel.x.data, accel.y.data, accel.z.data};
    if (!verticalAxisDetermined_) {
        determineVerticalAxis(rawAcl);
        verticalAxisDetermined_ = true;
    }





    // Ensures the data is newer than the previous data and that is not the same as the last data
    if (currentTimestamp_ms <= lastTimestamp_ms_)
    {
        return;
    }
    if(accel.x.timestamp_ms <= lastTimestamp_ms_)
    {
        return;
    }
    
    // Compute time step in seconds (dt).
    const float dt = (static_cast<float>(currentTimestamp_ms - lastTimestamp_ms_)) * kMillisecondsToSeconds;

    // Subtract gravity from the measured acceleration on the identified vertical axis.
    const auto verticalAxisIndex = static_cast<std::size_t>(static_cast<uint8_t>(verticalAxis_));
    inertialVerticalAcceleration_ =
        (rawAcl[verticalAxisIndex] * static_cast<float>(verticalDirection_)) - kGravity_mps2;

    // --- Prediction Step ---
    // State prediction:
    //     predictedAltitude_m = alt + vel * dt + 0.5 * a * dt^2
    //     predictedVelocity_mps = vel + a * dt
    const float predictedAltitude_m = stateAltitude_m_ + stateVelocity_mps_ * dt + 0.5F * inertialVerticalAcceleration_ * dt * dt;
    const float predictedVelocity_mps = stateVelocity_mps_ + inertialVerticalAcceleration_ * dt;

    // Process noise covariance Q (derived from acceleration noise variance).
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    const float q00 = 0.25F * dt4 * accelNoiseVariance_; // var in position
    const float q01 = 0.5F  * dt3 * accelNoiseVariance_; // covar pos-vel
    const float q10 = 0.5F  * dt3 * accelNoiseVariance_; // covar vel-pos
    const float q11 =        dt2 * accelNoiseVariance_;  // var in velocity

    // Predicted covariance: P' = F P F^T + Q
    // F = [ [1, dt], [0, 1] ]
    // so:
    // P'[0][0] = P_[0][0] + 2*dt*P_[0][1] + dt^2*P_[1][1] + q00
    // P'[0][1] = P_[0][1] + dt*P_[1][1] + q01
    // P'[1][0] = P_[1][0] + dt*P_[1][1] + q10  (should be symmetric to P'[0][1])
    // P'[1][1] = P_[1][1] + q11

    // 2.0F comes from the derivative of the position state equation
    const float predictedCov00 = P_[0][0] + 2.0F * dt * P_[0][1] + dt2 * P_[1][1] + q00; //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    const float predictedCov01 = P_[0][1] + dt * P_[1][1] + q01;
    const float predictedCov10 = P_[1][0] + dt * P_[1][1] + q10;  // note: P_[1][0] == P_[0][1] if always kept symmetric
    const float predictedCov11 = P_[1][1] + q11;

    // --- Measurement Update (using altimeter reading) ---
    // Measurement z = altimeter altitude
    const float z = altitude.data;
    // Innovation (residual): y = z - predictedAltitude_m
    const float y = z - predictedAltitude_m;
    // Innovation covariance: S = H P' H^T + R
    // H = [1, 0], so S = P'[0][0] + altimeter noise variance.
    const float innovationCovariance = predictedCov00 + altimeterNoiseVariance_;

    // Kalman gain: K = P' H^T / S = [ P'[0][0], P'[1][0] ] / S
    const float kalmanGain0 = predictedCov00 / innovationCovariance; 
    const float kalmanGain1 = predictedCov10 / innovationCovariance; 

    // Update state with measurement
    stateAltitude_m_ = predictedAltitude_m + kalmanGain0 * y;
    stateVelocity_mps_ = predictedVelocity_mps + kalmanGain1 * y;

    // Update covariance: P = (I - K H) P'
    // (I - K H) = [ [1 - K0, 0], [-K1, 1] ]
    // So:
    // P_[0][0] = (1 - K0)*P00
    // P_[0][1] = (1 - K0)*P01
    // P_[1][0] = P10 - K1*P00
    // P_[1][1] = P11 - K1*P01
    P_[0][0] = (1.0F - kalmanGain0) * predictedCov00;
    P_[0][1] = (1.0F - kalmanGain0) * predictedCov01;
    P_[1][0] = predictedCov10 - kalmanGain1 * predictedCov00;
    P_[1][1] = predictedCov11 - kalmanGain1 * predictedCov01;

    // Update time
    lastTimestamp_ms_ = currentTimestamp_ms;
}
// NOLINTEND(readability-identifier-length)

float VerticalVelocityEstimator::getEstimatedAltitude() const {
    return stateAltitude_m_;
}

float VerticalVelocityEstimator::getEstimatedVelocity() const {
    return stateVelocity_mps_;
}

float VerticalVelocityEstimator::getInertialVerticalAcceleration() const {
    return inertialVerticalAcceleration_;
}

int8_t VerticalVelocityEstimator::getVerticalAxis() const {
    return verticalAxis_;
}

int8_t VerticalVelocityEstimator::getVerticalDirection() const {
    return verticalDirection_;
}

uint32_t VerticalVelocityEstimator::getTimestamp() const {
    return lastTimestamp_ms_;
}
