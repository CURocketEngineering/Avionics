#include "state_estimation/ApogeePredictor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

// Clamp utility
template <typename T>
constexpr const T& clamp(const T& value, const T& lower, const T& upper) {
    return (value < lower) ? lower : (value > upper) ? upper : value;
}

// Constants
constexpr float MIN_VALID_DECEL = 0.000001F;
constexpr float MS_TO_S_FACTOR = 1000.0F;
constexpr float MAGIC_HALF = 0.5F;
constexpr float GRAVITY_MS2 = 9.80665F;
constexpr uint32_t MAX_WARMUPS = 1000;

}  // namespace

ApogeePredictor::ApogeePredictor(const VerticalVelocityEstimator& velocityEstimator, 
                                 float accelFilterAlpha,  // caution: two floats here are swappable //NOLINT(bugprone-easily-swappable-parameters)
                                 float minimumClimbVelocity_ms) //NOLINT(bugprone-easily-swappable-parameters)
    : vve_(velocityEstimator),
      filteredDecel_(0.0F),
      alpha_(clamp(accelFilterAlpha, 0.0F, 1.0F)),
      minClimbVel_(minimumClimbVelocity_ms),
      valid_(false),
      tToApogee_(0.0F),
      predApogeeTs_(0),
      predApogeeAlt_(0.0F),
      lastTs_(0),
      lastVel_(0.0F),
      numWarmups_(0) {}

void ApogeePredictor::update() {
    const uint32_t currentTimestamp = vve_.getTimestamp();
    const float velocity = vve_.getEstimatedVelocity();
    const float acceleration = vve_.getInertialVerticalAcceleration();

    // Raw deceleration estimate
    float decelSample = std::max(0.0F, -acceleration);

    if (currentTimestamp > lastTs_) {
        const auto deltaTime = static_cast<float>(currentTimestamp - lastTs_);
        const float deltaVelocity = velocity - lastVel_;
        float estimatedDecel = (deltaTime > 0.0F) ? std::max(0.0F, -deltaVelocity / deltaTime) : 0.0F;
        decelSample = MAGIC_HALF * (decelSample + estimatedDecel);
    }

    filteredDecel_ = alpha_ * decelSample + (1.0F - alpha_) * filteredDecel_;

    if (velocity > minClimbVel_ && filteredDecel_ > MIN_VALID_DECEL) {
        tToApogee_ = velocity / filteredDecel_;
        predApogeeTs_ = currentTimestamp + static_cast<uint32_t>(tToApogee_ * MS_TO_S_FACTOR);

        const float altitude = vve_.getEstimatedAltitude();
        predApogeeAlt_ = altitude + velocity * tToApogee_ -
                         MAGIC_HALF * filteredDecel_ * tToApogee_ * tToApogee_;

        valid_ = true;
    } else {
        valid_ = false;
    }

    lastTs_ = currentTimestamp;
    lastVel_ = velocity;
    numWarmups_ = std::min(numWarmups_ + 1, MAX_WARMUPS);
}

void ApogeePredictor::quad_update() {
    const uint32_t currentTimestamp = vve_.getTimestamp();
    const float velocity = vve_.getEstimatedVelocity();
    const float acceleration = vve_.getInertialVerticalAcceleration();

    float kEstimate = 0.0F;
    if (std::fabs(velocity) > 1.0F) {
        kEstimate = std::max(0.0F, -(acceleration + GRAVITY_MS2)) / (velocity * velocity);
    }

    filteredDecel_ = alpha_ * kEstimate + (1.0F - alpha_) * filteredDecel_;
    const float dragToMassRatio = filteredDecel_;

    if (velocity > minClimbVel_ && dragToMassRatio > MIN_VALID_DECEL) {
        const float terminalVelocity = std::sqrt(GRAVITY_MS2 / dragToMassRatio);
        tToApogee_ = (terminalVelocity / GRAVITY_MS2) * std::atan(velocity / terminalVelocity);
        const float deltaAltitude = (terminalVelocity * terminalVelocity / (2.0F * GRAVITY_MS2)) *
                                    std::log1p((velocity * velocity) /
                                               (terminalVelocity * terminalVelocity));

        predApogeeTs_ = currentTimestamp + static_cast<uint32_t>(tToApogee_ * MS_TO_S_FACTOR);
        predApogeeAlt_ = vve_.getEstimatedAltitude() + deltaAltitude;
        valid_ = true;
    } else {
        valid_ = false;
    }

    lastTs_ = currentTimestamp;
    lastVel_ = velocity;
}
void ApogeePredictor::poly_update() {
    const uint32_t currentTimestamp_ms = vve_.getTimestamp();
    const float altitude_m = vve_.getEstimatedAltitude();
    const float velocity_ms = vve_.getEstimatedVelocity();
    const float acceleration_ms2 = vve_.getInertialVerticalAcceleration();

    

    const size_t featureCount = 10;

    // Polynomial Regression Coefficients for C++
    const float coeffs[] = {
        /* 1 */ 0.00000000,
        /* vertical_velocity */ 5.06108448,
        /* vertical_acceleration */ 63.94744144,
        /* delta_h_simple */ 0.52115350,
        /* vertical_velocity^2 */ 0.01494354,
        /* vertical_velocity vertical_acceleration */ 0.46012269,
        /* vertical_velocity delta_h_simple */ 0.01274390,
        /* vertical_acceleration^2 */ 3.27864634,
        /* vertical_acceleration delta_h_simple */ -0.00747177,
        /* delta_h_simple^2 */ -0.00208120,
    };
    const float intercept = 308.64734694;

    // ───────────────────────────────────────────────────────
    // Compute delta_h_simple = v^2 / (2 * decel), with decel > 0
    double decel = std::fabs(acceleration_ms2);
    double delta_h_simple = (velocity_ms * velocity_ms) / (2.0 * decel);

    // ───────────────────────────────────────────────────────
    // Evaluate the regression model
    const double inputs[featureCount] = {
        1.0,
        velocity_ms,                // vertical_velocity
        acceleration_ms2,           // vertical_acceleration
        delta_h_simple,
        velocity_ms * velocity_ms,  // vertical_velocity^2
        velocity_ms * acceleration_ms2, // vertical_velocity vertical_acceleration
        velocity_ms * delta_h_simple, // vertical_velocity delta_h_simple
        acceleration_ms2 * acceleration_ms2, // vertical_acceleration^2
        acceleration_ms2 * delta_h_simple, // vertical_acceleration delta_h_simple
        delta_h_simple * delta_h_simple, // delta_h_simple^2
    };

    double apogeeRemaining_m = intercept;
    for (size_t i = 0; i < featureCount; ++i) {
        apogeeRemaining_m += coeffs[i] * inputs[i];
    }

    // ───────────────────────────────────────────────────────
    // Combine with current altitude to compute predicted apogee
    predApogeeAlt_ = altitude_m + apogeeRemaining_m;

    // Estimate time to apogee using kinematic model
    tToApogee_ = velocity_ms / decel;
    predApogeeTs_ = currentTimestamp_ms + static_cast<uint32_t>(tToApogee_ * MS_TO_S_FACTOR);

    valid_ = true;

    lastTs_ = currentTimestamp_ms;
    lastVel_ = velocity_ms;
    numWarmups_ = std::min(numWarmups_ + 1, MAX_WARMUPS);

    printf("Current Timestamp: %u, Altitude: %.2f, Velocity: %.2f, Acceleration: %.2f, Predicted Apogee Remaining: %.2f, Delta H: %.2f\n",
           currentTimestamp_ms, altitude_m, velocity_ms, acceleration_ms2, apogeeRemaining_m, delta_h_simple);
}



// Simple getters
bool ApogeePredictor::isPredictionValid() const { return valid_; }

float ApogeePredictor::getTimeToApogee_s() const {
    return valid_ ? tToApogee_ : 0.0F;
}

uint32_t ApogeePredictor::getPredictedApogeeTimestamp_ms() const {
    return valid_ ? predApogeeTs_ : 0;
}

float ApogeePredictor::getPredictedApogeeAltitude_m() const {
    return valid_ ? predApogeeAlt_ : 0.0F;
}

float ApogeePredictor::getFilteredDeceleration() const {
    return filteredDecel_;
}
