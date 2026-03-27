#include "state_estimation/ApogeePredictor.h"

#include <algorithm>
#include <array>
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
constexpr float kMinValidDecel = 0.000001F;
constexpr float kMillisecondsPerSecond = 1000.0F;
constexpr float kOneHalf = 0.5F;
constexpr float kGravity_mps2 = 9.80665F;
constexpr uint32_t kMaxWarmups = 1000;

}  // namespace

ApogeePredictor::ApogeePredictor(const VerticalVelocityEstimator& velocityEstimator, 
                                 float accelFilterAlpha,  // caution: two floats here are swappable //NOLINT(bugprone-easily-swappable-parameters)
                                 float minimumClimbVelocity_mps) //NOLINT(bugprone-easily-swappable-parameters)
    : vve_(velocityEstimator),
      filteredDecel_mps2_(0.0F),
      alpha_(clamp(accelFilterAlpha, 0.0F, 1.0F)),
      minimumClimbVelocity_mps_(minimumClimbVelocity_mps),
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
        const auto deltaTime_s = static_cast<float>(currentTimestamp - lastTs_) * kMillisecondsToSeconds;
        const float deltaVelocity_mps = velocity - lastVel_;
        float estimatedDecel_mps2 = (deltaTime_s > 0.0F) ? std::max(0.0F, -deltaVelocity_mps / deltaTime_s) : 0.0F;
        decelSample = kOneHalf * (decelSample + estimatedDecel_mps2);
    }

    filteredDecel_mps2_ = alpha_ * decelSample + (1.0F - alpha_) * filteredDecel_mps2_;

    if (velocity > minimumClimbVelocity_mps_ && filteredDecel_mps2_ > kMinValidDecel) {
        tToApogee_ = velocity / filteredDecel_mps2_;
        predApogeeTs_ = currentTimestamp + static_cast<uint32_t>(tToApogee_ * kMillisecondsPerSecond);

        const float altitude = vve_.getEstimatedAltitude();
        predApogeeAlt_ = altitude + velocity * tToApogee_ -
                         kOneHalf * filteredDecel_mps2_ * tToApogee_ * tToApogee_;

        valid_ = true;
    } else {
        valid_ = false;
    }

    lastTs_ = currentTimestamp;
    lastVel_ = velocity;
    numWarmups_ = std::min(numWarmups_ + 1, kMaxWarmups);
}

void ApogeePredictor::quadUpdate() {
    const uint32_t currentTimestamp = vve_.getTimestamp();
    const float velocity = vve_.getEstimatedVelocity();
    const float acceleration = vve_.getInertialVerticalAcceleration();

    float kEstimate = 0.0F;
    if (std::fabs(velocity) > 1.0F) {
        kEstimate = std::max(0.0F, -(acceleration + kGravity_mps2)) / (velocity * velocity);
    }

    filteredDecel_mps2_ = alpha_ * kEstimate + (1.0F - alpha_) * filteredDecel_mps2_;
    const float dragToMassRatio = filteredDecel_mps2_;

    if (velocity > minimumClimbVelocity_mps_ && dragToMassRatio > kMinValidDecel) {
        const float terminalVelocity = std::sqrt(kGravity_mps2 / dragToMassRatio);
        tToApogee_ = (terminalVelocity / kGravity_mps2) * std::atan(velocity / terminalVelocity);
        const float deltaAltitude = (terminalVelocity * terminalVelocity / (2.0F * kGravity_mps2)) *
                                    std::log1p((velocity * velocity) /
                                               (terminalVelocity * terminalVelocity));

        predApogeeTs_ = currentTimestamp + static_cast<uint32_t>(tToApogee_ * kMillisecondsPerSecond);
        predApogeeAlt_ = vve_.getEstimatedAltitude() + deltaAltitude;
        valid_ = true;
    } else {
        valid_ = false;
    }

    lastTs_ = currentTimestamp;
    lastVel_ = velocity;
}
void ApogeePredictor::polyUpdate() {
    const uint32_t currentTimestamp_ms = vve_.getTimestamp();
    const float altitude_m = vve_.getEstimatedAltitude();
    const float velocity_mps = vve_.getEstimatedVelocity();
    const float acceleration_mps2 = vve_.getInertialVerticalAcceleration();

    constexpr size_t kFeatureCount = 10; // NOLINT(cppcoreguidelines-init-variables)

    // Polynomial Regression Coefficients for C++
    const std::array<float, kFeatureCount> coeffs = {
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
    const float decel = std::fabs(acceleration_mps2);
    const float delta_h_simple = kOneHalf * (velocity_mps * velocity_mps) / decel;

    // ───────────────────────────────────────────────────────
    // Evaluate the regression model
    const std::array<float, kFeatureCount> inputs = {
        1.0F,
        velocity_mps,               // vertical_velocity
        acceleration_mps2,          // vertical_acceleration
        delta_h_simple,
        velocity_mps * velocity_mps,  // vertical_velocity^2
        velocity_mps * acceleration_mps2, // vertical_velocity vertical_acceleration
        velocity_mps * delta_h_simple, // vertical_velocity delta_h_simple
        acceleration_mps2 * acceleration_mps2, // vertical_acceleration^2
        acceleration_mps2 * delta_h_simple, // vertical_acceleration delta_h_simple
        delta_h_simple * delta_h_simple, // delta_h_simple^2
    };

    float apogeeRemaining_m = intercept;
    for (size_t i = 0; i < kFeatureCount; ++i) { // NOLINT(cppcoreguidelines-init-variables)
        apogeeRemaining_m += coeffs[i] * inputs[i];
    }

    // ───────────────────────────────────────────────────────
    // Combine with current altitude to compute predicted apogee
    predApogeeAlt_ = altitude_m + apogeeRemaining_m;

    // Estimate time to apogee using kinematic model
    tToApogee_ = velocity_mps / decel;
    predApogeeTs_ = currentTimestamp_ms + static_cast<uint32_t>(tToApogee_ * kMillisecondsPerSecond);

    valid_ = true;

    lastTs_ = currentTimestamp_ms;
    lastVel_ = velocity_mps;
    numWarmups_ = std::min(numWarmups_ + 1, kMaxWarmups);

    // printf("Current Timestamp: %u, Altitude: %.2f, Velocity: %.2f, Acceleration: %.2f, Predicted Apogee Remaining: %.2f, Delta H: %.2f\n",
    //        currentTimestamp_ms, altitude_m, velocity_mps, acceleration_mps2, apogeeRemaining_m, delta_h_simple);
}


void ApogeePredictor::analyticUpdate()
{
    //gets the current velocity and altitude of the rocket
    const float velocity = vve_.getEstimatedVelocity();
    const float height = vve_.getEstimatedAltitude();


    //variables for analytical calculation
    const float kVelocityEpsilon = 0.001F;
    const float kVelocityScaleForAlpha = 150.0F;
    const float kAlphaMin = 0.02F;
    const float kAlphaMax = 0.25F;
    const float kMinDragCoefficient = 0.00001F;
    const float kApogeeFactor = 0.5F;
    const float kBallisticDenominator = 2.0F;

    //if the velocity is less than or equal to zero, the rocket has already reach apogee and the apogee is the current altitude
    if (velocity <= 0.0F)
    {
        predApogeeAlt_ = height;
        valid_ = true;
        return;
    }

    //gets the current acceleration of the rocket
    const float acceleration = vve_.getInertialVerticalAcceleration();

    //calculates the measured drag coefficient
    const float kMeasured = -(acceleration + kGravity_mps2) /
        (velocity * velocity + kVelocityEpsilon);

    if (kMeasured > 0.0F && kMeasured < 1.0F)
    {
        float alpha = clamp(std::fabs(velocity) / kVelocityScaleForAlpha,
                    kAlphaMin,
                    kAlphaMax);
        currentDragCoefficient_ = (1.0F - alpha) * currentDragCoefficient_ + alpha * kMeasured;
    }

    // Analytic apogee calculation
    float apogee = 0.0F;

    if (currentDragCoefficient_ > kMinDragCoefficient)
    {
        apogee = height + (kApogeeFactor / currentDragCoefficient_) *
        logf((kGravity_mps2 + currentDragCoefficient_ * velocity * velocity) / kGravity_mps2);
    }
    else
    {
        // fallback if drag unknown
        apogee = height + (velocity * velocity) /
        (kBallisticDenominator * kGravity_mps2);
    }

    predApogeeAlt_ = apogee;
    valid_ = true;
}


void ApogeePredictor::simulateUpdate()
{
    const float estimatedVelocity = vve_.getEstimatedVelocity();
    const float estimatedAltitude = vve_.getEstimatedAltitude();
    const float inertialAccel = vve_.getInertialVerticalAcceleration();

    const float kVelocityEpsilon = 0.001F;
    const float kMinVelocityForDrag = 15.0F;
    const float kAlpha = 0.05F;           // slow filter constant
    const float kMaxDragCoefficient = 0.05F;
    const float kDt = 0.01F;              // simulation time step
    const int kMaxSimSteps = 500;         // safety limit

    // Already descending
    if (estimatedVelocity <= 0.0F)
    {
        predApogeeAlt_ = estimatedAltitude;
        valid_ = true;
        return;
    }

    // Only update drag during cost phase
    if (estimatedVelocity > kMinVelocityForDrag)
    {
        const float measuredDrag =
            -(inertialAccel + kGravity_mps2) / (estimatedVelocity * estimatedVelocity + kVelocityEpsilon);

        if (measuredDrag > 0.0F && measuredDrag < kMaxDragCoefficient)
        {
            currentDragCoefficient_ =
                (1.0F - kAlpha) * currentDragCoefficient_ +
                kAlpha * measuredDrag;
        }
    }

    // -------- Forward simulate trajectory --------
    float simAltitude = estimatedAltitude;
    float simVelocity = estimatedVelocity;

    for (int step = 0; step < kMaxSimSteps; step++)
    {
        const float dragAcceleration = currentDragCoefficient_ * simVelocity * simVelocity;
        const float totalAcceleration = -kGravity_mps2 - dragAcceleration;

        simVelocity += totalAcceleration * kDt;
        simAltitude += simVelocity * kDt;

        if (simVelocity <= 0.0F)
        {
            break;
        }
    }

    predApogeeAlt_ = simAltitude;
    valid_ = true;
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
    return filteredDecel_mps2_;
}

float ApogeePredictor::getDragCoefficient() const {
    return currentDragCoefficient_;
}
