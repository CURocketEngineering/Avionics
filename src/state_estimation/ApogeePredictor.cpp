#include "state_estimation/ApogeePredictor.h"
#include <algorithm>
#include <cmath>

template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}


ApogeePredictor::ApogeePredictor(const VerticalVelocityEstimator& vve,
                                 float accelFilterAlpha,
                                 float minClimbVel)
    : vve_(vve),
      filteredDecel_(0.0f),
      alpha_(clamp(accelFilterAlpha, 0.0f, 1.0f)),
      minClimbVel_(minClimbVel),
      valid_(false),
      tToApogee_(0.0f),
      predApogeeTs_(0),
      predApogeeAlt_(0.0f),
      lastTs_(0),
      lastVel_(0.0f)
{}

void ApogeePredictor::update()
{
    const uint32_t ts   = vve_.getTimestamp();
    const float    vel  = vve_.getEstimatedVelocity();          // m/s
    const float    acc  = vve_.getInertialVerticalAcceleration(); // m/s² (neg when climbing)

    /* ----- compute a deceleration sample ----- */
    float decelSample = std::max(0.0f, -acc);   // |a| from IMU

    /* fallback / cross‑check using Δv / Δt */
    if (ts > lastTs_) {
        const float dt = (ts - lastTs_) * 1e-3f;
        const float dv = vel - lastVel_;        // typically negative while climbing
        const float dvDecel = (dt > 0.0f) ? std::max(0.0f, -dv / dt) : 0.0f;
        /* average the two estimates */
        decelSample = 0.5f * (decelSample + dvDecel);
    }

    /* ----- low‑pass filter to tame noise ----- */
    filteredDecel_ = alpha_ * decelSample + (1.0f - alpha_) * filteredDecel_;

    /* ----- predict apogee if still climbing ----- */
    if (vel > minClimbVel_ && filteredDecel_ > 0.1f) {
        tToApogee_      = vel / filteredDecel_;          // s
        predApogeeTs_   = ts + static_cast<uint32_t>(tToApogee_ * 1000.0f);

        const float alt = vve_.getEstimatedAltitude();
        predApogeeAlt_  = alt + vel * tToApogee_ - 0.5f * filteredDecel_ * tToApogee_ * tToApogee_;

        valid_ = true;
    } else {
        valid_ = false;
    }

    /* save for next iteration */
    lastTs_  = ts;
    lastVel_ = vel;
}

/* ---------- simple getters ---------- */
bool  ApogeePredictor::isPredictionValid()            const { return valid_;        }
float ApogeePredictor::getTimeToApogee_s()            const { return valid_ ? tToApogee_     : 0.0f; }
uint32_t ApogeePredictor::getPredictedApogeeTimestamp_ms() const { return valid_ ? predApogeeTs_ : 0;  }
float ApogeePredictor::getPredictedApogeeAltitude_m() const { return valid_ ? predApogeeAlt_ : 0.0f; }
float ApogeePredictor::getFilteredDeceleration()      const { return filteredDecel_; }
