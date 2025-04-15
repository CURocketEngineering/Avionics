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
      lastVel_(0.0f),
      numWarmups_(0)
{}

void ApogeePredictor::update()
{
    const uint32_t ts   = vve_.getTimestamp();
    const float    vel  = vve_.getEstimatedVelocity();          // m/s
    const float    acc  = vve_.getInertialVerticalAcceleration(); // m/s² (neg when climbing)

    /* ----- compute a deceleration sample ----- */
    float decelSample = std::max(0.0f, -acc);   // |a| from IMU

    /* fallback / cross‑check using Δv / Δt */ // ------- take avg between actual and theoretical to avoid mess up
    if (ts > lastTs_) {
        const float dt = (ts - lastTs_) * 1e-3f;
        const float dv = vel - lastVel_;        // typically negative while climbing
        const float dvDecel = (dt > 0.0f) ? std::max(0.0f, -dv / dt) : 0.0f;
        /* average the two estimates */
        decelSample = 0.5f * (decelSample + dvDecel);
    }

    /* ----- low‑pass filter to tame noise ----- */  // ------- bc applies to instance, applies to entire flight
    filteredDecel_ = alpha_ * decelSample + (1.0f - alpha_) * filteredDecel_;

    /* ----- predict apogee if still climbing ----- */
    if (vel > minClimbVel_ && filteredDecel_ > 0.1f && numWarmups_ > 10) {
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

    numWarmups_++;
    if (numWarmups_ > 1000) {
        numWarmups_ = 1000; // cap to avoid overflow
    }
}

/*  ──────────────────────────────────────────────────────────────
NEW  “quadratic‑drag” apogee predictor
dv/dt = –g – k·v² ,   k = ρ·Cd·A / (2m)
--------------------------------------------------------------
time‑to‑apogee  tₐ =  Vt/g · atan(v / Vt)
altitude gain   hₐ = (Vt² / 2g) · ln(1 + v² / Vt²)
where Vt = √(g/k)  is the (up‑direction) terminal velocity.
See NASA GRC “Flight Equations with Drag” :contentReference[oaicite:0]{index=0}
────────────────────────────────────────────────────────────── */
void ApogeePredictor::quad_update()
{
    constexpr float g = 9.80665f;                       // m s⁻²

    /* ----- current flight state from estimators ----- */
    const uint32_t ts   = vve_.getTimestamp();
    const float    vel  = vve_.getEstimatedVelocity();          // m s⁻¹ ( + up )
    const float    acc  = vve_.getInertialVerticalAcceleration(); // m s⁻²

    /* ----- estimate “k” (drag‑to‑mass) from the ODE ----- */
    // dv/dt = acc = –g – k·v²  ⇒  k = –(acc + g)/v²
    float kSample = 0.0f;
    if (std::fabs(vel) > 1.0f) {                      // avoid div 0 near apogee
        kSample = std::max(0.0f, -(acc + g)) / (vel * vel);
    }

    /* ----- low‑pass filter k to kill noise (same α as before) ----- */
    filteredDecel_ = alpha_ * kSample + (1.0f - alpha_) * filteredDecel_;
    const float k  = filteredDecel_;

    /* ---------- predict only while still climbing ---------- */
    if (vel > minClimbVel_ && k > .000001f) {

        /* ----- closed‑form ballistic solution with quad‑drag ----- */
        const float Vt = std::sqrt(g / k);                         // m s⁻¹
        tToApogee_      = (Vt / g) * std::atan(vel / Vt);          // s
        const float dH  = (Vt * Vt / (2.0f * g))
                        * std::log1p((vel * vel) / (Vt * Vt));     // m

        predApogeeTs_   = ts + static_cast<uint32_t>(tToApogee_ * 1e3f);
        predApogeeAlt_  = vve_.getEstimatedAltitude() + dH;
        valid_          = true;
    } else {
        valid_ = false;
    }

    /* ---------- book‑keeping ---------- */
    lastTs_  = ts;
    lastVel_ = vel;
}

/* ---------- simple getters ---------- */
bool  ApogeePredictor::isPredictionValid()            const { return valid_;        }
float ApogeePredictor::getTimeToApogee_s()            const { return valid_ ? tToApogee_     : 0.0f; }
uint32_t ApogeePredictor::getPredictedApogeeTimestamp_ms() const { return valid_ ? predApogeeTs_ : 0;  }
float ApogeePredictor::getPredictedApogeeAltitude_m() const { return valid_ ? predApogeeAlt_ : 0.0f; }
float ApogeePredictor::getFilteredDeceleration()      const { return filteredDecel_; }
