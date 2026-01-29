#ifndef APOGEE_PREDICTOR_H
#define APOGEE_PREDICTOR_H

#include <cstdint>

#include "state_estimation/VerticalVelocityEstimator.h"

/**
 * @brief Predicts time‑to‑apogee and altitude‑at‑apogee from real‑time kinematics.
 *
 * The predictor treats the *current* deceleration (loss of vertical velocity)
 * as constant and analytically projects forward:
 *
 *      t_apogee = v / |a|          (time until v → 0)
 *      h_apogee = h + v·t − ½|a|t²
 *
 * where v and a are the **filtered** estimates of vertical velocity and
 * net vertical acceleration supplied by the VerticalVelocityEstimator.
 *
 * To damp sensor noise, we run a single‑pole low‑pass filter on the measured
 * deceleration magnitude. Call `update()` after each estimator refresh.
 *
 * @note When to use: provide early apogee timing/altitude estimates for
 *       telemetry or adaptive control while still ascending.
 */
class ApogeePredictor {
public:
    /**
     * @param velocityEstimator       Reference to the velocity estimator
     * @param accelFilterAlpha             EMA weight for smoothing deceleration [0–1]
     * @param minimumClimbVelocity_ms   Minimum upward velocity (m/s) for a valid prediction
     */
    explicit ApogeePredictor(const VerticalVelocityEstimator& velocityEstimator,
                             float accelFilterAlpha = 0.2F,
                             float minimumClimbVelocity_ms = 1.0F);

    /** Call after every estimator refresh to update prediction */
    void update();

    /** Optional: Update using a quadratic-drag model (more accurate under drag) */
    void quad_update();

    void poly_update();

    // ----- Accessors -----
    [[nodiscard]] bool     isPredictionValid()            const;
    [[nodiscard]] float    getTimeToApogee_s()            const;
    [[nodiscard]] uint32_t getPredictedApogeeTimestamp_ms() const;
    [[nodiscard]] float    getPredictedApogeeAltitude_m() const;
    [[nodiscard]] float    getFilteredDeceleration()      const;

private:
    const VerticalVelocityEstimator& vve_;

    float filteredDecel_;   ///< Smoothed deceleration (m/s², positive)
    float alpha_;           ///< EMA smoothing weight
    float minClimbVel_;     ///< Minimum climb speed (m/s) to consider a prediction

    // Latest prediction results
    bool     valid_;        ///< Whether the current prediction is valid
    float    tToApogee_;    ///< Time until apogee (seconds)
    uint32_t predApogeeTs_; ///< Timestamp of predicted apogee (ms)
    float    predApogeeAlt_;///< Predicted altitude at apogee (m)

    // Bookkeeping
    uint32_t lastTs_;       ///< Last timestamp received
    float    lastVel_;      ///< Last vertical velocity received
    uint32_t numWarmups_;   ///< Number of updates run (for warm-up tracking)
};

#endif // APOGEE_PREDICTOR_H
