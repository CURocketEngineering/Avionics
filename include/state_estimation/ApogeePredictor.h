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
 * To damp sensor noise we run a single‑pole low‑pass filter on the measured
 * deceleration magnitude.  Call update() immediately after each
 * VerticalVelocityEstimator update.
 */
class ApogeePredictor {
public:
    /**
     * @param vve                 reference to the estimator that owns the data
     * @param accelFilterAlpha    0–1, weighting for the deceleration EMA
     * @param minClimbVel         minimum upward velocity (m/s) required before
     *                            a prediction is considered valid
     */
    explicit ApogeePredictor(const VerticalVelocityEstimator& vve,
                             float accelFilterAlpha = 0.2f,
                             float minClimbVel      = 1.0f);

    /** Call every time the estimator is refreshed. */
    void update();

    /* ------------ accessors ------------- */
    bool     isPredictionValid()            const;   ///< true if still climbing & estimate ok
    float    getTimeToApogee_s()            const;   ///< seconds from *now*
    uint32_t getPredictedApogeeTimestamp_ms() const; ///< absolute timestamp (ms)
    float    getPredictedApogeeAltitude_m() const;   ///< metres above ground
    float    getFilteredDeceleration()      const;   ///< m/s² (positive)

private:
    const VerticalVelocityEstimator& vve_;

    /* filtered deceleration (positive, m/s²) */
    float filteredDecel_;
    float alpha_;
    float minClimbVel_;

    /* latest prediction */
    bool     valid_;
    float    tToApogee_;
    uint32_t predApogeeTs_;
    float    predApogeeAlt_;

    /* book‑keeping for Δv/Δt fallback */
    uint32_t lastTs_;
    float    lastVel_;
};

#endif // APOGEE_PREDICTOR_H
