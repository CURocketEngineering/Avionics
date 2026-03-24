#ifndef AIRRESISTANCESIMULATION_H
#define AIRRESISTANCESIMULATION_H

#include <cmath>
#include <cstdint>

/**
 * @brief 1‑D rocket simulator that adds a quadratic drag term.
 *
 * Drag is modelled with a single tunable coefficient *k* such that
 *     a_drag = –k · v · |v|
 * (always opposite the direction of travel).  k lumps together the usual
 * ½ρCdA / m term, so you can treat it as “acceleration per (m/s)²”.
 *
 * Call setDragCoefficient() at any point (even every tick) to change k and
 * emulate aero‑braking surfaces deploying / retracting.
 */
class AirResistanceSimulator {
public:
    AirResistanceSimulator(uint32_t launchTime_ms,
                           float     motorAcceleration,
                           uint32_t  motorBurnTime_ms,
                           uint32_t  tick_ms,
                           float     dragCoefficient = 0.0f);

    /* ------------ core loop ------------ */
    void tick();                       ///< advance one time‑step

    /* ------------ live data ------------ */
    float    getInertialVerticalAcl() const;   ///< what the IMU “feels”
    float    getVerticalVel()         const;
    float    getAltitude()            const;

    /* ------------ flight events -------- */
    float    getApogeeAlt()           const;
    uint32_t getApogeeTimestamp()     const;
    bool     getHasLanded()           const;

    /* ------------ timing --------------- */
    uint32_t getLaunchTimestamp()     const;
    uint32_t getCurrentTime()         const;

    /* ------------ drag control --------- */
    void  setDragCoefficient(float k);
    float getDragCoefficient() const;

private:
    /* --- user parameters --- */
    uint32_t launchTime_ms_;
    float    motorAccel_;          // thrust accel felt by IMU during burn
    uint32_t motorBurnTime_ms_;
    uint32_t tick_ms_;
    float    dragCoeff_;           // k term in –k·v·|v|

    /* --- state --- */
    uint32_t t_ms_{0};
    float    alt_{0.0f};
    float    vel_{0.0f};
    float    netAccel_{0.0f};
    bool     landed_{false};

    /* --- apogee tracking --- */
    float    apogeeAlt_{0.0f};
    uint32_t apogeeTime_ms_{0};
};

#endif  // AIRRESISTANCESIMULATION_H
