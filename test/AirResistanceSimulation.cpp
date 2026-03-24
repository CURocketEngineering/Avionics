#include "AirResistanceSimulation.h"

AirResistanceSimulator::AirResistanceSimulator(uint32_t launchTime_ms,
                                               float     motorAcceleration,
                                               uint32_t  motorBurnTime_ms,
                                               uint32_t  tick_ms,
                                               float     dragCoefficient)
    : launchTime_ms_(launchTime_ms),
      motorAccel_(motorAcceleration),
      motorBurnTime_ms_(motorBurnTime_ms),
      tick_ms_(tick_ms),
      dragCoeff_(dragCoefficient)
{}

/* ---------------- drag control ---------------- */
void AirResistanceSimulator::setDragCoefficient(float k)   { dragCoeff_ = k; }
float AirResistanceSimulator::getDragCoefficient() const   { return dragCoeff_; }

/* ---------------- main loop ------------------- */
void AirResistanceSimulator::tick()
{
    if (landed_) {            // nothing left to integrate
        t_ms_ += tick_ms_;
        return;
    }

    const float dt = tick_ms_ / 1000.0f;

    /* ---- determine forces ---- */
    if (t_ms_ < launchTime_ms_) {
        netAccel_ = 0.0f;                         // on pad
    } else {
        const bool burning = t_ms_ < launchTime_ms_ + motorBurnTime_ms_;
        const float thrustA = burning ? motorAccel_ : 0.0f;
        const float gravityA = -9.81f;
        const float dragA = (vel_ == 0.0f) ? 0.0f
                         : -dragCoeff_ * vel_ * std::abs(vel_);

        netAccel_ = thrustA + gravityA + dragA;
    }

    /* ---- integrate once we’re off the pad ---- */
    if (t_ms_ >= launchTime_ms_) {
        vel_ += netAccel_ * dt;
        alt_ += vel_ * dt;

        /* record apogee when we start descending */
        if (vel_ < 0.0f && alt_ > apogeeAlt_) {
            apogeeAlt_  = alt_;
            apogeeTime_ms_ = t_ms_;
        }

        /* simple ground contact */
        if (alt_ <= 0.0f) {
            alt_   = 0.0f;
            vel_   = 0.0f;
            netAccel_ = 0.0f;
            landed_ = true;
        }
    }

    t_ms_ += tick_ms_;
}

/* ---------------- getters --------------------- */
float    AirResistanceSimulator::getInertialVerticalAcl() const { return netAccel_; }
float    AirResistanceSimulator::getVerticalVel()         const { return vel_;      }
float    AirResistanceSimulator::getAltitude()            const { return alt_;      }

float    AirResistanceSimulator::getApogeeAlt()           const { return apogeeAlt_;}
uint32_t AirResistanceSimulator::getApogeeTimestamp()     const { return apogeeTime_ms_; }

bool     AirResistanceSimulator::getHasLanded()           const { return landed_;   }

uint32_t AirResistanceSimulator::getLaunchTimestamp()     const { return launchTime_ms_; }
uint32_t AirResistanceSimulator::getCurrentTime()         const { return t_ms_; }
