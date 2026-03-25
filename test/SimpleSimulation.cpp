#include "SimpleSimulation.h"

SimpleSimulator::SimpleSimulator(uint32_t launchTime_ms, float motorAcceleration, uint32_t motorBurnTime_ms, uint32_t tick_ms)
    : launchTime_ms_(launchTime_ms),
      motorAcceleration_(motorAcceleration),
      motorBurnTime_ms_(motorBurnTime_ms),
      tick_ms_(tick_ms),
      currentTime_ms_(0),
      altitude_(0.0f),
      velocity_(0.0f),
      netAcceleration_(0.0f),
      apogeeAltitude_(0.0f),
      apogeeTimestamp_ms_(0)
{
}

void SimpleSimulator::tick() {
    // Convert tick duration from milliseconds to seconds.
    float dt = tick_ms_ / 1000.0f;

    // Determine net acceleration for integration based on the simulation time.
    if (currentTime_ms_ < launchTime_ms_) {
        // Before launch: the rocket is on the pad (zero acceleration).
        netAcceleration_ = 0.0f;
    }
    else if (currentTime_ms_ < launchTime_ms_ + motorBurnTime_ms_) {
        // During boost: motor is firing.
        // For physics integration, the net acceleration is motor thrust minus gravity.
        netAcceleration_ = motorAcceleration_ - 9.81f;
    }
    else {
        // After motor burn: only gravity acts.
        netAcceleration_ = -9.81f;
    }

    // Update physics only after launch.
    if (currentTime_ms_ >= launchTime_ms_) {
        // Euler integration: update velocity and altitude.
        velocity_ += netAcceleration_ * dt;
        altitude_ += velocity_ * dt;

        // Prevent the altitude from dropping below ground level.
        if (altitude_ < 0.0f) {
            altitude_ = 0.0f;
            velocity_ = 0.0f;
            hasLanded_ = true;
        }

        // Record apogee if we start descending.
        if (velocity_ < 0.0f && altitude_ > apogeeAltitude_) {
            apogeeAltitude_ = altitude_;
            apogeeTimestamp_ms_ = currentTime_ms_;
        }
    }

    // Advance the simulation time by one tick.
    currentTime_ms_ += tick_ms_;
}

float SimpleSimulator::getIntertialVerticalAcl() const {
    return netAcceleration_;
}

float SimpleSimulator::getVerticalVel() const {
    return velocity_;
}

float SimpleSimulator::getAltitude() const {
    return altitude_;
}

float SimpleSimulator::getApogeeAlt() const {
    return apogeeAltitude_;
}

uint32_t SimpleSimulator::getApogeeTimestamp() const {
    return apogeeTimestamp_ms_;
}

uint32_t SimpleSimulator::getLaunchTimestamp() const {
    return launchTime_ms_;
}

bool SimpleSimulator::getHasLanded() const {
    return hasLanded_;
}

uint32_t SimpleSimulator::getCurrentTime() const {
    return currentTime_ms_;
}
