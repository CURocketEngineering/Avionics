#ifndef SIMPLESIMULATION_H
#define SIMPLESIMULATION_H

#include <cstdint>

/**
 * @brief A simple rocket simulation for testing apogee and launch detection algorithms.
 *
 * The simulator uses a millisecond time base. Before the launch time the rocket
 * remains on the ground (with 0 acceleration, velocity, and altitude). At or after
 * launch, the simulation begins. During motor burn (from launch until launchTime + motorBurnTime)
 * the inertial sensor reading (returned by getIntertialVerticalAcl()) is the provided motorAcceleration,
 * and the net acceleration applied to the rocket’s velocity is (motorAcceleration - 9.81 m/s²).
 * After motor burn the only acceleration is -9.81 m/s².
 */
class SimpleSimulator {
public:
    /**
     * @brief Constructs the rocket simulator with the given motor parameters.
     * @param launchTime_ms The time (in milliseconds) at which the rocket is launched.
     * @param motorAcceleration The thrust acceleration provided by the motor (in m/s²).
     * @param motorBurnTime_ms The duration of the motor burn (in milliseconds).
     * @param tick_ms The simulation time step (in milliseconds). Use 10 for 100 Hz.
     */
    SimpleSimulator(uint32_t launchTime_ms, float motorAcceleration, uint32_t motorBurnTime_ms, uint32_t tick_ms);

    /**
     * @brief Advances the simulation by one tick (time step).
     *
     * This method updates the rocket’s acceleration, velocity, altitude, and
     * also records the apogee (highest altitude reached) and its timestamp.
     */
    void tick();

    /**
     * @brief Gets the current inertial vertical acceleration (m/s²).
     * 
     * Examples:
     * - On ground: 0.0 m/s²
     * - During boost: (e.g.) 50.00 m/s²
     * - During coast: -9.81 m/s²
     * - In free-fall: -9.81 m/s²
     * 
     * @return The current vertical acceleration.
     */
    float getIntertialVerticalAcl() const;

    /**
     * @brief Gets the current vertical velocity (m/s).
     * @return The current vertical velocity.
     */
    float getVerticalVel() const;

    /**
     * @brief Gets the current altitude (m).
     * @return The current altitude.
     */
    float getAltitude() const;

    /**
     * @brief Gets the apogee altitude (highest altitude reached) (m).
     * @return The apogee altitude.
     */
    float getApogeeAlt() const;

    /**
     * @brief Gets the timestamp (in milliseconds) at which apogee occurred.
     * @return The apogee timestamp in ms.
     */
    uint32_t getApogeeTimestamp() const;

    /**
     * @brief Gets the launch timestamp (in milliseconds).
     * @return The launch timestamp in ms.
     */
    uint32_t getLaunchTimestamp() const;

    bool getHasLanded() const;

    uint32_t getCurrentTime() const;

private:
    // --- Simulation parameters ---
    uint32_t launchTime_ms_;      // Time at which the rocket launches (ms)
    float motorAcceleration_;     // Motor thrust acceleration (m/s²) when firing
    uint32_t motorBurnTime_ms_;   // Duration of motor burn (ms)
    uint32_t tick_ms_;            // Simulation tick period (ms)

    // --- Simulation state ---
    uint32_t currentTime_ms_;     // Current simulation time (ms)
    float altitude_;              // Current altitude (m)
    float velocity_;              // Current vertical velocity (m/s)
    float netAcceleration_;       // Net acceleration applied to velocity (m/s²)
    bool hasLanded_;            // Simulation complete flag

    // --- Apogee tracking ---
    float apogeeAltitude_;        // Maximum altitude reached (m)
    uint32_t apogeeTimestamp_ms_; // Timestamp (ms) when apogee was recorded
};

#endif  // SIMPLESIMULATION_H
