#ifndef BASE_STATE_MACHINE_H
#define BASE_STATE_MACHINE_H

#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/States.h"

/**
 * @brief Abstract interface for flight state machines driven by IMU/altimeter data.
 * @note When to use: derive a concrete state machine to map sensor inputs to
 *       discrete flight phases without changing call sites.
 */
class BaseStateMachine {
    public:
        /**
         * @brief Advance the state machine with the latest measurements.
         * @param accel Acceleration vector readings.
         * @param alt   Altimeter sample.
         * @note When to use: call every sensor update; return codes can signal
         *       events or errors.
         */
        virtual int update(const AccelerationTriplet& accel, const DataPoint& alt) = 0;

        /**
         * @brief Current discrete state identifier.
         * @note When to use: downstream logic (ejection, logging, UI) queries
         *       this to decide actions.
         */
        virtual uint8_t getState() const = 0;
};

#endif