#ifndef BASE_STATE_MACHINE_H
#define BASE_STATE_MACHINE_H

#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/States.h"

class BaseStateMachine {
    // Common interface that all StateMachines must implement
    // This allows for easy swapping of StateMachines
    public:
        virtual int update(const AccelerationTriplet& accel, const DataPoint& alt) = 0;
        virtual uint8_t getState() const = 0;
};

#endif