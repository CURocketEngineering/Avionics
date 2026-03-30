#include "state_estimation/BaseStateMachine.h"

BaseStateMachine::BaseStateMachine(FlightState initialState) : state_(initialState) {}

uint8_t BaseStateMachine::getState() const {
    return static_cast<uint8_t>(state_);
}

bool BaseStateMachine::registerOnStateEntry(FlightState targetState, StateEntryCallback functionPtr) {
    if (functionPtr == nullptr) {
        return false;
    }

    // Searching for a duplicate registration
    for (std::size_t i = 0; i < callbackCount_; i++) {
        const StateCallbackRegistration& registration = onStateEntryCallbacks_[i];
        if (registration.state == targetState && registration.functionPtr == functionPtr) {
            return false;
        }
    }

    // Checking if we have room for another callback
    if (callbackCount_ >= kMaxStateEntryCallbacks) {
        return false;
    }

    // Register the new callback
    onStateEntryCallbacks_[callbackCount_] = {targetState, functionPtr};
    callbackCount_++;
    return true;
}

bool BaseStateMachine::changeState(FlightState newState) {
    if (state_ == newState) {
        return false;
    }

    state_ = newState;

    // Calling the registered callbacks for the new state
    for (std::size_t i = 0; i < callbackCount_; i++) {
        const StateCallbackRegistration& registration = onStateEntryCallbacks_[i];
        if (registration.state == state_) {
            registration.functionPtr();
        }
    }

    return true;
}

FlightState BaseStateMachine::getFlightState() const {
    return state_;
}
