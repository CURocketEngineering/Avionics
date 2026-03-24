#include "state_estimation/BaseStateMachine.h"

BaseStateMachine::BaseStateMachine(FlightState initialState) : state(initialState) {}

uint8_t BaseStateMachine::getState() const {
    return static_cast<uint8_t>(state);
}

bool BaseStateMachine::registerOnStateEntry(FlightState targetState, StateEntryCallback fn) {
    if (fn == nullptr) {
        return false;
    }

    for (const StateCallbackRegistration& registration : onStateEntryCallbacks) {
        if (registration.state == targetState && registration.fn == fn) {
            return false;
        }
    }

    onStateEntryCallbacks.push_back({targetState, fn});
    return true;
}

bool BaseStateMachine::changeState(FlightState newState) {
    if (state == newState) {
        return false;
    }

    state = newState;

    for (const StateCallbackRegistration& registration : onStateEntryCallbacks) {
        if (registration.state == state) {
            registration.fn();
        }
    }

    return true;
}

FlightState BaseStateMachine::getFlightState() const {
    return state;
}
