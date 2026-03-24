#include "state_estimation/BaseStateMachine.h"

BaseStateMachine::BaseStateMachine(FlightState initialState) : state(initialState) {}

uint8_t BaseStateMachine::getState() const {
    return static_cast<uint8_t>(state);
}

bool BaseStateMachine::registerOnStateEntry(FlightState targetState, StateEntryCallback functionPtr) {
    if (functionPtr == nullptr) {
        return false;
    }

    // Searching for a duplicate registration
    for (std::size_t i = 0; i < callbackCount; i++) {
        const StateCallbackRegistration& registration = onStateEntryCallbacks[i];
        if (registration.state == targetState && registration.functionPtr == functionPtr) {
            return false;
        }
    }

    // Checking if we have room for another callback
    if (callbackCount >= kMaxStateEntryCallbacks) {
        return false;
    }

    // Register the new callback
    onStateEntryCallbacks[callbackCount] = {targetState, functionPtr};
    callbackCount++;
    return true;
}

bool BaseStateMachine::changeState(FlightState newState) {
    if (state == newState) {
        return false;
    }

    state = newState;

    // Calling the registered callbacks for the new state
    for (std::size_t i = 0; i < callbackCount; i++) {
        const StateCallbackRegistration& registration = onStateEntryCallbacks[i];
        if (registration.state == state) {
            registration.functionPtr();
        }
    }

    return true;
}

FlightState BaseStateMachine::getFlightState() const {
    return state;
}
