#ifndef BASE_STATE_MACHINE_H
#define BASE_STATE_MACHINE_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/States.h"

/**
 * @brief Base class for flight state machines driven by IMU/altimeter data.
 * @note When to use: derive a concrete state machine to map sensor inputs to
 *       discrete flight phases without changing call sites. This class owns
 *       current-state tracking plus on-entry callback dispatch with no dynamic
 *       memory allocation.
 */
class BaseStateMachine {
    public:
        // Type alias for a function pointer with void return and no args
        using StateEntryCallback = void (*)();

        static constexpr std::size_t kMaxStateEntryCallbacks = 32;

        explicit BaseStateMachine(FlightState initialState = STATE_UNARMED);
        virtual ~BaseStateMachine() = default;

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
        virtual uint8_t getState() const;

        /**
         * @brief Register a callback to invoke each time a target state is entered.
         * @param state The state that triggers the callback.
         * @param functionPtr Function to call when entering @p state.
         * @return true if callback was registered, false for nullptr, duplicate,
         *         or full callback buffer.
         */
        bool registerOnStateEntry(FlightState targetState, StateEntryCallback functionPtr);

        static constexpr std::size_t getMaxStateEntryCallbacks() {
            return kMaxStateEntryCallbacks;
        }

    protected:
        /**
         * @brief Transition to a new state and trigger registered on-entry callbacks.
         * @param newState State to transition into.
         * @return true if state changed, false if already in @p newState.
         */
        bool changeState(FlightState newState);

        /**
         * @brief Current state as FlightState enum.
         */
        FlightState getFlightState() const;

    private:
        struct StateCallbackRegistration {
            FlightState state;
            StateEntryCallback functionPtr;
        };

        FlightState state;
        std::size_t callbackCount = 0;
        std::array<StateCallbackRegistration, kMaxStateEntryCallbacks> onStateEntryCallbacks{};
};

#endif
