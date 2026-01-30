#ifndef FLIGHT_STATES_H
#define FLIGHT_STATES_H

// Not all states are used by all state machines
// Order matters, systems rely on comparisons like `current_state` > STATE_ASCENT to know if the 
// flight has passed the ascent state and is now falling. So states are ordered from earliest to latest.
// The literal value of these states should never be used because a new state can be added
// in between existing states, shifting their values.
// Keep in mind that there is a difference between states and events. For example,
// apogee is an event that causes a transition from STATE_ASCENT to STATE_DESCENT, but it is not a state itself.
// The state machines are in charge of detecting these transitions and updating the states. 
enum FlightState {
    STATE_UNARMED,
    STATE_ARMED,
    STATE_SOFT_ASCENT,
    STATE_ASCENT, // Don't use the ascent state if you are already using powered ascent and coast ascent
    STATE_POWERED_ASCENT,
    STATE_COAST_ASCENT,
    STATE_DESCENT,
    STATE_DROGUE_DEPLOYED,
    STATE_MAIN_DEPLOYED,
    STATE_LANDED,
};

#endif