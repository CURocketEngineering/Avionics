#ifndef FLIGHT_STATES_H
#define FLIGHT_STATES_H

#include <cstdint> // For uint8_t

// Not all states are used by all state machines
// Order matters, systems rely on comparisons like `current_state` > STATE_ASCENT to know if the 
// flight has passed the ascent state and is now falling. So states are ordered from earliest to latest.
// The literal value of these states should never be used because a new state can be added
// in between existing states, shifting their values. B/c we tag the version of Avionics used for each flight, 
// we can come back here to see the numerical values of each state when decoding the logs. The ground station
// keeps YAMLs of these state definitions every time they are changed. To avoid having a bunch of YAMLs we try
// to keep these values semi-stable, which is why it includes states that aren't currently used to be forward-compatible. 
// Keep in mind that there is a difference between states and events. For example,
// apogee is an event that causes a transition from STATE_ASCENT to STATE_DESCENT, but it is not a state itself.
// The state machines are in charge of detecting these transitions and updating the states. 
enum FlightState : uint8_t {
    STATE_UNARMED = 0,      // 0 This may not necessarily always be the 0th state, or the first state. A state could be added before it.
    STATE_ARMED,            // 1
    STATE_SOFT_ASCENT,      // 2
    STATE_ASCENT,           // 3 Don't use the ascent state if you are already using powered ascent and coast ascent
    STATE_POWERED_ASCENT,   // 4
    STATE_COAST_ASCENT,     // 5
    STATE_DESCENT,          // 6
    STATE_DROGUE_DEPLOYED,  // 7
    STATE_MAIN_DEPLOYED,    // 8
    STATE_LANDED,           // 9
};

#endif