#ifndef FLIGHT_STATES_H
#define FLIGHT_STATES_H

enum FlightState {
    STATE_UNARMED,
    STATE_ARMED,
    STATE_ASCENT,
    STATE_POWERED_ASCENT,
    STATE_COAST_ASCENT,
    STATE_DESCENT,
    STATE_DROGUE_DEPLOYED,
    STATE_MAIN_DEPLOYED,
    STATE_LANDED,
    STATE_SOFT_ASCENT // for FastLaunchDetector, will transition to STATE_ASCENT once confirmed by LaunchDetector
};

#endif