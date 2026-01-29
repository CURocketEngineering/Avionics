# state_estimation Directory

Tools for detecting flight events, fusing sensors, and managing rocket flight-state transitions.

## Files
- `ApogeeDetector.h`: Detects apogee when filtered altitude peaks and velocity goes negative. More robust than zero-velocity crossing, especially with noisy baro data.
- `ApogeePredictor.h`: Projects time/altitude to apogee using current velocity and deceleration; use for active-aero or adaptive control while still climbing.
- `BaseStateMachine.h`: Abstract interface for flight state machines.
- `BurnoutStateMachine.h`: State machine variant with an explicit burnout phase before coast; use when burnout-specific logic or logging matters.
- `GroundLevelEstimator.h`: Learns launch-site altitude pre-launch, then converts ASL to AGL after launch; use to normalize baro data.
- `LaunchDetector.h`: Sliding-window accelerometer detector that marks liftoff when sustained acceleration exceeds a threshold; use to gate launch-critical events.
- `StateEstimationTypes.h`: Shared data structures (e.g., `AccelerationTriplet`) passed among estimators and state machines.
- `StateMachine.h`: Nominal flight state machine that advances through phases using launch/apogee detectors and logs transitions.
- `States.h`: Enum of discrete flight states used across state machines they are all ordered from sequentially (earliest to latest) but not all states are used by all state machines but if STATE_A > STATE_B then STATE_A always occurs after STATE_B.
- `VerticalVelocityEstimator.h`: 1D Kalman filter fusing accelerometer and barometer to estimate altitude, vertical velocity, and inertial acceleration; feed its outputs to detectors and state machines.
