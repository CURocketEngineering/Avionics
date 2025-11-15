#ifndef FAST_LAUNCH_DETECTOR_H
#define FAST_LAUNCH_DETECTOR_H
 // need to figure out how to impliment checking for launch from LaunchDetector
#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"

// Potential returns from the update function
// Positive values are errors
// Negative values are warnings
enum FastLaunchDetectorStatus {
    FLD_LAUNCH_DETECTED = 0,
    FLD_ALREADY_LAUNCHED = -1,
    FLD_ACL_TOO_LOW = -2, // The acceleration is too low for launch
    FLD_DEFAULT_FAIL = 2,
};


class FastLaunchDetector
{
public:
    /**
    * Constructor
    * @param accelerationThreshold_ms2: The threshold for acceleration to be considered a launch
    * @param confirmationWindow_ms: The time window in ms to confirm the launch using LaunchDetector
    */
    FastLaunchDetector(float accelerationThreshold, uint32_t confirmationWindow_ms = 500);

    int update(AccelerationTriplet accel);

    bool hasLaunched() const { return launched; }
    uint32_t getLaunchedTime() const { return launchedTime_ms; }
    uint32_t getConfirmationWindow() const { return confirmationWindow_ms; }
    void reset();

private:
    float accelerationThresholdSq_ms2;

    bool launched;
    uint32_t launchedTime_ms;
    uint32_t confirmationWindow_ms;
};


#endif