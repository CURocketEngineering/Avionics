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
    LAUNCH_DETECTED = 0,
    ALREADY_LAUNCHED = -1,
    ACL_TOO_LOW = -2, // The acceleration is too low for launch
    DEFAULT_FAIL = 2,
};


class FastLaunchDetector
{
public:
    /**
    * Constructor
    * @param accelerationThreshold_ms2: The threshold for acceleration to be considered a launch
    */
    FastLaunchDetector(float accelerationThreshold);

    int update(AccelerationTriplet accel);

    bool hasLaunched() const { return launched; }
    uint32_t getLaunchedTime() const { return launchedTime_ms; }
    void reset();

private:
    float accelerationThresholdSq_ms2;

    bool launched;
    uint32_t launchedTime_ms;
};


#endif