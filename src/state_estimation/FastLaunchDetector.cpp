#include "state_estimation/FastLaunchDetector.h"

FastLaunchDetector::FastLaunchDetector(float accelerationThreshold_ms2)
    : accelerationThresholdSq_ms2(accelerationThreshold_ms2 * accelerationThreshold_ms2)
{}

int FastLaunchDetector::update(AccelerationTriplet accel){

    // Calculate the magnitude of the acceleration squared
    const float aclMagSq = accel.x.data * accel.x.data + accel.y.data * accel.y.data + accel.z.data * accel.z.data;

    // Take the average of the timestamps
    // Ideally these should all be the same
    const uint32_t time_ms = (accel.x.timestamp_ms + accel.y.timestamp_ms + accel.z.timestamp_ms) / 3;

    if (aclMagSq > accelerationThresholdSq_ms2){
        launched = true;
        launchedTime_ms = time_ms;
        return LP_LAUNCH_DETECTED;
    }
}