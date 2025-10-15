#include "state_estimation/FastLaunchDetector.h"

// #define DEBUG
#ifdef DEBUG
#include "ArduinoHAL.h"
#endif

FastLaunchDetector::FastLaunchDetector(float accelerationThreshold_ms2)
    : accelerationThresholdSq_ms2(accelerationThreshold_ms2 * accelerationThreshold_ms2)
{}

int FastLaunchDetector::update(AccelerationTriplet accel){

    // Calculate the magnitude of the acceleration squared
    const float aclMagSq = accel.x.data * accel.x.data + accel.y.data * accel.y.data + accel.z.data * accel.z.data;

    // Take the average of the timestamps
    // Ideally these should all be the same
    const uint32_t time_ms = (accel.x.timestamp_ms + accel.y.timestamp_ms + accel.z.timestamp_ms) / 3;

    //if launch already detected, ignore further data
    if (launched){
        #ifdef DEBUG
        Serial.println("FastLaunchDetector: Data point ignored because already launched");
        #endif
        return ALREADY_LAUNCHED;
    }

    //if accel higher than threshold, launch detected
    if (aclMagSq > accelerationThresholdSq_ms2){
        launched = true;
        launchedTime_ms = time_ms;
        return LAUNCH_DETECTED;
    }

    //if accel lower than threshold, acl too low
    if (aclMagSq < accelerationThresholdSq_ms2) {
        #ifdef DEBUG
        Serial.println("FastLaunchDetector: Acceloration below threshold");
        #endif
        return ACL_TOO_LOW;
    }

}