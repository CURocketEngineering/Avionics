#include "../../include/data_handling/ApogeePredictor.h"

#include <iostream>
#include <cmath>

#ifdef DEBUG
#include "ArduinoHAL.h"
#endif

ApogeePredictor::ApogeePredictor(float accelerationThreshold_ms2,
                                 uint16_t windowSize_ms,
                                 uint16_t windowInterval_ms,
                                 uint16_t direction)
    : AclMagSqWindow_ms2(windowSize_ms / windowInterval_ms), VelocityWindow_ms2(windowSize_ms / windowInterval_ms)
{
    accelerationThresholdSq_ms2 = accelerationThreshold_ms2 * accelerationThreshold_ms2;
    this->windowInterval_ms = windowInterval_ms;
    this->direction = direction;
    apogee = false;
    apogeeTime_ms = 0;
    velocity = 0;
    tenPercentWindowInterval_ms = windowInterval_ms * 0.1;
}

bool ApogeePredictor::update(DataPoint xac, DataPoint yac, DataPoint zac)
{
    // :xac: The x acceleration data point in ms^2
    // :yac: The y acceleration data point in ms^2
    // :zac: The z acceleration data point in ms^2 

    // Return false if the data is ignored, true if the data is accepted
    // Will set the apogee flag if the data is accepted and the apogee is detected
    if (apogee)
    {
        #ifdef DEBUG
        Serial.println("ApogeePredictor: Data point ignored because already at apogee");
        #endif
        return false;
    }

    // Calculate the magnitude of the acceleration squared
    // May only want to use the z acceleration as y and x could move around at apogee
    float aclMagSq = xac.data * xac.data + yac.data * yac.data + zac.data * zac.data;
    float accel = direction == 0 ? xac.data : direction == 1 ? yac.data : zac.data;

    // Take the average of the timestamps
    // Ideally these should all be the same
    uint32_t time_ms = (xac.timestamp_ms + yac.timestamp_ms + zac.timestamp_ms) / 3;

    // Making sure the new time is greater than the last time
    // Potentially could be a problem if the timestamps are not in sequenital order and we don't care
    if (time_ms < AclMagSqWindow_ms2.getFromHead(0).timestamp_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchPredictor: Data point ignored because of time is earlier than head");
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        #endif
        return false;
    }

    // Make sure we are near the window interval +- 10%
    // Again if we want to ignore doing time windowing we can remove this
    // Maybe get a system so that time can't change by that must be does not have to be linear
    uint32_t time_diff = time_ms - AclMagSqWindow_ms2.getFromHead(0).timestamp_ms;

    if (time_diff < windowInterval_ms - tenPercentWindowInterval_ms || time_diff > windowInterval_ms + tenPercentWindowInterval_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchPredictor: Data point ignored because of time difference");
        Serial.printf("Time diff: %d\n", time_diff);
        Serial.printf("Window interval: %d\n", windowInterval_ms);
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        #endif
        return false;
    }

    // Add the new data point to the window
    AclMagSqWindow_ms2.push(DataPoint(time_ms, aclMagSq));
    velocity = VelocityWindow_ms2.getMedian().data + accel * (windowInterval_ms / 1000);
    VelocityWindow_ms2.push(DataPoint(time_ms, velocity));
    // Check that the window is full
    if (!AclMagSqWindow_ms2.isFull())
    {
        return true;
    }

    // Maybe should be if the acceleration is 0 but more testing needs to be done
    // or velocity is 0
    if (VelocityWindow_ms2.getMedian().data == 0) {
        #ifdef DEBUG
        Serial.println("ApogeePredictor: Velocity is 0");
        Serial.println("Current time: %d", time_ms);
        Serial.println("Current accel: %f", accel);
        #endif
    }

    // Check if the median is equal to 0, acceleration is 0
    if (AclMagSqWindow_ms2.getMedian().data == 0)
    {
        #ifdef DEBUG
        Serial.println("ApogeePredictor: acceleration is 0");
        Serial.println("Current time: %d", time_ms);
        Serial.println("Current velocity: %f", velocity);
        #endif
        apogee = true;
        apogeeTime_ms = time_ms;
    }

    return true;

}

void ApogeePredictor::reset()
{
    apogee = false;
    apogeeTime_ms = 0;
    velocity = 0;

    // Clear the window
    AclMagSqWindow_ms2.clear();
}

// float ApogeePredictor::timeToApogee()