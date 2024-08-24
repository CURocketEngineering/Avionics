#include "data_handling/LaunchPredictor.h"

#include <iostream>

#ifdef DEBUG
#include "ArduinoHAL.h"
#endif

LaunchPredictor::LaunchPredictor(float accelerationThreshold_ms2,
                                 uint16_t windowSize_ms,
                                 uint16_t windowInterval_ms)
    : AclMagSqWindow_ms2(windowSize_ms / windowInterval_ms)
{

    accelerationThresholdSq_ms2 = accelerationThreshold_ms2 * accelerationThreshold_ms2;
    this->windowInterval_ms = windowInterval_ms;

    launched = false;
    launchedTime_ms = 0;
    tenPercentWindowInterval_ms = windowInterval_ms * 0.1;
}

bool LaunchPredictor::update(DataPoint xac, DataPoint yac, DataPoint zac)
{
    // :xac: The x acceleration data point in ms^2
    // :yac: The y acceleration data point in ms^2
    // :zac: The z acceleration data point in ms^2 

    // Return false if the data is ignored, true if the data is accepted
    // Will set the launched flag if the data is accepted and the launch is detected


    // If launched, don't update
    if (launched)
    {
        #ifdef DEBUG
        Serial.println("LaunchPredictor: Data point ignored because already launched");
        #endif
        return false;
    }
    // Calculate the magnitude of the acceleration squared
    float aclMagSq = xac.data * xac.data + yac.data * yac.data + zac.data * zac.data;

    // Take the average of the timestamps
    // Ideally these should all be the same
    uint32_t time_ms = (xac.timestamp_ms + yac.timestamp_ms + zac.timestamp_ms) / 3;

    // Making sure the new time is greater than the last time
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

    // Push the new data point
    AclMagSqWindow_ms2.push(DataPoint(time_ms, aclMagSq));

    // Check that the window is full
    if (!AclMagSqWindow_ms2.isFull())
    {
        return true;
    }

    // Check if the median is above the threshold
    if (AclMagSqWindow_ms2.getMedian().data > accelerationThresholdSq_ms2)
    {
        launched = true;
        launchedTime_ms = time_ms;
    }

    return true;
}

void LaunchPredictor::reset()
{
    launched = false;
    launchedTime_ms = 0;

    // Clear the window
    AclMagSqWindow_ms2.clear();
}