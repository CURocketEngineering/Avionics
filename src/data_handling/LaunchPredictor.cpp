#include "data_handling/LaunchPredictor.h"

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

    this->min_window_size_ms = windowSize_ms - windowSize_ms * 0.1;
    this->max_window_size_ms = windowSize_ms + windowSize_ms * 0.1;

    launched = false;
    launchedTime_ms = 0;
    tenPercentWindowInterval_ms = windowInterval_ms * 0.1;
    median_acceleration_squared = 0;
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

        // If the window isn't full yet, just push the data point
    if (!AclMagSqWindow_ms2.isFull())
    {
        #ifdef DEBUG
        Serial.println("LaunchPredictor: Populating initial window");
        #endif
        AclMagSqWindow_ms2.push(DataPoint(time_ms, aclMagSq));
        
        return true;
    }

    // Make sure we are near the window interval +- 10%
    uint32_t time_diff = time_ms - AclMagSqWindow_ms2.getFromHead(0).timestamp_ms;

    if (time_diff < windowInterval_ms - tenPercentWindowInterval_ms || time_diff > windowInterval_ms + tenPercentWindowInterval_ms)
    {
        #ifdef DEBUG
        // Serial.println("LaunchPredictor: Data point ignored because of time difference");
        // Serial.printf("Time diff: %d\n", time_diff);
        // Serial.printf("Window interval: %d\n", windowInterval_ms);
        // Serial.printf("Incoming time: %d\n", time_ms);
        // Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        #endif

        // If the time_diff is greater than the window interval, we need to clear the window
        if (time_diff > max_window_size_ms)
        {   
            #ifdef DEBUG
            Serial.println("LaunchPredictor: Clearing window");
            #endif
            AclMagSqWindow_ms2.clear();
        }
        return false;
    }

    // Push the new data point
    AclMagSqWindow_ms2.push(DataPoint(time_ms, aclMagSq));

    uint32_t time_range = AclMagSqWindow_ms2.getFromHead(0).timestamp_ms - AclMagSqWindow_ms2.getFromHead(AclMagSqWindow_ms2.getMaxSize() - 1).timestamp_ms;
  
    // Ensure that the time range is within the window size +- 10%
    if (time_range < min_window_size_ms || time_range > max_window_size_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchPredictor: Time range out of acceptable range, not attempting to detect launch");
        Serial.printf("Time range  %d < %d < %d \n", min_window_size_ms, time_range, max_window_size_ms);
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        Serial.printf("Tail time: %d\n", AclMagSqWindow_ms2.getFromHead(AclMagSqWindow_ms2.getMaxSize() - 1).timestamp_ms);
        #endif
        return true;
    }

    // Check that the window is full
    if (!AclMagSqWindow_ms2.isFull())
    {
        return true;
    }

    this->median_acceleration_squared = AclMagSqWindow_ms2.getMedian().data;

    // Check if the median is above the threshold
    if (median_acceleration_squared > accelerationThresholdSq_ms2)
    {
        launched = true;
        launchedTime_ms = time_ms;
    } else {
        #ifdef DEBUG
        Serial.println("LaunchPredictor: Median below threshold");
        // Print the median without being able to use %f because of the Arduino
        Serial.print("Median: ");
        Serial.println(median_acceleration_squared);
        Serial.print("Threshold: ");
        Serial.println(accelerationThresholdSq_ms2);
        #endif
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