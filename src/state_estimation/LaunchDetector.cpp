#include "state_estimation/LaunchDetector.h"

// #define DEBUG

#ifdef DEBUG
#include "ArduinoHAL.h"
#endif

LaunchDetector::LaunchDetector(float accelerationThreshold_ms2, //NOLINT(bugprone-easily-swappable-parameters)
                               uint16_t windowSize_ms,
                               uint16_t windowInterval_ms)
    : AclMagSqWindow_ms2(windowSize_ms / windowInterval_ms),
      accelerationThresholdSq_ms2(accelerationThreshold_ms2 * accelerationThreshold_ms2),
      windowInterval_ms(windowInterval_ms),
      launched(false),
      launchedTime_ms(0),
      acceptableTimeDifference_ms(static_cast<uint16_t>(static_cast<float>(windowInterval_ms) * ACCEPTABLE_PERCENT_DIFFERENCE_WINDOW_INTERVAL)),
      median_acceleration_squared(0)
{
    // These must remain here because they rely on AclMagSqWindow_ms2 being constructed
    min_window_size_ms = (windowInterval_ms - acceptableTimeDifference_ms) *
                         (AclMagSqWindow_ms2.getMaxSize() - 1);
    max_window_size_ms = (windowInterval_ms + acceptableTimeDifference_ms) *
                         (AclMagSqWindow_ms2.getMaxSize() - 1);
}


int LaunchDetector::update(AccelerationTriplet accel)
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
        Serial.println("LaunchDetector: Data point ignored because already launched");
        #endif
        return LP_ALREADY_LAUNCHED;
    }
    // Calculate the magnitude of the acceleration squared
    const float aclMagSq = accel.x.data * accel.x.data + accel.y.data * accel.y.data + accel.z.data * accel.z.data;

    // Take the average of the timestamps
    // Ideally these should all be the same
    const uint32_t time_ms = (accel.x.timestamp_ms + accel.y.timestamp_ms + accel.z.timestamp_ms) / 3;

    // Making sure the new time is greater than the last time
    if (time_ms < AclMagSqWindow_ms2.getFromHead(0).timestamp_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: Data point ignored because of time is earlier than head");
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        #endif
        return LP_YOUNGER_TIMESTAMP;
    }

    // If the window isn't full yet, just push the data point
    if (!AclMagSqWindow_ms2.isFull())
    {
        #ifdef DEBUG
        // Serial.println("LaunchDetector: Populating initial window");
        #endif
        AclMagSqWindow_ms2.push(DataPoint(time_ms, aclMagSq));
        
        return LP_INITIAL_POPULATION;
    }

    // Make sure we are near the window interval +- 10%
    uint32_t time_diff = time_ms - AclMagSqWindow_ms2.getFromHead(0).timestamp_ms; //NOLINT(cppcoreguidelines-init-variables)

    // Check that the data didn't come in too fast
    if (time_diff < windowInterval_ms - acceptableTimeDifference_ms){
        #ifdef DEBUG
        Serial.println("LaunchDetector: DATA TOO EARLY");
        Serial.printf("Time diff: %d\n", time_diff);
        Serial.printf("Window interval: %d\n", windowInterval_ms);
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        #endif
        return LP_DATA_TOO_FAST;
    }

    if (time_diff > windowInterval_ms + acceptableTimeDifference_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: DATA TOO LATE");
        Serial.printf("Time diff: %d\n", time_diff);
        Serial.printf("Window interval: %d\n", windowInterval_ms);
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        Serial.println("LaunchDetector: Clearing window");
        #endif
        
        AclMagSqWindow_ms2.clear();
        return LP_WINDOW_DATA_STALE;
    }

    // Push the new data point
    #ifdef DEBUG
    Serial.print("LaunchDetector: Pushing timestamp: ");
    Serial.println(time_ms);
    #endif

    AclMagSqWindow_ms2.push(DataPoint(time_ms, aclMagSq));

    const uint32_t head_timeStamp_ms = AclMagSqWindow_ms2.getFromHead(0).timestamp_ms; //NOLINT(cppcoreguidelines-init-variables)
    const uint32_t tail_timeStamp_ms = AclMagSqWindow_ms2.getFromHead(AclMagSqWindow_ms2.getMaxSize() - 1).timestamp_ms; //NOLINT(cppcoreguidelines-init-variables)
    const uint32_t time_range = head_timeStamp_ms - tail_timeStamp_ms;
    
    // Ensure the time_range isn't too small
    if (time_range < min_window_size_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: Time range too small, waiting...");
        Serial.printf("Time range: %d\n", time_range);
        Serial.printf("Min Time Range: %d\n", min_window_size_ms);
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        Serial.printf("Tail time: %d\n", AclMagSqWindow_ms2.getFromHead(AclMagSqWindow_ms2.getMaxSize() - 1).timestamp_ms);
        #endif

        return LP_WINDOW_TIME_RANGE_TOO_SMALL;
    }

    // Ensure the time_range isn't too large
    if (time_range > max_window_size_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: Time range too large, waiting...");
        Serial.printf("Time range: %d\n", time_range);
        Serial.printf("Max Time Range: %d\n", max_window_size_ms);
        Serial.printf("Incoming time: %d\n", time_ms);
        Serial.printf("Head time: %d\n", AclMagSqWindow_ms2.getFromHead(0).timestamp_ms);
        Serial.printf("Tail time: %d\n", AclMagSqWindow_ms2.getFromHead(AclMagSqWindow_ms2.getMaxSize() - 1).timestamp_ms);
        #endif

        return LP_WINDOW_TIME_RANGE_TOO_LARGE;
    }

    // Check that the window is full
    if (!AclMagSqWindow_ms2.isFull())
    {
        return LP_WINDOW_NOT_FULL;
    }

    this->median_acceleration_squared = AclMagSqWindow_ms2.getMedian().data;

    // Check if the median is above the threshold
    if (median_acceleration_squared > accelerationThresholdSq_ms2)
    {
        launched = true;
        launchedTime_ms = time_ms;
        return LP_LAUNCH_DETECTED;
    }

    #ifdef DEBUG
    Serial.println("LaunchDetector: Median below threshold");
    // Print the median without being able to use %f because of the Arduino
    Serial.print("Median: ");
    Serial.println(median_acceleration_squared);
    Serial.print("Threshold: ");
    Serial.println(accelerationThresholdSq_ms2);
    #endif
    return LP_ACL_TOO_LOW;
}

void LaunchDetector::reset()
{
    launched = false;
    launchedTime_ms = 0;

    // Clear the window
    AclMagSqWindow_ms2.clear();
}