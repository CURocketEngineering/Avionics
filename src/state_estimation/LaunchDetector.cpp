#include "state_estimation/LaunchDetector.h"
#include <cassert>

// #define DEBUG

#ifdef DEBUG
#include "ArduinoHAL.h"
#endif

namespace
{
uint8_t validateAndComputeWindowSize_slots(uint16_t windowSize_ms, uint16_t windowInterval_ms)
{
    assert(windowInterval_ms > 0U);

    const auto windowSize_slots = static_cast<uint16_t>(windowSize_ms / windowInterval_ms);
    assert(windowSize_slots >= 1U);
    assert(windowSize_slots <= static_cast<uint16_t>(kCircularArrayAllocatedSlots));

    return static_cast<uint8_t>(windowSize_slots);
}
} // namespace

LaunchDetector::LaunchDetector(float accelerationThreshold_ms2, //NOLINT(bugprone-easily-swappable-parameters)
                               uint16_t windowSize_ms,
                               uint16_t windowInterval_ms)
    : accelerationThresholdSq_ms2_(accelerationThreshold_ms2 * accelerationThreshold_ms2),
      windowInterval_ms_(windowInterval_ms),
      acceptableTimeDifference_ms_(static_cast<uint16_t>(static_cast<float>(windowInterval_ms) * kAcceptablePercentDifferenceWindowInterval)),
      accelMagnitudeSquaredWindow_(validateAndComputeWindowSize_slots(windowSize_ms, windowInterval_ms)),
      launched_(false),
      launchedTime_ms_(0),
      medianAccelerationSquared_(0)
{
    // These must remain here because they rely on accelMagnitudeSquaredWindow_ being constructed
    const uint16_t windowSpan_slots = static_cast<uint16_t>(accelMagnitudeSquaredWindow_.getMaxSize() - 1U);
    minWindowSize_ms_ = static_cast<uint16_t>(
        static_cast<uint32_t>(windowInterval_ms_ - acceptableTimeDifference_ms_) * windowSpan_slots);
    maxWindowSize_ms_ = static_cast<uint16_t>(
        static_cast<uint32_t>(windowInterval_ms_ + acceptableTimeDifference_ms_) * windowSpan_slots);
}


int LaunchDetector::update(AccelerationTriplet accel)
{
    // :xac: The x acceleration data point in ms^2
    // :yac: The y acceleration data point in ms^2
    // :zac: The z acceleration data point in ms^2 

    // Return false if the data is ignored, true if the data is accepted
    // Will set the launched flag if the data is accepted and the launch is detected


    // If launched, don't update
    if (launched_)
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
    if (time_ms < accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: Data point ignored because of time is earlier than head");
        Serial.printf("Incoming time: %lu\n", static_cast<unsigned long>(time_ms));
        Serial.printf("Head time: %lu\n", static_cast<unsigned long>(accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms));
        #endif
        return LP_YOUNGER_TIMESTAMP;
    }

    // If the window isn't full yet, just push the data point
    if (!accelMagnitudeSquaredWindow_.isFull())
    {
        #ifdef DEBUG
        // Serial.println("LaunchDetector: Populating initial window");
        #endif
        accelMagnitudeSquaredWindow_.push(DataPoint(time_ms, aclMagSq));
        
        return LP_INITIAL_POPULATION;
    }

    // Make sure we are near the window interval +- kAcceptablePercentDifferenceWindowInterval
    uint32_t timeDiff_ms = time_ms - accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms; //NOLINT(cppcoreguidelines-init-variables)
    const uint32_t minAllowedDiff_ms = static_cast<uint32_t>(windowInterval_ms_) - static_cast<uint32_t>(acceptableTimeDifference_ms_);
    const uint32_t maxAllowedDiff_ms = static_cast<uint32_t>(windowInterval_ms_) + static_cast<uint32_t>(acceptableTimeDifference_ms_);

    // Check that the data didn't come in too fast
    if (timeDiff_ms < minAllowedDiff_ms){
        #ifdef DEBUG
        Serial.println("LaunchDetector: DATA TOO EARLY");
        Serial.printf("Time diff: %lu\n", static_cast<unsigned long>(timeDiff_ms));
        Serial.printf("Window interval: %u\n", static_cast<unsigned int>(windowInterval_ms_));
        Serial.printf("Incoming time: %lu\n", static_cast<unsigned long>(time_ms));
        Serial.printf("Head time: %lu\n", static_cast<unsigned long>(accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms));
        #endif
        return LP_DATA_TOO_FAST;
    }

    if (timeDiff_ms > maxAllowedDiff_ms)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: DATA TOO LATE");
        Serial.printf("Time diff: %lu\n", static_cast<unsigned long>(timeDiff_ms));
        Serial.printf("Window interval: %u\n", static_cast<unsigned int>(windowInterval_ms_));
        Serial.printf("Incoming time: %lu\n", static_cast<unsigned long>(time_ms));
        Serial.printf("Head time: %lu\n", static_cast<unsigned long>(accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms));
        Serial.println("LaunchDetector: Clearing window");
        #endif
        
        accelMagnitudeSquaredWindow_.clear();
        return LP_WINDOW_DATA_STALE;
    }

    // Push the new data point
    #ifdef DEBUG
    Serial.print("LaunchDetector: Pushing timestamp: ");
    Serial.println(time_ms);
    #endif

    accelMagnitudeSquaredWindow_.push(DataPoint(time_ms, aclMagSq));

    const uint32_t headTimestamp_ms = accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms; //NOLINT(cppcoreguidelines-init-variables)
    const uint32_t tailTimestamp_ms = accelMagnitudeSquaredWindow_.getFromHead(accelMagnitudeSquaredWindow_.getMaxSize() - 1).timestamp_ms; //NOLINT(cppcoreguidelines-init-variables)
    const uint32_t timeRange_ms = headTimestamp_ms - tailTimestamp_ms;
    
    // Ensure the time_range isn't too small
    if (timeRange_ms < minWindowSize_ms_)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: Time range too small, waiting...");
        Serial.printf("Time range: %lu\n", static_cast<unsigned long>(timeRange_ms));
        Serial.printf("Min Time Range: %u\n", static_cast<unsigned int>(minWindowSize_ms_));
        Serial.printf("Incoming time: %lu\n", static_cast<unsigned long>(time_ms));
        Serial.printf("Head time: %lu\n", static_cast<unsigned long>(accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms));
        Serial.printf("Tail time: %lu\n", static_cast<unsigned long>(accelMagnitudeSquaredWindow_.getFromHead(accelMagnitudeSquaredWindow_.getMaxSize() - 1).timestamp_ms));
        #endif

        return LP_WINDOW_TIME_RANGE_TOO_SMALL;
    }

    // Ensure the time_range isn't too large
    if (timeRange_ms > maxWindowSize_ms_)
    {
        #ifdef DEBUG
        Serial.println("LaunchDetector: Time range too large, waiting...");
        Serial.printf("Time range: %lu\n", static_cast<unsigned long>(timeRange_ms));
        Serial.printf("Max Time Range: %u\n", static_cast<unsigned int>(maxWindowSize_ms_));
        Serial.printf("Incoming time: %lu\n", static_cast<unsigned long>(time_ms));
        Serial.printf("Head time: %lu\n", static_cast<unsigned long>(accelMagnitudeSquaredWindow_.getFromHead(0).timestamp_ms));
        Serial.printf("Tail time: %lu\n", static_cast<unsigned long>(accelMagnitudeSquaredWindow_.getFromHead(accelMagnitudeSquaredWindow_.getMaxSize() - 1).timestamp_ms));
        #endif

        return LP_WINDOW_TIME_RANGE_TOO_LARGE;
    }

    // Check that the window is full
    if (!accelMagnitudeSquaredWindow_.isFull())
    {
        return LP_WINDOW_NOT_FULL;
    }

    medianAccelerationSquared_ = accelMagnitudeSquaredWindow_.getMedian().data;

    // Check if the median is above the threshold
    if (medianAccelerationSquared_ > accelerationThresholdSq_ms2_)
    {
        launched_ = true;
        launchedTime_ms_ = time_ms;
        return LP_LAUNCH_DETECTED;
    }

    #ifdef DEBUG
    Serial.println("LaunchDetector: Median below threshold");
    // Print the median without being able to use %f because of the Arduino
    Serial.print("Median: ");
    Serial.println(medianAccelerationSquared_);
    Serial.print("Threshold: ");
    Serial.println(accelerationThresholdSq_ms2_);
    #endif
    return LP_ACL_TOO_LOW;
}

void LaunchDetector::reset()
{
    launched_ = false;
    launchedTime_ms_ = 0;

    // Clear the window
    accelMagnitudeSquaredWindow_.clear();
}
