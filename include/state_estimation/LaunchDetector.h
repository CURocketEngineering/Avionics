/*
 * Simple launch prediction algorithm
 * Simply, create an instance of this class and update it with
 * new acceleration data as often as possible.
 */

#ifndef LAUNCH_DETECTOR_H
#define LAUNCH_DETECTOR_H

#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"

constexpr float kAcceptablePercentDifferenceWindowInterval = 0.5F;
constexpr std::size_t kCircularArrayAllocatedSlots = 100; // 100 slots allocated for the circular array (100 * sizeof(DataPoint)) = 800 bytes allocated)

// Potential returns from the update function
// Positive values are errors
// Negative values are warnings
enum LaunchDetectorStatus {
    LP_LAUNCH_DETECTED = 0,
    LP_ALREADY_LAUNCHED = -1,
    LP_YOUNGER_TIMESTAMP = -2, // The timestamp is younger than the last timestamp
    LP_INITIAL_POPULATION = -3, // The window is not full yet
    LP_DATA_TOO_FAST = -4, // The data came in faster than the desired window
    LP_WINDOW_DATA_STALE = 1, // The data given makes the window data relatively too old, resets the window
    LP_WINDOW_TIME_RANGE_TOO_SMALL = -5, // The window is full, but the time difference between the head and the tail is too little
    LP_WINDOW_TIME_RANGE_TOO_LARGE = -6, // The window is full, but the time difference between the head and the tail is too large
    LP_WINDOW_NOT_FULL = -7, // The window is not full yet
    LP_ACL_TOO_LOW = -8, // The acceleration is too low for launch
    LP_DEFAULT_FAIL = 2,
};

/**
 * Predicts launch by looking for a sustained acceleration above a threshold
 * It takes the median acceleration magnitude over a window of time and compares it
 * to the threshold. Because of the size of the window, short spikes are ignored.
 *
 * The window is a circular array / rolling window of acceleration magnitudes squared
 *
 * The delay in launch detection will equal half the window size because
 * the median will only be high once half the window is high
 */
/**
 * @brief Sliding-window launch detector based on acceleration magnitude.
 * @note When to use: detect liftoff robustly against spikes by requiring a
 *       sustained acceleration median over a configurable window.
 */
class LaunchDetector
{
public:
    /**
     * Constructor
     * @param accelerationThreshold_ms2: The threshold for acceleration to be considered a launch
     * @param windowSize_ms: The size of the window to calculate the median acceleration.
     *                       The delay in launch detection will equal half the window size
     *                       A window too small will cause false positives.
     * @param windowInterval_ms: The interval between acceleration value in the window
     *                           You must be able to update the detector faster than this interval
     *                           The detector will reject data that comes in faster than this interval to maintain this interval or slower
     */
    LaunchDetector(float accelerationThreshold_ms2,
                    uint16_t windowSize_ms,
                    uint16_t windowInterval_ms);

    /**
     * Updates the detector with new acceleration data
     * @param accel: The newest acceleration data (triplet of x, y, and z acceleration data points)
     * @return: False if the data is ignored, true if the data is accepted
     */
    int update(AccelerationTriplet accel);
    bool isLaunched() {return launched_;}
    uint32_t getLaunchedTime() {return launchedTime_ms_;}
    float getMedianAccelerationSquared() {return medianAccelerationSquared_;}
    void reset();

    // --------------
    // Testing Methods
    // --------------
    // Gives a pointer to the window
    CircularArray<DataPoint, kCircularArrayAllocatedSlots>* getWindowPtr() {return &accelMagnitudeSquaredWindow_;}
    // Gives the threshold in ms^2 squared
    float getThreshold() {return accelerationThresholdSq_ms2_;}
    // Gives the window interval in ms
    uint16_t getWindowInterval() {return windowInterval_ms_;}
    uint16_t getAcceptableTimeDifference() {return acceptableTimeDifference_ms_;}
 

private:
    // The threshold for acceleration to be considered a launch squared
    float accelerationThresholdSq_ms2_;
    uint16_t windowInterval_ms_;

    // Min and max window sizes calculated based on the window interval and the acceptable time difference
    uint16_t minWindowSize_ms_ = 0; // If the calculated time range is less than this, don't try to detect launch
    uint16_t maxWindowSize_ms_ = 0; // If the calculated time range is greater than this, don't try to detect launch

    uint16_t acceptableTimeDifference_ms_;
    // The window holding the acceleration magnitude squared b/c sqrt is expensive
    CircularArray<DataPoint, kCircularArrayAllocatedSlots> accelMagnitudeSquaredWindow_;
    bool launched_;
    uint32_t launchedTime_ms_;

    float medianAccelerationSquared_;
};

#endif
