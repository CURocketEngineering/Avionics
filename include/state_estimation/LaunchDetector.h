/*
 * Simple launch prediction algorithm
 * Simply, create an instance of this class and update it with
 * new acceleration data as often as possible.
 */

#ifndef LAUNCH_DETECTOR_H
#define LAUNCH_DETECTOR_H

#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"

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
     *                           You must be able to update the predictor faster than this interval
     *                           The predictor will reject data that comes in faster than this interval to maintain this interval or slower
     */
    LaunchDetector(float accelerationThreshold_ms2,
                    uint16_t windowSize_ms,
                    uint16_t windowInterval_ms);

    /**
     * Updates the predictor with new acceleration data
     * @param xac: The x acceleration data point in ms^2
     * @param yac: The y acceleration data point in ms^2
     * @param zac: The z acceleration data point in ms^2
     * @return: False if the data is ignored, true if the data is accepted
     */
    int update(DataPoint xac, DataPoint yac, DataPoint zac);
    bool isLaunched() {return launched;}
    float getLaunchedTime() {return launchedTime_ms;}
    float getMedianAccelerationSquared() {return median_acceleration_squared;}
    void reset();

    // --------------
    // Testing Methods
    // --------------
    // Gives a pointer to the window
    CircularArray<DataPoint>* getWindowPtr() {return &AclMagSqWindow_ms2;}
    // Gives the threshold in ms^2 squared
    float getThreshold() {return accelerationThresholdSq_ms2;}
    // Gives the window interval in ms
    uint16_t getWindowInterval() {return windowInterval_ms;}
    uint16_t getAcceptableTimeDifference() {return acceptableTimeDifference_ms;}
 

private:
    // The threshold for acceleration to be considered a launch squared
    float accelerationThresholdSq_ms2;
    uint16_t windowInterval_ms;

    // Min and max window sizes calculated as +- 10% of the window size
    uint16_t min_window_size_ms; // If the calculated time range is less than this, don't try to detect launch
    uint16_t max_window_size_ms; // If the calculated time range is greater than this, don't try to detect launch

    float acceptableTimeDifference_ms;
    // The window holding the acceleration magnitude squared b/c sqrt is expensive
    CircularArray<DataPoint> AclMagSqWindow_ms2;
    bool launched;
    uint32_t launchedTime_ms;

    float median_acceleration_squared;
};

#endif