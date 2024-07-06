/*
 * Simple launch prediction algorithm
 * Simply, create an instance of this class and update it with
 * new acceleration data as often as possible.
 */

#ifndef LAUNCH_PREDICTOR_H
#define LAUNCH_PREDICTOR_H

#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"

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
class LaunchPredictor
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
    LaunchPredictor(float accelerationThreshold_ms2,
                    uint16_t windowSize_ms,
                    uint16_t windowInterval_ms);

    /**
     * Updates the predictor with new acceleration data
     * @param xac: The x acceleration data point in ms^2
     * @param yac: The y acceleration data point in ms^2
     * @param zac: The z acceleration data point in ms^2
     * @return: False if the data is ignored, true if the data is accepted
     */
    bool update(DataPoint xac, DataPoint yac, DataPoint zac);
    bool isLaunched() {return launched;}
    float getLaunchedTime() {return launchedTime_ms;}
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
 

private:
    // The threshold for acceleration to be considered a launch squared
    float accelerationThresholdSq_ms2;
    uint16_t windowInterval_ms;
    float tenPercentWindowInterval_ms;
    // The window holding the acceleration magnitude squared b/c sqrt is expensive
    CircularArray<DataPoint> AclMagSqWindow_ms2;
    bool launched;
    uint32_t launchedTime_ms;
};

#endif