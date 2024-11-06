/*
 * Simple apogee prediction algorithm
 * Simply, create an instance of this class and update it with
 * new acceleration data as often as possible.
 */

#ifndef LAUNCH_PREDICTOR_H
#define LAUNCH_PREDICTOR_H

#include "CircularArray.h"
#include "DataPoint.h"

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

class ApogeePredictor 
{
public:
    /**
     * Constructor
     * @param windowSize_ms: The size of the window to calculate the median acceleration.
     *                       The delay in launch detection will equal half the window size
     *                       A window too small will cause false positives.
     * @param windowInterval_ms: The interval between acceleration value in the window
     *                           You must be able to update the predictor faster than this interval
     *                           The predictor will reject data that comes in faster than this interval to maintain this interval or slower
     */
    ApogeePredictor(uint16_t windowSize_ms,
                    uint16_t windowInterval_ms,
                    uint16_t direction);
    /**
     * Updates the predictor with new acceleration data
     * @param xac: The x acceleration data point in ms^2
     * @param yac: The y acceleration data point in ms^2
     * @param zac: The z acceleration data point in ms^2
     * @return: False if the data is ignored, true if the data is accepted
     */
    bool update(DataPoint xac, DataPoint yac, DataPoint zac);
    bool isApogee() {return apogee;}
    float getApogeeTime() {return apogeeTime_ms;}
    void reset();
    float getVelocity() {return velocity;}
    // float timeToApogee();


private:
    // Is the rocket at apogee
    bool apogee;
    // The time the rocket reached apogee
    uint32_t apogeeTime_ms;
    // The threshold for acceleration to be considered to have reached apogee
    float accelerationThresholdSq_ms2;
    // Keeping track of the velocity using the acceleration data
    float velocity;
    CircularArray<DataPoint> VelocityWindow_ms2;
    uint16_t windowInterval_ms;
    uint16_t tenPercentWindowInterval_ms;
    CircularArray<DataPoint> AclMagSqWindow_ms2;
    uint16_t direction;
};









#endif