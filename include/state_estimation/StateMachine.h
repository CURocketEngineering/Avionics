#ifndef FLIGHT_STATE_MACHINE_H
#define FLIGHT_STATE_MACHINE_H

#include "state_estimation/States.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/BaseStateMachine.h"

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

/**
 * @brief Nominal flight state machine using launch/apogee detection and VVE.
 * @note When to use: standard flights where launch->coast->descent transitions
 *       are driven by detectors and logging is desired at each change.
 */
class StateMachine : public BaseStateMachine {
  public: 
    /**
     * @brief Wire dependencies for the state machine.
     * @param dataSaver  Logger used to persist state changes.
     * @param launchDetector Launch detector instance.
     * @param apogeeDetector Apogee detector instance.
     * @param verticalVelocityEstimator Vertical velocity estimator instance.
     * @note When to use: build once during setup with already configured
     *       estimator/detector instances.
     */
    StateMachine(IDataSaver* dataSaver, LaunchDetector* launchDetector, ApogeeDetector* apogeeDetector, 
                 VerticalVelocityEstimator* verticalVelocityEstimator);

    /**
     * @brief Process new sensor data and transition states if thresholds are met.
     * @param accel Latest acceleration measurements.
     * @param alt   Latest altitude sample.
     * @note When to use: call on every control loop iteration.
     */
    int update(const AccelerationTriplet& accel, const DataPoint& alt) override;

    /**
     * @brief Retrieve the current state value.
     * @note When to use: downstream logic that needs to branch on flight phase.
     */
    uint8_t getState() const override;

  private:
    uint8_t state;
    IDataSaver* dataSaver;
    LaunchDetector* launchDetector;
    ApogeeDetector* apogeeDetector;
    VerticalVelocityEstimator* verticalVelocityEstimator;
};


#endif