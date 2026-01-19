#ifndef BASM_STATE_MACHINE_H
#define BASM_STATE_MACHINE_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/States.h"
#include "state_estimation/BaseStateMachine.h"

/**
 * @brief State machine variant that explicitly models motor burnout before coast.
 * @note When to use: flights requiring a distinct burnout phase separate from
 *       powered ascent and coast, originally designed for aerobrake testing.
 */
class BurnoutStateMachine : public BaseStateMachine {
  public:
    /**
     * @brief Construct with logging and detector dependencies.
     * @param dataSaver  Logger used to persist transitions.
     * @param launchDetector Launch detector instance.
     * @param apogeeDetector Apogee detector instance.
     * @param verticalVelocityEstimator Vertical velocity estimator instance.
     * @note When to use: setup time wiring of components prior to loop.
     */
    BurnoutStateMachine(IDataSaver* dataSaver, LaunchDetector* launchDetector, ApogeeDetector* apogeeDetector,
      VerticalVelocityEstimator* verticalVelocityEstimator);

    /**
     * @brief Update machine with new sensor inputs and transition on burnout cues.
     * @param accel Latest acceleration readings.
     * @param alt   Latest altitude sample.
     * @note When to use: call each sensor update; return value can reflect
     *       transition outcomes.
     */
    int update(const AccelerationTriplet& accel, const DataPoint& alt) override;

    /**
     * @brief Current state identifier.
     * @note When to use: consumers needing current phase (e.g., pyro logic).
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