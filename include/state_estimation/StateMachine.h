#ifndef FLIGHT_STATE_MACHINE_H
#define FLIGHT_STATE_MACHINE_H

#include "state_estimation/States.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/BaseStateMachine.h"

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"


class StateMachine : public BaseStateMachine {
  public: 
    StateMachine(IDataSaver* dataSaver, LaunchDetector* launchDetector, ApogeeDetector* apogeeDetector, 
                 VerticalVelocityEstimator* verticalVelocityEstimator);

    int update(const AccelerationTriplet& accel, const DataPoint& alt) override;

    uint8_t getState() const override;

  private:
    uint8_t state;
    IDataSaver* dataSaver;
    LaunchDetector* launchDetector;
    ApogeeDetector* apogeeDetector;
    VerticalVelocityEstimator* verticalVelocityEstimator;
};


#endif