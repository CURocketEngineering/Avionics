#ifndef BASM_STATE_MACHINE_H
#define BASM_STATE_MACHINE_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/States.h"
#include "state_estimation/BaseStateMachine.h"

class BurnoutStateMachine : public BaseStateMachine {
  public:
    BurnoutStateMachine(IDataSaver* dataSaver, LaunchDetector* launchDetector, ApogeeDetector* apogeeDetector,
      VerticalVelocityEstimator* verticalVelocityEstimator);

    int update(AccelerationTriplet accel, DataPoint alt) override;

    uint8_t getState() const override;

  private:
    uint8_t state;
    IDataSaver* dataSaver;
    LaunchDetector* launchDetector;
    ApogeeDetector* apogeeDetector;
    VerticalVelocityEstimator* verticalVelocityEstimator;
};


#endif