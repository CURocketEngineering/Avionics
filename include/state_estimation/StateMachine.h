#ifndef FLIGHT_STATE_MACHINE_H
#define FLIGHT_STATE_MACHINE_H

#include "state_estimation/States.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"


class StateMachine {
  public: 
    StateMachine(IDataSaver* dataSaver, LaunchDetector* launchDetector, ApogeeDetector* apogeeDetector, 
                 VerticalVelocityEstimator* verticalVelocityEstimator);

    int update(AccelerationTriplet accel, DataPoint alt);

    uint8_t getState() const;

  private:
    uint8_t state;
    IDataSaver* dataSaver;
    LaunchDetector* launchDetector;
    ApogeeDetector* apogeeDetector;
    VerticalVelocityEstimator* verticalVelocityEstimator;
};


#endif