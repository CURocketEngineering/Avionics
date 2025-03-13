#ifndef FLIGHT_STATE_MACHINE_H
#define FLIGHT_STATE_MACHINE_H

#include "state_estimation/States.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/LaunchPredictor.h"

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"


class StateMachine {
  public: 
    StateMachine(IDataSaver* dataSaver, LaunchPredictor* launchPredictor, ApogeeDetector* apogeeDetector);

    int update(DataPoint aclX, DataPoint aclY, DataPoint aclZ, DataPoint alt);

    uint8_t getState();

  private:
    uint8_t state;
    IDataSaver* dataSaver;
    LaunchPredictor* launchPredictor;
    ApogeeDetector* apogeeDetector;
};


#endif