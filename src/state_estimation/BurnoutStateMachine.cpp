#include "state_estimation/BurnoutStateMachine.h"
#include "data_handling/DataNames.h"
#include "ArduinoHAL.h"

#define GRAVITY 9.8

BurnoutStateMachine::BurnoutStateMachine(IDataSaver* dataSaver, LaunchPredictor* launchPredictor, ApogeeDetector* apogeeDetector,
                                        VerticalVelocityEstimator* verticalVelocityEstimator) {
    this->dataSaver = dataSaver;
    this->launchPredictor = launchPredictor;
    this->apogeeDetector = apogeeDetector;
    this->verticalVelocityEstimator = verticalVelocityEstimator;
    this->state = STATE_ARMED;
}

int BurnoutStateMachine::update(DataPoint aclX, DataPoint aclY, DataPoint aclZ, DataPoint alt) {
    // Update the state
    int lpStatus = LP_DEFAULT_FAIL; 

    // Update the vertical velocity estimator
    verticalVelocityEstimator->update(aclX, aclY, aclZ, alt);
    
    switch (state) {
        case STATE_ARMED:
            // Serial.println("lp update");
            lpStatus = launchPredictor->update(aclX, aclY, aclZ);
            // Serial.println(lpStatus);
            if (launchPredictor->isLaunched()) {
                // Change state to ascent
                state = STATE_POWERED_ASCENT;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(aclX.timestamp_ms, STATE_POWERED_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver->launchDetected(launchPredictor->getLaunchedTime());
                
                // Start the apogee detection system
                apogeeDetector->init(alt.data, alt.timestamp_ms);
            }
            break;

        case STATE_POWERED_ASCENT:
            // Serial.println("apogee update");
            if (aclY.data <= GRAVITY) { // when acceleration returns to less than gravity after launch, we're coasting
                state = STATE_COAST_ASCENT;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(aclY.timestamp_ms, STATE_COAST_ASCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_COAST_ASCENT:
            // Serial.println("apogee update");
            if (apogeeDetector->isApogeeDetected()) {
                state = STATE_DESCENT;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(aclX.timestamp_ms, STATE_DESCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_DESCENT:
            // Do nothing
            break;
    }

    return 0;
}

uint8_t BurnoutStateMachine::getState() {
    return state;
}