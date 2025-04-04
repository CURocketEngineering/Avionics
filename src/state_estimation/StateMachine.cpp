#include "state_estimation/StateMachine.h"
#include "data_handling/DataNames.h"
#include "ArduinoHAL.h"

StateMachine::StateMachine(IDataSaver* dataSaver, LaunchPredictor* launchPredictor, ApogeeDetector* apogeeDetector, 
                           VerticalVelocityEstimator* verticalVelocityEstimator) {
    this->dataSaver = dataSaver;
    this->launchPredictor = launchPredictor;
    this->apogeeDetector = apogeeDetector;
    this->verticalVelocityEstimator = verticalVelocityEstimator;
    this->state = STATE_ARMED;
}

int StateMachine::update(DataPoint aclX, DataPoint aclY, DataPoint aclZ, DataPoint alt) {
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
                state = STATE_ASCENT;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(aclX.timestamp_ms, STATE_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver->launchDetected(launchPredictor->getLaunchedTime());
                
                // Start the apogee detection system
                apogeeDetector->init(alt.data, alt.timestamp_ms);
            }
            break;
        case STATE_ASCENT:
            // Serial.println("apogee update");
            apogeeDetector->update(verticalVelocityEstimator);
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

uint8_t StateMachine::getState() {
    return state;
}