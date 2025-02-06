#include "state_estimation/StateMachine.h"
#include "data_handling/DataNames.h"
#include "ArduinoHAL.h"

StateMachine::StateMachine(IDataSaver* dataSaver, LaunchPredictor* launchPredictor, ApogeeDetector* apogeeDetector) {
    this->dataSaver = dataSaver;
    this->launchPredictor = launchPredictor;
    this->apogeeDetector = apogeeDetector;
    this->state = STATE_ARMED;
}

int StateMachine::update(DataPoint aclX, DataPoint aclY, DataPoint aclZ, DataPoint alt) {
    // Update the state
    switch (state) {
        case STATE_ARMED:
            // Serial.println("lp update");
            launchPredictor->update(aclX, aclY, aclZ);
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
            apogeeDetector->update(aclX, aclY, aclZ, alt);
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

uint16_t StateMachine::getState() {
    return state;
}