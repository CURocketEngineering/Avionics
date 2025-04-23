#include "state_estimation/BurnoutStateMachine.h"
#include "data_handling/DataNames.h"
#include "ArduinoHAL.h"

#define GRAVITY 9.8

int ts =0;
int count = 0;

BurnoutStateMachine::BurnoutStateMachine(IDataSaver* dataSaver, LaunchDetector* launchDetector, ApogeeDetector* apogeeDetector,
                                        VerticalVelocityEstimator* verticalVelocityEstimator) {
    this->dataSaver = dataSaver;
    this->launchDetector = launchDetector;
    this->apogeeDetector = apogeeDetector;
    this->verticalVelocityEstimator = verticalVelocityEstimator;
    this->state = STATE_ARMED;
}

int BurnoutStateMachine::update(DataPoint aclX, DataPoint aclY, DataPoint aclZ, DataPoint alt) {
    // Update the state
    int lpStatus = LP_DEFAULT_FAIL; 

    if(count == 0)
    {
        ts = aclX.timestamp_ms;
        count++;
    }
    Serial.print("EA: ");
    Serial.println(verticalVelocityEstimator->getEstimatedAltitude());
    Serial.print("Ts: ");
    Serial.println(aclX.timestamp_ms);

    switch (state) {
        case STATE_ARMED:
            // Serial.println("lp update");
            lpStatus = launchDetector->update(aclX, aclY, aclZ);
            // Serial.println(lpStatus);
            if (launchDetector->isLaunched()) {
                // Change state to ascent
                state = STATE_POWERED_ASCENT;

                // Log the state change
                Serial.println("To pa (launch detected)");
                Serial.print("PA timestamp: ");
                Serial.println(aclX.timestamp_ms);
                dataSaver->saveDataPoint(
                    DataPoint(aclX.timestamp_ms, STATE_POWERED_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver->launchDetected(launchDetector->getLaunchedTime());
                
                // Start the vertical velocity estimator
                verticalVelocityEstimator->update(aclX, aclY, aclZ, alt);

                // Start the apogee detection system
                apogeeDetector->init(alt.data, alt.timestamp_ms);
                 
            }
            break;

        case STATE_POWERED_ASCENT:
            // Serial.println("apogee update");
            verticalVelocityEstimator->update(aclX, aclY, aclZ, alt);
            apogeeDetector->update(verticalVelocityEstimator);
            Serial.println(verticalVelocityEstimator->getInertialVerticalAcceleration());
            if (verticalVelocityEstimator->getInertialVerticalAcceleration() <= 0) { // when acceleration returns to less than gravity after launch, we're coasting
                state = STATE_COAST_ASCENT;

                // Log the state change
                Serial.println("To ca");
                Serial.print("CA timestamp: ");
                Serial.println(aclY.timestamp_ms);
                dataSaver->saveDataPoint(
                    DataPoint(aclY.timestamp_ms, STATE_COAST_ASCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_COAST_ASCENT:
            verticalVelocityEstimator->update(aclX, aclY, aclZ, alt);
            apogeeDetector->update(verticalVelocityEstimator);
            if (apogeeDetector->isApogeeDetected()) {
                state = STATE_DESCENT;

                // Log the state change
                Serial.println("To descent");
                Serial.print("Descent timestamp: ");
                Serial.println(aclX.timestamp_ms);
                dataSaver->saveDataPoint(
                    DataPoint(aclX.timestamp_ms, STATE_DESCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_DESCENT:
            // Do nothing
            // Serial.println("in descent");
            Serial.print("Timestamp: ");
            Serial.println(ts);
            break;
    }

    return 0;
}

uint8_t BurnoutStateMachine::getState() {
    return state;
}