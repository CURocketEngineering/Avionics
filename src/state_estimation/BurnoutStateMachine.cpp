#include "ArduinoHAL.h"

#include "data_handling/DataNames.h"
#include "data_handling/DataPoint.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/BurnoutStateMachine.h"

constexpr float GRAVITY = 9.8;

uint32_t tempTimeStamp =0;
int count = 0;

BurnoutStateMachine::BurnoutStateMachine(IDataSaver* dataSaver,
                                         LaunchDetector* launchDetector,
                                         ApogeeDetector* apogeeDetector,
                                         VerticalVelocityEstimator* verticalVelocityEstimator)
    : dataSaver(dataSaver),
      launchDetector(launchDetector),
      apogeeDetector(apogeeDetector),
      verticalVelocityEstimator(verticalVelocityEstimator),
      state(STATE_ARMED)
{
}

int BurnoutStateMachine::update(AccelerationTriplet accel, DataPoint alt) {
    // Update the state
    int lpStatus = LP_DEFAULT_FAIL; 

    if(count == 0)
    {
        tempTimeStamp = accel.x.timestamp_ms;
        count++;
    }
    Serial.print("EA: ");
    Serial.println(verticalVelocityEstimator->getEstimatedAltitude());
    Serial.print("Ts: ");
    Serial.println(accel.x.timestamp_ms);

    switch (state) {
        case STATE_ARMED:
            // Serial.println("lp update");
            lpStatus = launchDetector->update(accel);
            // Serial.println(lpStatus);
            if (launchDetector->isLaunched()) {
                // Change state to ascent
                state = STATE_POWERED_ASCENT;

                // Log the state change
                Serial.println("To pa (launch detected)");
                Serial.print("PA timestamp: ");
                Serial.println(accel.x.timestamp_ms);
                dataSaver->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_POWERED_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver->launchDetected(launchDetector->getLaunchedTime());
                
                // Start the vertical velocity estimator
                verticalVelocityEstimator->update(accel, alt);

                // Start the apogee detection system
                apogeeDetector->init({alt.data, alt.timestamp_ms});
                 
            }
            break;

        case STATE_POWERED_ASCENT:
            // Serial.println("apogee update");
            verticalVelocityEstimator->update(accel, alt);
            apogeeDetector->update(verticalVelocityEstimator);
            Serial.println(verticalVelocityEstimator->getInertialVerticalAcceleration());
            if (verticalVelocityEstimator->getInertialVerticalAcceleration() <= 0) { // when acceleration returns to less than gravity after launch, we're coasting
                state = STATE_COAST_ASCENT;

                // Log the state change
                Serial.println("To ca");
                Serial.print("CA timestamp: ");
                Serial.println(accel.y.timestamp_ms);
                dataSaver->saveDataPoint(
                    DataPoint(accel.y.timestamp_ms, STATE_COAST_ASCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_COAST_ASCENT:
            verticalVelocityEstimator->update(accel, alt);
            apogeeDetector->update(verticalVelocityEstimator);
            if (apogeeDetector->isApogeeDetected()) {
                state = STATE_DESCENT;

                // Log the state change
                Serial.println("To descent");
                Serial.print("Descent timestamp: ");
                Serial.println(accel.x.timestamp_ms);
                dataSaver->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_DESCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_DESCENT:
            // Do nothing
            // Serial.println("in descent");
            Serial.print("Timestamp: ");
            Serial.println(tempTimeStamp);
            break;
    }

    return 0;
}

uint8_t BurnoutStateMachine::getState() const {
    return state;
}