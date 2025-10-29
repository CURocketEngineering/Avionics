#include "ArduinoHAL.h"

#include "data_handling/DataNames.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/StateMachine.h"


StateMachine::StateMachine(IDataSaver* dataSaver,
                           LaunchDetector* launchDetector,
                           ApogeeDetector* apogeeDetector,
                           VerticalVelocityEstimator* verticalVelocityEstimator,
                           FastLaunchDetector* fastLaunchDetector)
    : dataSaver(dataSaver),
      launchDetector(launchDetector),
      apogeeDetector(apogeeDetector),
      verticalVelocityEstimator(verticalVelocityEstimator),
      fastLaunchDetector(fastLaunchDetector),
      state(STATE_ARMED)
{
}

int StateMachine::update(const AccelerationTriplet& accel, const DataPoint& alt) {
    // Update the state
    int lpStatus = LP_DEFAULT_FAIL; 
    int fldStatus = FLD_DEFAULT_FAIL;

    switch (state) {
        case STATE_ARMED:
            // Serial.println("lp update");
            lpStatus = launchDetector->update(accel);
            fldStatus = fastLaunchDetector->update(accel);
            // Serial.println(lpStatus);
            if (launchDetector->isLaunched()) {
                // Change state to ascent
                state = STATE_ASCENT;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver->launchDetected(launchDetector->getLaunchedTime());
                
                // Start the apogee detection system
                apogeeDetector->init({alt.data, alt.timestamp_ms});

                // Update the vertical velocity estimator
                verticalVelocityEstimator->update(accel, alt);
            }
            if (fastLaunchDetector->hasLaunched()) {
                // Change state to soft ascent
                state = STATE_SOFT_ASCENT;

                // Save the FLD launch time
                fldLaunchTime_ms = fastLaunchDetector->getLaunchedTime();

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_SOFT_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver->launchDetected(fastLaunchDetector->getLaunchedTime());
            }
            break;
        case STATE_ASCENT:
            // Serial.println("apogee update");
            // Update the vertical velocity estimator
            verticalVelocityEstimator->update(accel, alt);
            apogeeDetector->update(verticalVelocityEstimator);
            if (apogeeDetector->isApogeeDetected()) {
                state = STATE_DESCENT;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_DESCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_SOFT_ASCENT:
            // Serial.println("lp update");
            lpStatus = launchDetector->update(accel);
            // Serial.println(lpStatus);
            if (launchDetector->isLaunched()) {
                // Change state to ascent
                state = STATE_ASCENT;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_ASCENT),
                    STATE_CHANGE
                );

                // Start the apogee detection system
                apogeeDetector->init({alt.data, alt.timestamp_ms});

                // Update the vertical velocity estimator
                verticalVelocityEstimator->update(accel, alt);
            }
            if (accel.x.timestamp_ms - fldLaunchTime_ms > fastLaunchDetector->getConfirmationWindow()) {
                // If the confirmation window has passed without launch detected by LaunchDetector,
                // revert to ARMED state
                state = STATE_ARMED;

                // Log the state change
                dataSaver->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_ARMED),
                    STATE_CHANGE
                );

                // Clear post-launch mode
                dataSaver->clearPostLaunchMode();
            }
            break;

        case STATE_DESCENT:
            // Do nothing
            break;
    }

    return 0;
}

uint8_t StateMachine::getState() const {
    return state;
}