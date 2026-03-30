#include "ArduinoHAL.h"

#include "data_handling/DataNames.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/StateMachine.h"


StateMachine::StateMachine(IDataSaver* dataSaver,
                           LaunchDetector* launchDetector,
                           ApogeeDetector* apogeeDetector,
                           VerticalVelocityEstimator* verticalVelocityEstimator,
                           FastLaunchDetector* fastLaunchDetector)
    : BaseStateMachine(STATE_ARMED),
      dataSaver_(dataSaver),
      launchDetector_(launchDetector),
      apogeeDetector_(apogeeDetector),
      verticalVelocityEstimator_(verticalVelocityEstimator),
      fastLaunchDetector_(fastLaunchDetector)
{
}

int StateMachine::update(const AccelerationTriplet& accel, const DataPoint& alt) {
    // Update the state.
    switch (getFlightState()) {
        case STATE_ARMED:
            // Serial.println("lp update");
            launchDetector_->update(accel);
            fastLaunchDetector_->update(accel);
            if (fastLaunchDetector_->hasLaunched()) {
                // Change state to soft ascent.
                changeState(STATE_SOFT_ASCENT);

                // Save the FLD launch time
                fldLaunchTime_ms_ = fastLaunchDetector_->getLaunchedTime();

                // Log the state change.
                dataSaver_->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_SOFT_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver_->launchDetected(fastLaunchDetector_->getLaunchedTime());
            }
            
            // The FLD should always trigger before the LP, but we check for LP launch just in case
            if (launchDetector_->isLaunched()) {
                // Change state to ascent.
                changeState(STATE_ASCENT);

                // Log the state change.
                dataSaver_->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver_->launchDetected(launchDetector_->getLaunchedTime());
                
                // Start the apogee detection system
                apogeeDetector_->init({alt.data, alt.timestamp_ms});

                // Update the vertical velocity estimator
                verticalVelocityEstimator_->update(accel, alt);
            }
            break;

        case STATE_SOFT_ASCENT:
        /*
        * In soft ascent, we are waiting for confirmation of launch from the LaunchDetector.
        * If LaunchDetector confirms launch within the confirmation window, we transition to ASCENT.
        * If the confirmation window passes without confirmation, we revert to ARMED
        * and clear post-launch mode.
        */
            // Serial.println("lp update");
            launchDetector_->update(accel);
            if (launchDetector_->isLaunched()) {
                // Change state to ascent.
                changeState(STATE_ASCENT);
                
                // Log the state change.
                dataSaver_->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_ASCENT),
                    STATE_CHANGE
                );

                // Start the apogee detection system
                apogeeDetector_->init({alt.data, alt.timestamp_ms});

                // Update the vertical velocity estimator
                verticalVelocityEstimator_->update(accel, alt);
            }
            else if (accel.x.timestamp_ms - fldLaunchTime_ms_ > fastLaunchDetector_->getConfirmationWindow()) {
                // If the confirmation window has passed without launch detected by LaunchDetector,
                // Revert to ARMED state.
                changeState(STATE_ARMED);
                fldLaunchTime_ms_ = 0;
                fastLaunchDetector_->reset();

                // Log the state change.
                dataSaver_->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_ARMED),
                    STATE_CHANGE
                );

                // Clear post-launch mode
                dataSaver_->clearPostLaunchMode();
            }
            break;
            
        case STATE_ASCENT:
            // Serial.println("apogee update");
            // Update the vertical velocity estimator
            verticalVelocityEstimator_->update(accel, alt);
            apogeeDetector_->update(verticalVelocityEstimator_);
            if (apogeeDetector_->isApogeeDetected()) {
                changeState(STATE_DESCENT);

                // Log the state change.
                dataSaver_->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_DESCENT),
                    STATE_CHANGE
                );
            }
            break;

        default:
            // All other states have no actions or transitions
            break;
    }

    return 0;
}
