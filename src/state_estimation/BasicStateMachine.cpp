#include "ArduinoHAL.h"

#include "data_handling/DataNames.h"
#include "state_estimation/StateEstimationTypes.h"
#include "state_estimation/BasicStateMachine.h"


BasicStateMachine::BasicStateMachine(IDataSaver* dataSaver,
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

int BasicStateMachine::update(const AccelerationTriplet& accel, const DataPoint& alt) {
    // Update the state.
    switch (getFlightState()) {
        case STATE_ARMED:

            // Update launch detector and check for launch detection
            // As soon as this is true, jump straight to ascent, regardless of the FLD
            launchDetector_->update(accel);
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
                return 0;
            }


            // If the launch detector didn't trigger, check the fast launch detector
            // The fast launch detector should trigger before the launch detector, but it is more susceptible to noise.
            // This allows us to jump to a soft ascent while waiting for the launch detector to confirm.
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
                return 0;
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
                return 0;
            }
            if (accel.x.timestamp_ms - fldLaunchTime_ms_ > fastLaunchDetector_->getConfirmationWindow()) {
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
                return 0;
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
            return 0;

        case STATE_DESCENT:
            return 0; // Do nothing state

        default:
            // Unexpected state, error return 
            return 1;
    }

    return 0;
}
