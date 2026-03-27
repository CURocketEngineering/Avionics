#include "ArduinoHAL.h"

#include "data_handling/DataNames.h"
#include "data_handling/DataPoint.h"
#include "state_estimation/BurnoutStateMachine.h"
#include "state_estimation/StateEstimationTypes.h"

BurnoutStateMachine::BurnoutStateMachine(IDataSaver* dataSaver,
                                         LaunchDetector* launchDetector,
                                         ApogeeDetector* apogeeDetector,
                                         VerticalVelocityEstimator* verticalVelocityEstimator)
    : dataSaver_(dataSaver),
      launchDetector_(launchDetector),
      apogeeDetector_(apogeeDetector),
      verticalVelocityEstimator_(verticalVelocityEstimator),
      state_(STATE_ARMED)
{
}

int BurnoutStateMachine::update(const AccelerationTriplet& accel, const DataPoint& alt) {
    switch (state_) {
        case STATE_ARMED:
            // Serial.println("lp update");
            launchDetector_->update(accel);
            // Serial.println(lpStatus);
            if (launchDetector_->isLaunched()) {
                // Change state to ascent.
                state_ = STATE_POWERED_ASCENT;

                // Log the state change.
                Serial.println("To pa (launch detected)");
                Serial.print("PA timestamp: ");
                Serial.println(accel.x.timestamp_ms);
                dataSaver_->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_POWERED_ASCENT),
                    STATE_CHANGE
                );

                // Put the data saver into post-launch mode
                dataSaver_->launchDetected(launchDetector_->getLaunchedTime());
                
                // Start the vertical velocity estimator
                verticalVelocityEstimator_->update(accel, alt);

                // Start the apogee detection system
                apogeeDetector_->init({alt.data, alt.timestamp_ms});
                 
            }
            break;

        case STATE_POWERED_ASCENT:
            // Serial.println("apogee update");
            verticalVelocityEstimator_->update(accel, alt);
            apogeeDetector_->update(verticalVelocityEstimator_);
            Serial.println(verticalVelocityEstimator_->getInertialVerticalAcceleration());
            if (verticalVelocityEstimator_->getInertialVerticalAcceleration() <= 0) { // when acceleration returns to less than gravity after launch, we're coasting
                state_ = STATE_COAST_ASCENT;

                // Log the state change.
                Serial.println("To ca");
                Serial.print("CA timestamp: ");
                Serial.println(accel.y.timestamp_ms);
                dataSaver_->saveDataPoint(
                    DataPoint(accel.y.timestamp_ms, STATE_COAST_ASCENT),
                    STATE_CHANGE
                );
            }
            break;

        case STATE_COAST_ASCENT:
            verticalVelocityEstimator_->update(accel, alt);
            apogeeDetector_->update(verticalVelocityEstimator_);
            if (apogeeDetector_->isApogeeDetected()) {
                state_ = STATE_DESCENT;

                // Log the state change.
                Serial.println("To descent");
                Serial.print("Descent timestamp: ");
                Serial.println(accel.x.timestamp_ms);
                dataSaver_->saveDataPoint(
                    DataPoint(accel.x.timestamp_ms, STATE_DESCENT),
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

uint8_t BurnoutStateMachine::getState() const {
    return state_;
}
