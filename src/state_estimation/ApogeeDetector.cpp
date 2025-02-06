#include "state_estimation/ApogeeDetector.h"
#include <cmath>

// Constructor
ApogeeDetector::ApogeeDetector(float accelNoiseVar, float altimeterNoiseVar, float apogeeThresh)
: state_alt(0.0f),
  state_vel(0.0f),
  lastTimestamp(0),
  initialized(false),
  apogee_flag(false),
  accelNoiseVariance(accelNoiseVar),
  altimeterNoiseVariance(altimeterNoiseVar),
  apogeeThreshold(apogeeThresh),
  maxAltitude(0.0f),
  maxAltitudeTimestamp(0)
{
    // Initialize the covariance matrix P with moderate initial uncertainty.
    P[0][0] = 1.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 1.0f;
}

// Initialize the filter state
void ApogeeDetector::init(float initialAltitude, uint32_t initialTimestamp) {
    state_alt = initialAltitude;
    state_vel = 0.0f;
    lastTimestamp = initialTimestamp;
    initialized = true;
    apogee_flag = false;
    verticalAxisDetermined = false;
    verticalAxis = 0;
    verticalDirection = 0;

    maxAltitude = initialAltitude;
    maxAltitudeTimestamp = initialTimestamp;
    // Reset covariance matrix
    P[0][0] = 1.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 1.0f;
}

void ApogeeDetector::determineVerticalAxis(const float rawAcl[3]){
    float mag[3] = { fabsf(rawAcl[0]), fabsf(rawAcl[1]), fabsf(rawAcl[2]) };
    
    // Find index of max absolute acceleration
    verticalAxis = 0; // X axis
    if (mag[1] > mag[verticalAxis]) verticalAxis = 1; // Y axis
    if (mag[2] > mag[verticalAxis]) verticalAxis = 2; // Z axis
    
    // Set the direction of the vertical axis
    verticalDirection = (rawAcl[verticalAxis] > 0) ? 1 : -1;
}

// Update the filter with new sensor data.
void ApogeeDetector::update(const DataPoint &accelX, const DataPoint &accelY, const DataPoint &accelZ, const DataPoint &altimeter) {
    // Use the altimeter timestamp for the update.
    uint32_t currentTimestamp = altimeter.timestamp_ms;
    if (!initialized) {
        init(altimeter.data, currentTimestamp);
        return;
    }

    float rawAcl[3] = { accelX.data, accelY.data, accelZ.data };

    if (!verticalAxisDetermined){
        determineVerticalAxis(rawAcl);
        verticalAxisDetermined = true;
    }
    
    // Compute time step (dt in seconds). If timestamps are the same, use a small default dt.
    float dt = (currentTimestamp > lastTimestamp) ? (currentTimestamp - lastTimestamp) / 1000.0f : 0.01f;
    
    // Use the vertical acceleration measurement.
    // (Assuming accelZ.data is in m/s²: at rest it is about +9.81 m/s², in free fall it is ~0.)
    // To obtain the inertial acceleration (i.e. the acceleration of the rocket) we subtract gravity.

    intertialVerticalAccleration = rawAcl[verticalAxis] * verticalDirection - g;
    
    // --- Prediction step ---
    // State transition model:
    //   predicted_alt = state_alt + state_vel*dt + 0.5*dt²*u
    //   predicted_vel = state_vel + dt*u
    float predicted_alt = state_alt + state_vel * dt + 0.5f * dt * dt * intertialVerticalAccleration;
    float predicted_vel = state_vel + dt * intertialVerticalAccleration;
    
    // Process noise covariance Q (based on acceleration noise)
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float Q00 = 0.25f * dt4 * accelNoiseVariance;
    float Q01 = 0.5f * dt3 * accelNoiseVariance;
    float Q10 = 0.5f * dt3 * accelNoiseVariance;
    float Q11 = dt2 * accelNoiseVariance;
    
    // Predicted covariance matrix P':
    //   P' = F*P*F^T + Q, where F = [ [1, dt], [0, 1] ]
    float P00 = P[0][0] + 2 * dt * P[0][1] + dt2 * P[1][1] + Q00;
    float P01 = P[0][1] + dt * P[1][1] + Q01;
    float P10 = P01; // symmetry
    float P11 = P[1][1] + Q11;
    
    // --- Measurement update ---
    // Measurement matrix H = [1, 0]. Altimeter measurement z:
    float z = altimeter.data;
    // Innovation (residual)
    float y = z - predicted_alt;
    // Innovation covariance: S = H*P'*H^T + R = P00 + altimeterNoiseVariance
    float S = P00 + altimeterNoiseVariance;
    // Kalman gain K = P'*H^T / S
    float K0 = P00 / S;
    float K1 = P10 / S;
    
    // Updated state estimate
    state_alt = predicted_alt + K0 * y;
    state_vel = predicted_vel + K1 * y;
    
    // Updated covariance matrix: P = (I - K*H) * P'
    P[0][0] = (1 - K0) * P00;
    P[0][1] = (1 - K0) * P01;
    P[1][0] = P10 - K1 * P00;
    P[1][1] = P11 - K1 * P01;
    
    // Update last timestamp
    lastTimestamp = currentTimestamp;
    
    // Update maximum altitude seen so far.
    if (state_alt > maxAltitude) {
        maxAltitude = state_alt;
        maxAltitudeTimestamp = currentTimestamp;
    }
    
    // Apogee detection: if the current estimated altitude is at least apogeeThreshold
    // below the maximum and the estimated velocity is negative, mark apogee.
    if (!apogee_flag && ((maxAltitude - state_alt) >= apogeeThreshold) && (state_vel < 0)) {
        apogee_flag = true;
        // (Optionally, you could “snap” the altitude to maxAltitude here.)
    }
}

// Return true if apogee has been detected.
bool ApogeeDetector::isApogeeDetected() const {
    return apogee_flag;
}

// Get the detected apogee as a DataPoint.
// If apogee has not been detected, returns a DataPoint with timestamp 0 and altitude 0.
DataPoint ApogeeDetector::getApogee() const {
    if (apogee_flag) {
        return DataPoint(maxAltitudeTimestamp, maxAltitude);
    }
    return DataPoint(0, 0.0f);
}

// Return the current estimated altitude.
float ApogeeDetector::getEstimatedAltitude() const {
    return state_alt;
}

// Return the current estimated vertical velocity.
float ApogeeDetector::getEstimatedVelocity() const {
    return state_vel;
}

float ApogeeDetector::getInertialVerticalAcceleration() const {
    return intertialVerticalAccleration;
}

int8_t ApogeeDetector::getVerticalAxis() const {
    return verticalAxis;
}

int8_t ApogeeDetector::getVerticalDirection() const {
    return verticalDirection;
}
