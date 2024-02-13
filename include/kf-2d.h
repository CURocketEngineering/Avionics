#ifndef KF2D_H
#define KF2D_H

#include <linalg.h>  //matrix declaration is column major
#include <chrono>    //for keeping track of dt between updates
#include <Arduino.h> //serial printf
using namespace linalg::aliases;
using namespace linalg;
using namespace std::chrono;

class KF2D
{
public:
    // Derivables and measurables
    int n = 3; // Number of state variables, measurable or derivable by equations
    int m = 2; // Number of measurable variables by sensor //will be 3 with pitot or vin --> replace all 2's with 3's & measurementvector with float3

    float lastTime = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count(); // keep track of the last time in millis

    // Define the state vector (position, velocity, acceleration) and measurement vector (altitude, vertical velocity, vertical acceleration)
    typedef float3 StateVector;       // state variable x, 3 float vector
    typedef float2 MeasurementVector; // state measurement z, 2 float vector // will be 3 with pitot or vin

    StateVector x_hat; // Estimated state

    KF2D();                                                            // constructor
    void InitializeKalmanFilter(const MeasurementVector &measurement); // initialize, only needs to run once. can also be used to reset a KF. should be ran at
    void Predict();                                                    // should I also have a delta time predict?? or take average over time dt and use that
    void Update(const MeasurementVector &measurement);

private:
    // Define Kalman Filter matrices (P, A, H, R, Q)
    mat<float, 3, 3> P; // Estimate error covariance //n,n
    mat<float, 3, 3> A; // State transition matrix //n,n
    mat<float, 2, 3> H; // State to measurement matrix //m,n
    mat<float, 2, 2> R; // Measurement noise covariance matrix //m,m
    mat<float, 3, 3> Q; // Process noise covariance matrix //n,n
    float3 B;           // Control Variable transition matrix //ulen,n
    float uk;           // control variable "matrix"

    //debating removing the accel from the state matrix, maybe even adding the derivations and integrations for velocity as inputs and letting the system choose between state, deriv, integ, and eventually pitot

    // Phone: -144m +-12m (1030.74 hPa)
    // Altimeter: -147m
    // 3m off of phone, +-15m

    // i should really stop coding past midnight
    // if youre a future employer reading this commit hello hi
    // if youre a student reading this commit go read some better code or at least future commits where its hopefully better
};

#endif