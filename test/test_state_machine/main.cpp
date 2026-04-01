#include "unity.h"
#include "SimpleSimulation.h"   
#include "state_estimation/StateMachine.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaverSPI.h"
#include "DataSaver_mock.h"
#include "state_estimation/States.h"
#include "ArduinoHAL.h"  // if required for your platform

// Forward declaration of CSV test
void test_state_machine_with_real_data(void);

#include <fstream>
#include <iostream>  // for error messages (if needed)
#include <random>

DataSaverMock dataSaver;
IDataSaver* dataSaverPtr = &dataSaver;

//
// setUp() and tearDown()
//
void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

void test_init(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    FastLaunchDetector fld(30, 500);
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve, &fld);

    TEST_ASSERT_EQUAL(STATE_ARMED, sm.getState());
}

void test_launch(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    FastLaunchDetector fld(30, 500);
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve, &fld);

    // Start sim
    SimpleSimulator sim(10000, 70, 3000, 10);

    while (sim.getApogeeTimestamp() == 0) {
        sim.tick();
        DataPoint aclX(sim.getCurrentTime(), 0);
        DataPoint aclY(sim.getCurrentTime(), 0);

        // Adding 9.8 because the launchDetector expects measured acceleration
        // I.e. 0m/s^2 stationary on ground is measured as 9.8m/s^2 by the accelerometer
        DataPoint aclZ(sim.getCurrentTime(), sim.getIntertialVerticalAcl() + 9.8f);
        DataPoint alt(sim.getCurrentTime(), sim.getAltitude());
        // std::cout << aclZ.data << "  mag: " << lp.getMedianAccelerationSquared() << " Ts: " << aclZ.timestamp_ms << std::endl;
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }

    // Check that the launchDetector detected the launch
    TEST_ASSERT_TRUE(lp.isLaunched());

    // Check that the lp was within 1s of the actual launch time
    TEST_ASSERT_UINT32_WITHIN(500, sim.getLaunchTimestamp(), lp.getLaunchedTime());

    // Check that the state machine state is greater than ARMED
    TEST_ASSERT_GREATER_THAN(STATE_ARMED, sm.getState());
}

void test_apogee_detection(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    FastLaunchDetector fld(30, 500);
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve, &fld);

    // Start sim
    SimpleSimulator sim(3000, 70, 2000, 5);

    // init noise for acl and alt
    std::mt19937 gen(42); 
    std::normal_distribution<float> aclNoise(0, 5);
    std::normal_distribution<float> altNoise(0, 5);

    while (!sim.getHasLanded()) {
        sim.tick();
        DataPoint aclX(sim.getCurrentTime(), 0);
        DataPoint aclY(sim.getCurrentTime(), 0);

        // Subtracting 9.8 because the launchDetector expects measured acceleration
        // I.e. 0m/s^2 stationary on ground is measured as -9.8m/s^2 by the accelerometer
        DataPoint aclZ(sim.getCurrentTime(), sim.getIntertialVerticalAcl() + 9.8f + aclNoise(gen));
        DataPoint alt(sim.getCurrentTime(), sim.getAltitude() + altNoise(gen));
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }

    // Check that the apogeeDetector detected the apogee
    TEST_ASSERT_TRUE(ad.isApogeeDetected());

    // Check that the state machine state is greater than ASCENT b/c apoge was detected and now descending
    TEST_ASSERT_GREATER_THAN(STATE_ASCENT, sm.getState());

    // Check that the ad was within .5s of the actual apogee time
    std::cout << "Sim apogee: " << sim.getApogeeTimestamp() << "  AD apogee: " << ad.getApogee().timestamp_ms << std::endl;
    TEST_ASSERT_UINT32_WITHIN(500, sim.getApogeeTimestamp(), ad.getApogee().timestamp_ms);
}

void test_apogee_detection_noise(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    FastLaunchDetector fld(30, 500);
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve, &fld);

    // Start sim
    SimpleSimulator sim(10000, 70, 3000, 10);

    while (!sim.getHasLanded()) {
        sim.tick();
        DataPoint aclX(sim.getCurrentTime(), 0);
        DataPoint aclY(sim.getCurrentTime(), 0);

        // Subtracting 9.8 because the launchDetector expects measured acceleration
        // I.e. 0m/s^2 stationary on ground is measured as -9.8m/s^2 by the accelerometer
        DataPoint aclZ(sim.getCurrentTime(), sim.getIntertialVerticalAcl() + 9.8f);
        DataPoint alt(sim.getCurrentTime(), sim.getAltitude());
        // std::cout << aclZ.data << "  mag: " << lp.getMedianAccelerationSquared() << " Ts: " << aclZ.timestamp_ms << std::endl;
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }

    // Check that the apogeeDetector detected the apogee
    TEST_ASSERT_TRUE(ad.isApogeeDetected());

    // Check that the state machine state is greater than ASCENT b/c apoge was detected and now descending
    TEST_ASSERT_GREATER_THAN(STATE_ASCENT, sm.getState());

    // Check that the ad was within 1s of the actual apogee time
    std::cout << "Sim apogee: " << sim.getApogeeTimestamp() << "  AD apogee: " << ad.getApogee().timestamp_ms << std::endl;
    TEST_ASSERT_UINT32_WITHIN(0, sim.getApogeeTimestamp(), ad.getApogee().timestamp_ms);
}

//Triggers fast launch detector then does not trigger launch detector during confirmation window
//should clear post launch flag 
void test_fast_launch_with_revert(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    FastLaunchDetector fld(30, 100);

    DataSaverSPI* dss;
    Adafruit_SPIFlash* flash;
    flash = new Adafruit_SPIFlash();
    dss = new DataSaverSPI(100, flash);

    StateMachine sm(dss, &lp, &ad, &vve, &fld);

    //feed stateMachine one point of acceleration data to trigger fld
    DataPoint fldaclX(0, 100);
    DataPoint fldaclY(0, 100);
    DataPoint fldaclZ(0, 100);
    DataPoint fldalt(0, 0);
    AccelerationTriplet fldaccel = {fldaclX, fldaclY, fldaclZ};
    sm.update(fldaccel, fldalt);

    // Check that the FastLaunchDetector detected a launch
    TEST_ASSERT_TRUE(fld.hasLaunched());

    // Check that data saver is temporarily in Post launch Mode
    TEST_ASSERT_TRUE(dss->quickGetPostLaunchMode());

    //feed 100 points of false data to revert fld after no confirmation
    for (int i = 0; i < 100; i++) {
        const uint32_t timestamp_ms = static_cast<uint32_t>(i) * 10U;
        DataPoint aclX(timestamp_ms, 0.0f);
        DataPoint aclY(timestamp_ms, 0.0f);
        DataPoint aclZ(timestamp_ms, 0.0f);
        DataPoint alt(timestamp_ms, 0.0f);
        // std::cout << aclZ.data << "  mag: " << lp.getMedianAccelerationSquared() << " Ts: " << aclZ.timestamp_ms << std::endl;
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }   
    // Check that the FastLaunchDetector reverted
    TEST_ASSERT_FALSE(fld.hasLaunched());

    // Check that the launchDetector did not detect a launc
    TEST_ASSERT_FALSE(lp.isLaunched());

    // Check that the state machine state is ARMED
    TEST_ASSERT_EQUAL(STATE_ARMED, sm.getState());

    // Check that data saver is not in post launch mode
    TEST_ASSERT_FALSE(dss->quickGetPostLaunchMode());

    delete dss;
    delete flash;
}

//Triggers fast launch detector then does trigger launch detector during confirmation window
//should not clear post launch flag, should be in ASCENT, Fld should detect launch sooner than lp
void test_fast_launch_with_confirm(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    FastLaunchDetector fld(30, 50000); //very high confirmation window

    DataSaverSPI* dss;
    Adafruit_SPIFlash* flash;
    flash = new Adafruit_SPIFlash();
    dss = new DataSaverSPI(100, flash);

    StateMachine sm(dss, &lp, &ad, &vve, &fld);

    // Start sim
    SimpleSimulator sim(10000, 70, 3000, 10);

    while (sim.getApogeeTimestamp() == 0) {
        sim.tick();
        DataPoint aclX(sim.getCurrentTime(), 0);
        DataPoint aclY(sim.getCurrentTime(), 0);

        // Adding 9.8 because the launchDetector expects measured acceleration
        // I.e. 0m/s^2 stationary on ground is measured as 9.8m/s^2 by the accelerometer
        DataPoint aclZ(sim.getCurrentTime(), sim.getIntertialVerticalAcl() + 9.8f);
        DataPoint alt(sim.getCurrentTime(), sim.getAltitude());
        // std::cout << aclZ.data << "  mag: " << lp.getMedianAccelerationSquared() << " Ts: " << aclZ.timestamp_ms << std::endl;
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }

    // Check that the FastLaunchDetector detected the launch
    TEST_ASSERT_TRUE(fld.hasLaunched());

    // Check that the launchDetector detected the launch
    TEST_ASSERT_TRUE(lp.isLaunched());

    // Check that the lp was within 1s of the actual launch time
    TEST_ASSERT_UINT32_WITHIN(500, sim.getLaunchTimestamp(), lp.getLaunchedTime());

    // Check that the state machine state is greater than ARMED
    TEST_ASSERT_GREATER_THAN(STATE_ARMED, sm.getState());

    // Check that the state machine state is greater than SOFT_ASCENT
    TEST_ASSERT_GREATER_THAN(STATE_SOFT_ASCENT, sm.getState());

    // Check that LD launch time is greater than FLD launch time
    TEST_ASSERT_GREATER_THAN(fld.getLaunchedTime(), lp.getLaunchedTime());

    // Check that data saver is in post launch mode
    TEST_ASSERT_TRUE(dss->quickGetPostLaunchMode());

    delete dss;
    delete flash;
}

//
// Main: Run all tests
//
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init);
    RUN_TEST(test_launch);
    RUN_TEST(test_apogee_detection);
    RUN_TEST(test_state_machine_with_real_data);
    RUN_TEST(test_fast_launch_with_revert);
    RUN_TEST(test_fast_launch_with_confirm);
    return UNITY_END();
}
