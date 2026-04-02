#define DEBUG 
#include "unity.h"
#include "state_estimation/FastLaunchDetector.h"
#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"
#include "ArduinoHAL.h" 
#include "state_estimation/StateEstimationTypes.h"

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

/*
    Test that once launch is detected, further updates are ignored
*/
void test_already_launched(void){
    FastLaunchDetector fld(10.0f);
    TEST_ASSERT_FALSE(fld.hasLaunched());
    //give point that exceeds threshold
    DataPoint dp1(1000U, 100.0f);
    AccelerationTriplet accel1 = { dp1, dp1, dp1 };
    fld.update(accel1);
    TEST_ASSERT_TRUE(fld.hasLaunched());
    //new update
    int ret = fld.update(accel1);
    TEST_ASSERT_EQUAL_INT(FLD_ALREADY_LAUNCHED, ret);
}

/*
    Test that resetting fld will reset launched and launchtime vars
*/
void test_reset(void){
    FastLaunchDetector fld(10.0f);
    //first update with value above threshold
    DataPoint dp1(1000U, 100.0f);
    AccelerationTriplet accel1 = { dp1, dp1, dp1 };
    fld.update(accel1);
    TEST_ASSERT_TRUE(fld.hasLaunched());

    //then reset and check
    fld.reset();
    TEST_ASSERT_FALSE(fld.hasLaunched());
    TEST_ASSERT_EQUAL_UINT32(0, fld.getLaunchedTime());
}

/**
 * Test that switching from below threshold to above threshold in subsequent updates
 * leads to launch detection.
 */
void test_acceleration_edge_case(void) {
    FastLaunchDetector fld(10.0f);
    // First update with value just below threshold.
    DataPoint dp1(1000U, 9.9f);
    DataPoint dp2(1000U, 0.0f);     
    AccelerationTriplet accel1 = { dp1, dp2, dp2 };
    fld.update(accel1); // Total MAG is only 9.9^2 = 98.01 which is less than 100
    TEST_ASSERT_FALSE(fld.hasLaunched());
    
    // Then update with value just above threshold.
    DataPoint dp3(1000U, 10.1f);     
    AccelerationTriplet accel2 = { dp3, dp2, dp2 }; 
    fld.update(accel2);// Total MAG is 10.1^2 = 102.01 which is more than 100
    TEST_ASSERT_TRUE(fld.hasLaunched());
}

/**
 * Test that when the accel is above the threshold the launch is detected.
 */
void test_acceleration_above_threshold(void) {
    FastLaunchDetector fld(10.0f);
    // update with high acceleration values.
    DataPoint dp1(1000U, 100.0f);
    AccelerationTriplet accel1 = { dp1, dp1, dp1 };
    int result = fld.update(accel1);
    TEST_ASSERT_EQUAL_INT(FLD_LAUNCH_DETECTED, result);

    // acceleration squared is much higher than 10^2.
    TEST_ASSERT_TRUE(fld.hasLaunched());
    TEST_ASSERT_NOT_EQUAL(0, fld.getLaunchedTime());
}

/**
 * Test that when the accel is below the threshold, a launch is not detected
 */
void test_acceleration_below_threshold(void) {
    // Use a threshold of 10 m/s^2; squared threshold = 100.
    FastLaunchDetector fld(10.0f);
    // update with low acceleration value.
    DataPoint dp1(1000U, 1.0f);
    AccelerationTriplet accel1 = { dp1, dp1, dp1 };
    fld.update(accel1);
    // [1,1,1] results in acceleration squared = 1^2+1^2+1^2 = 3
    TEST_ASSERT_FALSE(fld.hasLaunched());
}

int main(){
    UNITY_BEGIN();
    RUN_TEST(test_already_launched);
    RUN_TEST(test_reset);
    RUN_TEST(test_acceleration_edge_case);
    RUN_TEST(test_acceleration_above_threshold);
    RUN_TEST(test_acceleration_below_threshold);
    return UNITY_END();
}
