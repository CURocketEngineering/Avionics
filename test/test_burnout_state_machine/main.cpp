#include <fstream>
#include <iostream>  // for error messages (if needed)
#include <random>

#include "unity.h"
#include "SimpleSimulation.h"   
#include "state_estimation/BurnoutStateMachine.h"
#include "data_handling/DataPoint.h"
#include "DataSaver_mock.h"
#include "state_estimation/States.h"
// #include "include/hal/serial_mock.h"
#include "ArduinoHAL.h"  // for serial

DataSaverMock dataSaver;
IDataSaver* dataSaverPtr = &dataSaver;

void test_burnout_state_machine_with_real_data(void);

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

//
// Main: Run all tests
//
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_burnout_state_machine_with_real_data);
    return UNITY_END();
}
