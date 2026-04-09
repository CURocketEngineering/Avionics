#define DEBUG
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include "unity.h"
#include "state_estimation/OrientationEstimator.h"
#include "data_handling/DataPoint.h"
#include "ArduinoHAL.h"
#include "state_estimation/StateEstimationTypes.h"
#include "../CSVMockData.h"

// Use a mock Serial for debug prints in tests
MockSerial Serial;

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

void test_orientation_estimator_with_real_data(void) {
    // Create CSV provider to read test data with 25Hz sample rate (40ms interval)
    CSVDataProvider provider("data/AA Data Collection - Second Launch Trimmed.csv", 25.0f);
    OrientationEstimator estimator;


    std::ofstream csv("orientation_test_output.csv");
    csv << "time,accelx,accely,accelz,gyrox,gyroy,gyroz,magx,magy,magz,roll,pitch,yaw\n";

    bool hasData = false;
    uint32_t lastTime = 0;
    uint32_t currentTime = 0;
    while (provider.hasNextDataPoint()) {
        hasData = true;
        SensorData data = provider.getNextDataPoint();

        // Create sensor triplets from the data point
        AccelerationTriplet accel = {
            DataPoint(data.time, data.accelx),
            DataPoint(data.time, data.accely),
            DataPoint(data.time, data.accelz)
        };
        GyroTriplet gyro = {
            DataPoint(data.time, data.gyrox),
            DataPoint(data.time, data.gyroy),
            DataPoint(data.time, data.gyroz)
        };
        MagTriplet mag = {
            DataPoint(data.time, data.magx),
            DataPoint(data.time, data.magy),
            DataPoint(data.time, data.magz)
        };

        // Update the orientation estimator with the new sensor data
        estimator.update(accel, gyro, mag, data.time);

        // Write results to CSV
        csv << data.time << ","
            << accel.x.data << "," << accel.y.data << "," << accel.z.data << ","
            << gyro.x.data << "," << gyro.y.data << "," << gyro.z.data << ","
            << mag.x.data << "," << mag.y.data << "," << mag.z.data << ","
            << estimator.getRoll() << "," 
            << estimator.getPitch() << "," 
            << estimator.getYaw() << "\n";

        TEST_ASSERT_TRUE(-180.0f <= estimator.getRoll() && estimator.getRoll() <= 180.0f);
        TEST_ASSERT_TRUE(-180.0f <= estimator.getPitch() && estimator.getPitch() <= 180.0f);
        TEST_ASSERT_TRUE(-180.0f <= estimator.getYaw() && estimator.getYaw() <= 180.0f);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_orientation_estimator_with_real_data);
    return UNITY_END();
}

