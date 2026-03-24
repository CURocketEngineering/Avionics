#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include "unity.h"
#include "state_estimation/StateMachine.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "data_handling/DataPoint.h"
#include "DataSaver_mock.h"
#include "../CSVMockData.h"

/**
 * Test the StateMachine using real flight data from CSV.
 * This test reads sensor data from a CSV file, feeds it into the state machine,
 * and logs the results including launch and apogee predictions.
 */
void test_state_machine_with_real_data(void) {
    // Create CSV provider to read test data with 25Hz sample rate (40ms interval)
    CSVDataProvider provider("data/MARTHA_3-8_1.3_B2_SingleID_transformed.csv", 25.0f);
    
    // Initialize components
    LaunchDetector lp(30, 1000, 40);  // threshold, window, min samples
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    DataSaverMock dataSaver;
    FastLaunchDetector fld (30, 500);
    StateMachine sm(&dataSaver, &lp, &ad, &vve, &fld);
    
    // Create output CSV file for analysis
    std::ofstream outputFile("state_machine_results.csv");
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open output CSV file" << std::endl;
        TEST_FAIL();
        return;
    }
    
    // Write CSV header
    outputFile << "timestamp,accelX,accelY,accelZ,raw_altitude,state,launch_predicted,apogee_predicted,estimated_altitude,estimated_velocity\n";
    
    bool hasData = false;
    float maxAltitude = -1000.0f;
    uint32_t maxAltitudeTime = 0;
    uint32_t launchTime = 0;
    
    // Process each data point at 25Hz
    while (provider.hasNextDataPoint()) {
        hasData = true;
        SensorData data = provider.getNextDataPoint();
        
        // Create DataPoints for the state machine
        DataPoint accelX(data.time, data.accelx);
        DataPoint accelY(data.time, data.accely);
        DataPoint accelZ(data.time, data.accelz);
        DataPoint alt(data.time, data.altitude);

        AccelerationTriplet accel = {accelX, accelY, accelZ};
        
        // Update state machine with sensor data
        sm.update(accel, alt);
        
        // Track maximum altitude for validation
        if (data.altitude > maxAltitude) {
            maxAltitude = data.altitude;
            maxAltitudeTime = data.time;
        }
        
        // Track launch time
        if (lp.isLaunched() && launchTime == 0) {
            launchTime = lp.getLaunchedTime();
        }
        
        // Write results to CSV
        // Only output apogee predictions after launch is detected
        float estAlt = 0.0f;
        float estVel = 0.0f;
        if (sm.getState() >= STATE_ASCENT) {
            estAlt = vve.getEstimatedAltitude();
            estVel = vve.getEstimatedVelocity();
        }
        
        outputFile << data.time << ","
                 << data.accelx << ","
                  << data.accely << ","
                  << data.accelz << ","
                  << data.altitude << ","
                  << sm.getState() << ","
                  << (lp.isLaunched() ? "1" : "0") << ","
                  << (ad.isApogeeDetected() ? "1" : "0") << ","
                  << estAlt << ","
                  << estVel << "\n";
    }
    
    outputFile.close();
    
    // Verify test data was processed
    TEST_ASSERT_TRUE(hasData);
    
    // Verify launch was detected
    TEST_ASSERT_TRUE(lp.isLaunched());
    
    // Verify apogee was detected
    TEST_ASSERT_TRUE(ad.isApogeeDetected());
    
    // Verify state progression
    TEST_ASSERT_GREATER_THAN(STATE_ARMED, sm.getState());
    
    // Get detected apogee information
    DataPoint detectedApogee = ad.getApogee();
    
    // Verify apogee altitude is reasonable (within 10% of max recorded altitude)
    float altitudeDifference = std::abs(detectedApogee.data - maxAltitude);
    float allowedError = maxAltitude * 0.10f;
    TEST_ASSERT_TRUE_MESSAGE(altitudeDifference <= allowedError, 
                           "Detected apogee altitude differs significantly from maximum recorded altitude");
    
    // Verify apogee timing is reasonable (within 1 second of max altitude time)
    uint32_t timeDifference = std::abs((int64_t)detectedApogee.timestamp_ms - (int64_t)maxAltitudeTime);
    TEST_ASSERT_TRUE_MESSAGE(timeDifference <= 1000, 
                           "Detected apogee time differs significantly from maximum altitude time");
}
