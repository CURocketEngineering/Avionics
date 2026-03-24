#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include "unity.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "data_handling/DataPoint.h"
#include "../CSVMockData.h"

// Data sources to use
// data/MARTHA_IREC_2025_B2_transformed.csv
// data/AA Data Collection - Second Launch Trimmed.csv

// Function prototypes
void test_apogee_detector_with_real_data(void);
void test_apogee_detector_with_file(const std::string& file);

/**
 * Test the ApogeeDetector using real flight data from CSV.
 * This test reads sensor data from a CSV file, feeds it into the detector,
 * and verifies apogee detection while outputting results to a CSV for analysis.
 */
void test_apogee_detector_with_real_data(void) {
    // Vector of sim files
    std::vector<std::string> simFiles = {
        "data/MARTHA_IREC_2025_B2_transformed.csv",
        "data/AA Data Collection - Second Launch Trimmed.csv"
    };

    for (const auto& file : simFiles) {
        std::cout << "Testing with file: " << file << std::endl;
        // Run the test for each file
        test_apogee_detector_with_file(file);
    }
}


void test_apogee_detector_with_file(const std::string& file) {
    // Create CSV provider to read test data with 25Hz sample rate (40ms interval)
    CSVDataProvider provider(file, 25.0f);
    ApogeeDetector detector;
    VerticalVelocityEstimator vve;
    
    // Create output CSV file for analysis
    std::ofstream outputFile("apogee_detector_results_real.csv");
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open output CSV file" << std::endl;
        TEST_FAIL();
        return;
    }
    
    // Write CSV header
    outputFile << "timestamp,raw_altitude,estimated_altitude,estimated_velocity,vertical_acceleration,apogee_detected\n";
    
    bool hasData = false;
    float maxAltitude = -1000.0f; // Initialize to a very low value
    uint32_t maxAltitudeTime = 0;
    
    // Process each data point at 25Hz
    while (provider.hasNextDataPoint()) {
        hasData = true;
        SensorData data = provider.getNextDataPoint();
        
        // Create DataPoints for the detector
        DataPoint accelX(data.time, data.accelx);
        DataPoint accelY(data.time, data.accely);
        DataPoint accelZ(data.time, data.accelz);
        DataPoint alt(data.time, data.altitude);
        
        // Update detector with sensor data
        AccelerationTriplet accel {
            accelX,  // X-axis acceleration
            accelY,  // Y-axis acceleration
            accelZ   // Z-axis acceleration (vertical)
        };
        vve.update(accel, alt);
        detector.update(&vve);
        
        // Track maximum altitude for validation
        if (data.altitude > maxAltitude) {
            maxAltitude = data.altitude;
            maxAltitudeTime = data.time;
        }
        
        // Write results to CSV
        outputFile << data.time << ","
                  << data.altitude << ","
                  << vve.getEstimatedAltitude() << ","
                  << vve.getEstimatedVelocity() << ","
                  << vve.getInertialVerticalAcceleration() << ","
                  << (detector.isApogeeDetected() ? "1" : "0") << "\n";
    }
    
    outputFile.close();
    
    // Verify test data was processed
    TEST_ASSERT_TRUE(hasData);
    
    // Verify apogee detection
    TEST_ASSERT_TRUE(detector.isApogeeDetected());
    
    // Get detected apogee information
    DataPoint detectedApogee = detector.getApogee();
    
    // Verify apogee altitude is reasonable (within 10% of max recorded altitude)
    float altitudeDifference = std::abs(detectedApogee.data - maxAltitude);
    float allowedError = maxAltitude * 0.10f; // 10% error margin
    TEST_ASSERT_TRUE_MESSAGE(altitudeDifference <= allowedError, 
                           "Detected apogee altitude differs significantly from maximum recorded altitude");
    
    // Verify apogee timing is reasonable (within 1 second of max altitude time)
    uint32_t timeDifference = std::abs((int64_t)detectedApogee.timestamp_ms - (int64_t)maxAltitudeTime);
    TEST_ASSERT_TRUE_MESSAGE(timeDifference <= 1000, 
                           "Detected apogee time differs significantly from maximum altitude time");
}
