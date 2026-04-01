#define DEBUG
#include "unity.h"
#include "state_estimation/GroundLevelEstimator.h"
#include "ArduinoHAL.h"

#include <fstream>
#include <iostream>
#include <random>

void setUp(void)   { Serial.clear(); }
void tearDown(void){ Serial.clear(); }

// -----------------------------------------------------------------------------
// Test 1 – Initialization
// -----------------------------------------------------------------------------
void test_initialization(void)
{
    GroundLevelEstimator estimator;
    
    // Initial state: not launched, ground level should be 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.getEGL());
    
    // First update should return 0 AGL (still on ground)
    float agl = estimator.update(250.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, agl);
    
    // Ground level should now reflect the first sample
    TEST_ASSERT_EQUAL_FLOAT(250.0f, estimator.getEGL());
}

// -----------------------------------------------------------------------------
// Test 2 – Ground level estimation with single altitude
// -----------------------------------------------------------------------------
void test_ground_level_single_altitude(void)
{
    GroundLevelEstimator estimator;
    
    float groundASL = 350.0f;
    
    // Feed multiple samples at same altitude
    for (int i = 0; i < 100; ++i)
    {
        float agl = estimator.update(groundASL);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, agl);  // Should always return 0 before launch
    }
    
    // Ground level should converge to the input altitude
    TEST_ASSERT_FLOAT_WITHIN(0.01f, groundASL, estimator.getEGL());
}

// -----------------------------------------------------------------------------
// Test 3 – Ground level estimation with noisy data
// -----------------------------------------------------------------------------
void test_ground_level_with_noise(void)
{
    GroundLevelEstimator estimator;
    
    float trueGroundASL = 500.0f;
    std::default_random_engine rng{std::random_device{}()};
    std::normal_distribution<float> noise(0.0f, 2.0f);  // ±2m standard deviation
    
    // Feed noisy samples
    for (int i = 0; i < 500; ++i) // At 200 samples, the test was a but flaky due to noise.
    {
        float noisyASL = trueGroundASL + noise(rng);
        float agl = estimator.update(noisyASL);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, agl);
    }
    
    // With enough samples, running average should converge close to true value
    TEST_ASSERT_FLOAT_WITHIN(1.2f, trueGroundASL, estimator.getEGL());
}

// -----------------------------------------------------------------------------
// Test 4 – Ground level with varying pre-launch readings
// -----------------------------------------------------------------------------
void test_ground_level_varying_readings(void)
{
    GroundLevelEstimator estimator;
    
    // Simulate barometric drift on the pad
    float readings[] = {248.0f, 249.5f, 250.0f, 250.2f, 249.8f, 
                        250.1f, 249.9f, 250.3f, 250.0f, 249.7f};
    
    for (float reading : readings)
    {
        float agl = estimator.update(reading);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, agl);
    }
    
    // With EMA (alpha = 0.1), the estimate converges toward recent values
    // After 10 samples starting at 248.0f and varying around 250f,
    // the estimate will be influenced more by later samples
    // Expected value is approximately 249.8-250.0f (closer to recent readings)
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 249.2f, estimator.getEGL());
}

// -----------------------------------------------------------------------------
// Test 5 – AGL calculation after launch
// -----------------------------------------------------------------------------
void test_agl_after_launch(void)
{
    GroundLevelEstimator estimator;
    
    float groundASL = 300.0f;
    
    // Establish ground level
    for (int i = 0; i < 50; ++i)
    {
        estimator.update(groundASL);
    }
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, groundASL, estimator.getEGL());
    
    // Signal launch
    estimator.launchDetected();
    
    // Now updates should return AGL
    float agl1 = estimator.update(310.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, agl1);
    
    float agl2 = estimator.update(350.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, agl2);
    
    float agl3 = estimator.update(425.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 125.0f, agl3);
    
    // Ground level should remain frozen
    TEST_ASSERT_FLOAT_WITHIN(0.01f, groundASL, estimator.getEGL());
}

// -----------------------------------------------------------------------------
// Test 6 – Ground level remains frozen after launch
// -----------------------------------------------------------------------------
void test_ground_level_frozen_after_launch(void)
{
    GroundLevelEstimator estimator;
    
    float groundASL = 450.0f;
    
    // Establish ground level
    for (int i = 0; i < 30; ++i)
    {
        estimator.update(groundASL);
    }
    
    float eglBeforeLaunch = estimator.getEGL();
    
    // Launch
    estimator.launchDetected();
    
    // Feed many different altitudes
    for (int i = 0; i < 100; ++i)
    {
        float currentASL = groundASL + (static_cast<float>(i) * 10.0f);  // Climbing
        estimator.update(currentASL);
    }
    
    // Ground level should not have changed
    TEST_ASSERT_FLOAT_WITHIN(0.01f, eglBeforeLaunch, estimator.getEGL());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, groundASL, estimator.getEGL());
}

// -----------------------------------------------------------------------------
// Test 7 – Full flight simulation with CSV output
// -----------------------------------------------------------------------------
void test_full_flight_simulation(void)
{
    GroundLevelEstimator estimator;
    
    uint32_t ts = 0;
    float groundASL = 400.0f;
    float currentASL = groundASL;
    float velocity = 0.0f;
    const float dt = 0.01f;  // 10ms
    
    std::default_random_engine rng{std::random_device{}()};
    std::normal_distribution<float> noise(0.0f, 0.5f);
    
    std::ofstream csv("ground_level_test_output.csv");
    csv << "ts_ms,asl_m,egl_m,agl_m,launched\n";
    
    // ----------- Pre-launch phase (3 seconds on pad) -----------
    for (int i = 0; i < 300; ++i)
    {
        float noisyASL = groundASL + noise(rng);
        float agl = estimator.update(noisyASL);
        
        csv << ts << ',' << noisyASL << ',' 
            << estimator.getEGL() << ',' << agl << ",0\n";
        
        TEST_ASSERT_EQUAL_FLOAT(0.0f, agl);
        ts += 10;
    }
    
    // Verify ground level is established
    TEST_ASSERT_FLOAT_WITHIN(1.0f, groundASL, estimator.getEGL());
    
    // ----------- Launch detection -----------
    estimator.launchDetected();
    float eglAtLaunch = estimator.getEGL();
    
    // ----------- Powered ascent (3 seconds, ~70 m/s² net accel) -----------
    const float netAccel = 70.0f;
    for (int i = 0; i < 300; ++i)
    {
        velocity += netAccel * dt;
        currentASL += velocity * dt;
        
        float noisyASL = currentASL + noise(rng);
        float agl = estimator.update(noisyASL);
        
        csv << ts << ',' << noisyASL << ',' 
            << estimator.getEGL() << ',' << agl << ",1\n";
        
        // AGL should match ASL - ground level
        TEST_ASSERT_FLOAT_WITHIN(2.0f, currentASL - eglAtLaunch, agl);
        ts += 10;
    }
    
    // ----------- Coast phase -----------
    float maxAGL = 0.0f;
    while (velocity > 0.0f)
    {
        velocity += -9.81f * dt;
        currentASL += velocity * dt;
        
        float noisyASL = currentASL + noise(rng);
        float agl = estimator.update(noisyASL);
        
        if (agl > maxAGL) maxAGL = agl;
        
        csv << ts << ',' << noisyASL << ',' 
            << estimator.getEGL() << ',' << agl << ",1\n";
        
        ts += 10;
    }
    
    // ----------- Descent phase -----------
    for (int i = 0; i < 300 && currentASL > groundASL; ++i)
    {
        velocity += -9.81f * dt;
        currentASL += velocity * dt;
        if (currentASL < groundASL) currentASL = groundASL;
        
        float noisyASL = currentASL + noise(rng);
        float agl = estimator.update(noisyASL);
        
        csv << ts << ',' << noisyASL << ',' 
            << estimator.getEGL() << ',' << agl << ",1\n";
        
        ts += 10;
    }
    
    csv.close();
    
    // Ground level should still be frozen at launch value
    TEST_ASSERT_FLOAT_WITHIN(0.01f, eglAtLaunch, estimator.getEGL());
    
    // Should have reached significant altitude
    TEST_ASSERT_GREATER_THAN(1000.0f, maxAGL);
}

// -----------------------------------------------------------------------------
// Test 8 – Negative AGL during descent
// -----------------------------------------------------------------------------
void test_negative_agl_on_descent(void)
{
    GroundLevelEstimator estimator;
    
    float groundASL = 200.0f;
    
    // Establish ground
    for (int i = 0; i < 50; ++i)
    {
        estimator.update(groundASL);
    }
    
    estimator.launchDetected();
    
    // Flight to altitude
    estimator.update(500.0f);
    
    // Descent below original ground level (e.g., landing in valley)
    float agl = estimator.update(190.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, agl);
    
    agl = estimator.update(180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -20.0f, agl);
}

// -----------------------------------------------------------------------------
// Test 9 – Different ground altitudes (sea level to high altitude)
// -----------------------------------------------------------------------------
void test_various_ground_altitudes(void)
{
    float testAltitudes[] = {0.0f, 50.0f, 500.0f, 1500.0f, 3000.0f, 4500.0f};
    
    for (float groundASL : testAltitudes)
    {
        GroundLevelEstimator estimator;
        
        // Establish ground
        for (int i = 0; i < 50; ++i)
        {
            float agl = estimator.update(groundASL);
            TEST_ASSERT_EQUAL_FLOAT(0.0f, agl);
        }
        
        TEST_ASSERT_FLOAT_WITHIN(0.01f, groundASL, estimator.getEGL());
        
        // Launch and verify AGL calculation
        estimator.launchDetected();
        
        float testASL = groundASL + 100.0f;
        float agl = estimator.update(testASL);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, agl);
    }
}

// -----------------------------------------------------------------------------
// Test 10 – Early launch detection (few samples)
// -----------------------------------------------------------------------------
void test_early_launch_detection(void)
{
    GroundLevelEstimator estimator;
    
    float groundASL = 350.0f;
    
    // Only a few samples before launch (realistic quick countdown)
    estimator.update(349.5f);
    estimator.update(350.0f);
    estimator.update(350.5f);
    
    // Ground level estimate with limited samples
    float egl = estimator.getEGL();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, groundASL, egl);
    
    // Launch
    estimator.launchDetected();
    
    // AGL should still be calculated correctly
    float agl = estimator.update(400.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 50.0f, agl);
}

// -----------------------------------------------------------------------------
// Test 11 – Typo in method name (launchDetected vs launchDetected)
// -----------------------------------------------------------------------------
void test_launch_detection_method(void)
{
    GroundLevelEstimator estimator;
    
    estimator.update(300.0f);
    
    // Note: The method is spelled "launchDetected" (likely a typo)
    // This test documents the actual API
    estimator.launchDetected();
    
    float agl = estimator.update(310.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, agl);
}

// -----------------------------------------------------------------------------
// Test 12 – Exponential moving average accuracy
// -----------------------------------------------------------------------------
void test_exponential_moving_average_accuracy(void)
{
    GroundLevelEstimator estimator;
    
    // Known sequence to verify EMA calculation (alpha = 0.1)
    float samples[] = {100.0f, 102.0f, 98.0f, 101.0f, 99.0f};
    
    // Calculate expected EMA manually:
    // Sample 0: 100.0 (initialization)
    // Sample 1: 0.1*102.0 + 0.9*100.0 = 100.2
    // Sample 2: 0.1*98.0  + 0.9*100.2 = 99.98
    // Sample 3: 0.1*101.0 + 0.9*99.98 = 100.082
    // Sample 4: 0.1*99.0  + 0.9*100.082 = 99.9738
    float expectedEMA = 99.9738f;
    
    for (float sample : samples)
    {
        estimator.update(sample);
    }
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expectedEMA, estimator.getEGL());
}

void test_ground_level_estimation_can_change_after_many_samples(void)
{
    GroundLevelEstimator estimator;
    
    // Feed many samples at one altitude
    for (int i = 0; i < 1000; ++i)
    {
        estimator.update(300.0f);
    }
    
    // Ground level should be stable around 300m
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, estimator.getEGL());
    
    // Now feed a different altitude before launch
    for (int i = 0; i < 100; ++i)
    {
        estimator.update(350.0f);
    }
    
    // Ground level should have shifted towards the new value due to EMA
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 350.0f, estimator.getEGL());
}

// -----------------------------------------------------------------------------
// Test 13 – Zero altitude ground level
// -----------------------------------------------------------------------------
void test_zero_altitude_ground(void)
{
    GroundLevelEstimator estimator;
    
    // Launch site at sea level
    for (int i = 0; i < 50; ++i)
    {
        float agl = estimator.update(0.0f);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, agl);
    }
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.getEGL());
    
    estimator.launchDetected();
    
    float agl = estimator.update(50.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, agl);
}

// -----------------------------------------------------------------------------
// Test 14 – High sample count stability
// -----------------------------------------------------------------------------
void test_high_sample_count_stability(void)
{
    GroundLevelEstimator estimator;
    
    float groundASL = 400.0f;
    
    // Very long pre-launch period (simulating long pad wait)
    for (int i = 0; i < 10000; ++i)
    {
        estimator.update(groundASL + (i % 2 ? 0.1f : -0.1f));
    }
    
    // Should still be stable
    TEST_ASSERT_FLOAT_WITHIN(0.5f, groundASL, estimator.getEGL());
    
    estimator.launchDetected();
    float agl = estimator.update(500.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, agl);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_ground_level_single_altitude);
    RUN_TEST(test_ground_level_with_noise);
    RUN_TEST(test_ground_level_varying_readings);
    RUN_TEST(test_agl_after_launch);
    RUN_TEST(test_ground_level_frozen_after_launch);
    RUN_TEST(test_full_flight_simulation);
    RUN_TEST(test_negative_agl_on_descent);
    RUN_TEST(test_various_ground_altitudes);
    RUN_TEST(test_early_launch_detection);
    RUN_TEST(test_launch_detection_method);
    RUN_TEST(test_exponential_moving_average_accuracy);
    RUN_TEST(test_ground_level_estimation_can_change_after_many_samples);
    RUN_TEST(test_zero_altitude_ground);
    RUN_TEST(test_high_sample_count_stability);
    return UNITY_END();
}
