// test_VerticalVelocityEstimator.cpp
// Unit tests for VerticalVelocityEstimator – inspired by ApogeeDetector tests
// -----------------------------------------------------------------------------
#define DEBUG
#include "unity.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/StateEstimationTypes.h"   // AccelerationTriplet
#include "data_handling/DataPoint.h"
#include "ArduinoHAL.h"                              // (mock Serial)
#include "test_vve_csv.h"

#include <cmath>
#include <random>

void setUp(void)   { Serial.clear(); }
void tearDown(void){ Serial.clear(); }

// -----------------------------------------------------------------------------
// Test 1 – Default‑constructed estimator has zero state and is not initialised
// -----------------------------------------------------------------------------
void test_default_state(void)
{
    VerticalVelocityEstimator vve;

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, vve.getEstimatedAltitude());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, vve.getEstimatedVelocity());
    TEST_ASSERT_EQUAL_UINT32(0U, vve.getTimestamp());
    TEST_ASSERT_EQUAL_INT8(0,  vve.getVerticalAxis());         // not yet determined → 0
    TEST_ASSERT_EQUAL_INT8(0,  vve.getVerticalDirection());    // idem
}

// -----------------------------------------------------------------------------
// Test 2 – First update initialises the filter and determines vertical axis
// -----------------------------------------------------------------------------
void test_first_update_initialises_filter(void)
{
    VerticalVelocityEstimator vve;

    const uint32_t ts1 = 1000;
    // +g on Z → axis 2, direction +1 – hover at 0 m
    AccelerationTriplet accel = makeAccel(ts1, 0.0f, 0.0f, 9.81f);
    DataPoint          alt   (ts1, 0.0f);

    vve.update(accel, alt);

    TEST_ASSERT_EQUAL_UINT32(ts1, vve.getTimestamp());

    const uint32_t ts2 = 1010;
    // +g on Z → axis 2, direction +1 – hover at 0 m
    AccelerationTriplet accel2 = makeAccel(ts2, 0.0f, 0.0f, 9.81f);
    DataPoint          alt2   (ts2, 0.0f);

    vve.update(accel2, alt2);

    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, vve.getEstimatedAltitude());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, vve.getEstimatedVelocity());
    TEST_ASSERT_EQUAL_UINT32(ts2, vve.getTimestamp());
    TEST_ASSERT_EQUAL_INT8(2,  vve.getVerticalAxis());
    TEST_ASSERT_EQUAL_INT8(1,  vve.getVerticalDirection());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, vve.getInertialVerticalAcceleration());
}

// -----------------------------------------------------------------------------
// Test 3 – Hover: constant altitude with accel ≈ +g keeps velocity ≈ 0
// -----------------------------------------------------------------------------
void test_hover(void)
{
    VerticalVelocityEstimator vve;
    uint32_t ts = 1000;

    // Initialise (same as previous test)
    vve.update(makeAccel(ts, 0.0f, 0.0f, 9.81f), DataPoint(ts, 10.0f));

    // Hold position at 10 m for 1000×10 ms = 10 s
    for (int i = 0; i < 1000; ++i)
    {
        ts += 10;
        vve.update(makeAccel(ts, 0.0f, 0.0f, 9.81f), DataPoint(ts, 10.0f));
    }

    TEST_ASSERT_FLOAT_WITHIN(0.3f, 10.0f, vve.getEstimatedAltitude());
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 0.0f,  vve.getEstimatedVelocity());
}

// -----------------------------------------------------------------------------
// Test 4 – Constant 10 m/s² net upward accel for 1 s (g+10 on Z)
// -----------------------------------------------------------------------------
void test_constant_accel_ascent(void)
{
    VerticalVelocityEstimator vve;

    uint32_t ts       = 1000;
    const float dt    = 0.01f;    // 10 ms
    const int   steps = 100;      // 1 s total

    // Start at altitude 0 m
    vve.update(makeAccel(ts, 0.0f, 0.0f, 9.81f), DataPoint(ts, 0.0f));

    float trueVel = 0.0f;
    float trueAlt = 0.0f;

    for (int i = 0; i < steps; ++i)
    {
        ts += 10;
        // Raw accel = g + 10 → inertial +10 m/s²
        vve.update(makeAccel(ts, 0.0f, 0.0f, 19.81f), DataPoint(ts, trueAlt));

        // Ground‑truth propagation
        trueVel += 10.0f * dt;
        trueAlt += trueVel * dt;
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, trueAlt, vve.getEstimatedAltitude());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, trueVel, vve.getEstimatedVelocity());
}

// -----------------------------------------------------------------------------
// Test 5 – Vertical axis determination (negative direction)
// -----------------------------------------------------------------------------
void test_vertical_axis_negative_direction(void)
{
    VerticalVelocityEstimator vve;
    const uint32_t ts = 2000;

    // Device lying on its +Z face: gravity gives –g on X (largest magnitude)
    vve.update(makeAccel(ts, -9.81f, 0.0f, 0.0f), DataPoint(ts, 0.0f));

    // It takes 2 updates before it determines the vertical axis
    const uint32_t ts2 = 2010;
    vve.update(makeAccel(ts2, -9.81f, 0.0f, 0.0f), DataPoint(ts2, 0.0f));

    TEST_ASSERT_EQUAL_INT8(0,  vve.getVerticalAxis());
    TEST_ASSERT_EQUAL_INT8(-1, vve.getVerticalDirection());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, vve.getInertialVerticalAcceleration());
}

// -----------------------------------------------------------------------------
// Test 6 – Graceful handling of an older timestamp (dt → MINIMUM_DELTA_T_S)
// -----------------------------------------------------------------------------
void test_update_with_old_timestamp(void)
{
    VerticalVelocityEstimator vve;

    uint32_t ts = 3000;
    vve.update(makeAccel(ts, 0.0f, 0.0f, 9.81f), DataPoint(ts, 5.0f));

    const float alt_before = vve.getEstimatedAltitude();

    // Provide an older reading (ts – 100 ms)
    uint32_t oldTs = ts - 100;
    vve.update(makeAccel(oldTs, 0.0f, 0.0f, 9.81f), DataPoint(oldTs, 5.0f));

    // Altitude should not jump backwards or NaN
    TEST_ASSERT_TRUE(std::isfinite(vve.getEstimatedAltitude()));
    TEST_ASSERT_TRUE(vve.getEstimatedAltitude() >= alt_before - 0.5f);
}

// -----------------------------------------------------------------------------
// Test 7 – Noise robustness: random walk altimeter noise, stationary target
// -----------------------------------------------------------------------------
void test_noise_robustness(void)
{
    VerticalVelocityEstimator vve;
    std::default_random_engine          rng{std::random_device{}()};
    std::normal_distribution<float> nAlt(0.0f, 0.5f);   // ±0.5 m 1σ
    std::normal_distribution<float> nAcc(0.0f, 0.05f);  // ±0.05 m/s² 1σ

    uint32_t ts = 4000;
    const float trueAlt = 20.0f;

    // Init at 20 m
    vve.update(makeAccel(ts, 0.0f, 0.0f, 9.81f + nAcc(rng)), DataPoint(ts, trueAlt + nAlt(rng)));

    // 5 s stationary
    for (int i = 0; i < 500; ++i)
    {
        ts += 10;
        vve.update(makeAccel(ts, 0.0f, 0.0f, 9.81f + nAcc(rng)), DataPoint(ts, trueAlt + nAlt(rng)));
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, trueAlt, vve.getEstimatedAltitude());
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f,     vve.getEstimatedVelocity());
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_default_state);
    RUN_TEST(test_first_update_initialises_filter);
    RUN_TEST(test_hover);
    RUN_TEST(test_constant_accel_ascent);
    RUN_TEST(test_vertical_axis_negative_direction);
    RUN_TEST(test_update_with_old_timestamp);
    RUN_TEST(test_noise_robustness);
    RUN_TEST(test_vve_with_real_data);
    return UNITY_END();
}
