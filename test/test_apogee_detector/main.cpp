// test_ApogeeDetector.cpp
#define DEBUG
#include "unity.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/StateEstimationTypes.h"   // << AccelerationTriplet
#include "data_handling/DataPoint.h"
#include "ArduinoHAL.h"                              // (mock Serial)

#include <fstream>
#include <iostream>
#include <random>

// Forward declaration of optional CSV test (if you keep it elsewhere)
void test_apogee_detector_with_real_data(void);

void setUp(void)   { Serial.clear(); }
void tearDown(void){ Serial.clear(); }

// -----------------------------------------------------------------------------
// Test 1 – Initialisation
// -----------------------------------------------------------------------------
void test_initialization(void)
{
    ApogeeDetector            detector;
    VerticalVelocityEstimator vve;

    TEST_ASSERT_FALSE(detector.isApogeeDetected());
    DataPoint apo = detector.getApogee();
    TEST_ASSERT_EQUAL_UINT32(0,     apo.timestamp_ms);
    TEST_ASSERT_EQUAL_FLOAT (0.0f,  apo.data);

    const uint32_t ts = 1000;
    // 0 m alt, free-fall accelerations in X/Y, +g+10 m/s² in Z
    AccelerationTriplet accel{
        DataPoint(ts, 0.0f),          // X
        DataPoint(ts, 0.0f),          // Y
        DataPoint(ts, 19.81f)         // Z
    };
    DataPoint alt(ts, 0.0f);

    vve.update(accel, alt);
    detector.update(&vve);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, vve.getEstimatedAltitude());
    TEST_ASSERT_FALSE(detector.isApogeeDetected());
}

// -----------------------------------------------------------------------------
// Test 2 – No apogee while climbing
// -----------------------------------------------------------------------------
void test_no_apogee_during_ascent(void)
{
    ApogeeDetector            detector;
    VerticalVelocityEstimator vve;

    uint32_t ts       = 1000;
    float     altitude = 0.0f;

    for (int i = 0; i < 50; ++i)            // 50×10 ms steps
    {
        ts       += 10;
        altitude += 0.5f;                   // +0.5 m per step

        AccelerationTriplet accel{
            DataPoint(ts, 0.0f),            // X
            DataPoint(ts, 0.0f),            // Y
            DataPoint(ts, 19.81f)           // Z (g + 10 m/s²)
        };
        DataPoint alt(ts, altitude);

        vve.update(accel, alt);
        detector.update(&vve);
        TEST_ASSERT_FALSE(detector.isApogeeDetected());
    }
}

// -----------------------------------------------------------------------------
// Test 3 – Full-flight simulation (powered, coast, descent) + CSV log
// -----------------------------------------------------------------------------
void test_apogee_detection(void)
{
    ApogeeDetector            detector;
    VerticalVelocityEstimator vve;

    uint32_t ts = 1000;            // ms
    float    trueAlt     = 0.0f;   // m
    float    trueVel     = 0.0f;   // m/s
    const float dt       = 0.01f;  // 10 ms
    const int   dt_ms    = 10;

    // ------------- powered ascent -------------
    const int   burnSteps      = 300;       // 3 s
    const float netAccel_mps2  = 70.0f;     // net upward accel (excluding -g)
    std::default_random_engine             rng{std::random_device{}()};
    std::normal_distribution<float> noiseA (0.0f, 0.05f);
    std::normal_distribution<float> noiseH (0.0f, 0.3f);

    std::ofstream csv("apogee_test_output.csv");
    csv << "ts,trueAlt,estAlt,estVel,apogee\n";

    for (int i = 0; i < burnSteps; ++i)
    {
        ts      += dt_ms;
        trueVel += netAccel_mps2 * dt;
        trueAlt += trueVel * dt;

        AccelerationTriplet accel{
            DataPoint(ts, noiseA(rng)),                     // X
            DataPoint(ts, noiseA(rng)),                     // Y
            DataPoint(ts, netAccel_mps2 + noiseA(rng))      // Z
        };
        DataPoint alt(ts, trueAlt + noiseH(rng));

        vve.update(accel, alt);
        detector.update(&vve);

        csv << ts << ',' << trueAlt << ','
            << vve.getEstimatedAltitude() << ','
            << vve.getEstimatedVelocity() << ','
            << (detector.isApogeeDetected() ? 1 : 0) << '\n';
    }

    // ------------- coast (free-fall ascent) -------------
    float maxAlt       = trueAlt;
    uint32_t trueApoTs = ts;

    while (trueVel > 0.0f)
    {
        ts      += dt_ms;
        trueVel += -9.81f * dt;
        trueAlt += trueVel * dt;

        if (trueAlt > maxAlt) { maxAlt = trueAlt; trueApoTs = ts; }

        AccelerationTriplet accel{
            DataPoint(ts, noiseA(rng)),
            DataPoint(ts, noiseA(rng)),
            DataPoint(ts, 0.0f + noiseA(rng))
        };
        DataPoint alt(ts, trueAlt + noiseH(rng));

        vve.update(accel, alt);
        detector.update(&vve);

        csv << ts << ',' << trueAlt << ','
            << vve.getEstimatedAltitude() << ','
            << vve.getEstimatedVelocity() << ','
            << (detector.isApogeeDetected() ? 1 : 0) << '\n';

        TEST_ASSERT_FALSE(detector.isApogeeDetected());
    }

    // trigger a first descent step
    ts      += dt_ms;
    trueVel += -9.81f * dt;
    trueAlt += trueVel * dt;
    {
        AccelerationTriplet accel{
            DataPoint(ts, noiseA(rng)),
            DataPoint(ts, noiseA(rng)),
            DataPoint(ts, 0.0f + noiseA(rng))
        };
        DataPoint alt(ts, trueAlt + noiseH(rng));
        vve.update(accel, alt);
        detector.update(&vve);
    }

    // ------------- descent loop -------------
    for (int i = 0; i < 200 && !detector.isApogeeDetected(); ++i)
    {
        ts      += dt_ms;
        trueVel += -9.81f * dt;
        trueAlt += trueVel * dt;

        AccelerationTriplet accel{
            DataPoint(ts, noiseA(rng)),
            DataPoint(ts, noiseA(rng)),
            DataPoint(ts, 0.0f + noiseA(rng))
        };
        DataPoint alt(ts, trueAlt + noiseH(rng));

        vve.update(accel, alt);
        detector.update(&vve);
    }
    csv.close();

    TEST_ASSERT_TRUE(detector.isApogeeDetected());
    DataPoint apo = detector.getApogee();
    TEST_ASSERT_FLOAT_WITHIN(20.0f, maxAlt, apo.data);
    TEST_ASSERT_UINT32_WITHIN(100, trueApoTs, apo.timestamp_ms);
}

// -----------------------------------------------------------------------------
// Test 4 – getEstimated* helpers
// -----------------------------------------------------------------------------
void test_get_estimated_values(void)
{
    ApogeeDetector            detector;
    VerticalVelocityEstimator vve;

    uint32_t ts = 1000;
    float    alt = 0.0f;

    // first update
    {
        AccelerationTriplet accel{
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f),
            DataPoint(ts, 19.81f)
        };
        DataPoint dpAlt(ts, alt);
        vve.update(accel, dpAlt);
        detector.update(&vve);
    }

    // steady - hover @10 m, accel ≈ g
    for (int i = 0; i < 1000; ++i)
    {
        ts += 10;
        alt = 10.0f;

        AccelerationTriplet accel{
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f),
            DataPoint(ts, 9.8f)
        };
        DataPoint dpAlt(ts, alt);

        vve.update(accel, dpAlt);
        detector.update(&vve);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.5f, alt, vve.getEstimatedAltitude());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, vve.getEstimatedVelocity());
}

// -----------------------------------------------------------------------------
// Test 5 – handle an old timestamp gracefully
// -----------------------------------------------------------------------------
void test_update_with_old_timestamp(void)
{
    ApogeeDetector            detector;
    VerticalVelocityEstimator vve;

    uint32_t ts = 1000;
    float    alt = 0.0f;

    // first update
    {
        AccelerationTriplet accel{
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f),
            DataPoint(ts, 19.81f)
        };
        vve.update(accel, DataPoint(ts, alt));
        detector.update(&vve);
    }

    // older timestamp
    uint32_t oldTs = 900;
    AccelerationTriplet accelOld{
        DataPoint(oldTs, 0.0f),
        DataPoint(oldTs, 0.0f),
        DataPoint(oldTs, 19.81f)
    };
    vve.update(accelOld, DataPoint(oldTs, alt));
    detector.update(&vve);

    TEST_ASSERT_TRUE(vve.getEstimatedAltitude() >= alt);
}

// -----------------------------------------------------------------------------
// Test 6 – apogee remains fixed after detection
// -----------------------------------------------------------------------------
void test_multiple_updates_after_apogee(void)
{
    ApogeeDetector            detector;
    VerticalVelocityEstimator vve;

    uint32_t ts  = 1000;
    float    alt = 0.0f;

    // ascent
    for (int i = 0; i < 200; ++i)
    {
        ts  += 10;
        alt += 0.5f;
        AccelerationTriplet accel{
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f),
            DataPoint(ts, 19.81f)
        };
        vve.update(accel, DataPoint(ts, alt));
        detector.update(&vve);
    }

    // descent to trigger apogee
    for (int i = 0; i < 200; ++i)
    {
        ts  += 10;
        alt -= 0.5f;
        AccelerationTriplet accel{
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f)
        };
        vve.update(accel, DataPoint(ts, alt));
        detector.update(&vve);
        if (detector.isApogeeDetected()) break;
    }

    TEST_ASSERT_TRUE(detector.isApogeeDetected());
    DataPoint apo1 = detector.getApogee();

    // further descent updates should not move the apogee point
    for (int i = 0; i < 10; ++i)
    {
        ts  += 10;
        alt -= 0.5f;
        AccelerationTriplet accel{
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f),
            DataPoint(ts, 0.0f)
        };
        vve.update(accel, DataPoint(ts, alt));
        detector.update(&vve);
    }

    DataPoint apo2 = detector.getApogee();
    TEST_ASSERT_EQUAL_UINT32(apo1.timestamp_ms, apo2.timestamp_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, apo1.data, apo2.data);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_no_apogee_during_ascent);
    RUN_TEST(test_apogee_detection);
    RUN_TEST(test_get_estimated_values);
    RUN_TEST(test_update_with_old_timestamp);
    RUN_TEST(test_multiple_updates_after_apogee);
    RUN_TEST(test_apogee_detector_with_real_data);   // external CSV-driven test
    return UNITY_END();
}
