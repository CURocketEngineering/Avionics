#include "unity.h"
#include "state_estimation/ApogeePredictor.h"
#include <iostream>

/* ---------------- Stub VVE ---------------- */
class VerticalVelocityEstimatorStub : public VerticalVelocityEstimator {
    public:
        VerticalVelocityEstimatorStub() {} 
    
        void set(float vel_mps, float netAcl_mps2, float alt_m, uint32_t timestamp_ms) {
            vel_ = vel_mps;
            acl_ = netAcl_mps2;
            alt_ = alt_m;
            ts_  = timestamp_ms;
        }

        float getEstimatedVelocity() const override { return vel_; }
        float getInertialVerticalAcceleration() const override { return acl_; }
        float getEstimatedAltitude() const override { return alt_; }
        uint32_t getTimestamp() const override { return ts_; }
    
    private:
        float vel_{0.0f}, acl_{0.0f}, alt_{0.0f};
        uint32_t ts_{10};
    };

/* ---------- Unity Fixtures ---------- */
void setUp(void) {}
void tearDown(void) {}

/* 1) Min climb velocity check */
void test_min_climb_velocity_gate(void) {
    VerticalVelocityEstimatorStub vve;
    ApogeePredictor apo(vve, 1.0f, 1.0f);

    vve.set(0.5f, -5.0f, 0.0f, 5);
    apo.update();
    TEST_ASSERT_FALSE(apo.isPredictionValid());

    vve.set(10.0f, -5.0f, 0.0f, 10);
    apo.update();
    TEST_ASSERT_TRUE(apo.isPredictionValid());
}

/* 2) Closed-form check */
void test_time_and_altitude_projection(void) {
    VerticalVelocityEstimatorStub vve;
    ApogeePredictor apo(vve, 1.0f, 0.0f);

    const float v = 20.0f, acl = -5.0f, h0 = 100.0f;
    const uint32_t t0 = 1000;

    vve.set(v, acl, h0, t0);

    // Update twice to overwhelm the filters
    apo.update();
    apo.update();

    const float t_apogee = v / std::abs(acl);  // 4s
    const float h_apogee = h0 + v*t_apogee - 0.5f*std::abs(acl)*t_apogee*t_apogee;
    const uint32_t ts_expected = t0 + static_cast<uint32_t>(t_apogee * 1000.0f + 0.5f);

    TEST_ASSERT_TRUE(apo.isPredictionValid());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, t_apogee, apo.getTimeToApogee_s());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, h_apogee, apo.getPredictedApogeeAltitude_m());
    TEST_ASSERT_EQUAL_UINT32(ts_expected, apo.getPredictedApogeeTimestamp_ms());
}

/* 3) EMA test */
void test_filtered_deceleration_ema(void) {
    VerticalVelocityEstimatorStub vve;
    constexpr float alpha = 0.2f;
    ApogeePredictor apo(vve, alpha, 0.0f);

    vve.set(10.0f, -4.0f, 0.0f, 0);
    apo.update();

    TEST_ASSERT_GREATER_THAN_FLOAT(0.1f, apo.getFilteredDeceleration());

    vve.set(10.0f, -6.0f, 0.0f, 10);
    apo.update();
    apo.update();
    TEST_ASSERT_GREATER_THAN_FLOAT(0.1f, apo.getFilteredDeceleration());
}

/* 4) Invalidation on descent */
void test_invalid_after_descent(void) {
    VerticalVelocityEstimatorStub vve;
    ApogeePredictor apo(vve, 1.0f, 0.0f);

    vve.set(5.0f, -9.81f, 50.0f, 0);
    apo.update();
    TEST_ASSERT_TRUE(apo.isPredictionValid());

    vve.set(-2.0f, -9.81f, 60.0f, 100);
    apo.update();
    TEST_ASSERT_FALSE(apo.isPredictionValid());
}

/* -------------- Main ------------------ */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_min_climb_velocity_gate);
    RUN_TEST(test_time_and_altitude_projection);
    RUN_TEST(test_filtered_deceleration_ema);
    RUN_TEST(test_invalid_after_descent);
    return UNITY_END();
}
