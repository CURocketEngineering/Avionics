// test_VerticalVelocityEstimator_real_data.cpp
// Regression tests for VerticalVelocityEstimator using real‑flight CSV logs
// -----------------------------------------------------------------------------
#define DEBUG
#include "unity.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/StateEstimationTypes.h"   // AccelerationTriplet
#include "data_handling/DataPoint.h"
#include "../CSVMockData.h"                            // CSVDataProvider, SensorData

#include <cstdlib>
#include <string>
#include <vector>
#include <cmath>

// -----------------------------------------------------------------------------
// Helper – build AccelerationTriplet with common timestamp
// -----------------------------------------------------------------------------
static inline AccelerationTriplet makeAccel(uint32_t ts, float ax, float ay, float az)
{
    return AccelerationTriplet{ DataPoint(ts, ax), DataPoint(ts, ay), DataPoint(ts, az) };
}

static void test_vve_with_file(const std::string& file, float sampleRate_hz);

// -----------------------------------------------------------------------------
// Test entry point (iterates over multiple CSVs)
// -----------------------------------------------------------------------------
void test_vve_with_real_data(void)
{
    struct Src { std::string path; float fs; };   // path & sampling rate
    const std::vector<Src> sources{
        {"data/MARTHA_IREC_2025_B2_transformed.csv",         25.0f},
        {"data/AA Data Collection - Second Launch Trimmed.csv", 25.0f}
    };

    for (const auto& s : sources)
    {
        std::cout << "VerticalVelocityEstimator regression – file: " << s.path << std::endl;
        test_vve_with_file(s.path, s.fs);
    }
}

// -----------------------------------------------------------------------------
// Per‑file test logic
// -----------------------------------------------------------------------------
static void test_vve_with_file(const std::string& file, float sampleRate_hz)
{
    CSVDataProvider provider(file, sampleRate_hz);
    VerticalVelocityEstimator vve;

    // For diagnostics: write estimator trace to CSV
    const std::string outName = "vve_results_" + file.substr(file.find_last_of("/") + 1);
    std::ofstream out(outName);
    TEST_ASSERT_TRUE_MESSAGE(out.is_open(), "Failed to create results CSV");
    out << "t_ms,raw_alt,est_alt,est_vel,fdiff_vel,accZ,err_vel,err_alt\n";

    // Metrics we care about
    bool   first = true;
    float  prevAlt = 0.0f;
    uint32_t prevT = 0;
    float  sumSqVelErr = 0.0f;
    float  maxAltErr   = 0.0f;
    size_t nSamples    = 0;
    float fdiffVel = 0.0f;

    float smoothedVel  = 0.0f;
    constexpr float alpha = 0.50f;  // 0.0 = no update, 1.0 = no smoothing (more = snappier)

    while (provider.hasNextDataPoint())
    {
        SensorData d = provider.getNextDataPoint();
        const uint32_t t = d.time;

        AccelerationTriplet accel = makeAccel(t, d.accelx, d.accely, d.accelz);
        DataPoint alt(t, d.altitude);
        vve.update(accel, alt);

        if (!first)
        {
            const float dt = (t - prevT) * MILLISECONDS_TO_SECONDS;
            fdiffVel = (d.altitude - prevAlt) / dt;

            // Apply IIR low-pass filter to smooth finite-difference velocity
            smoothedVel = alpha * fdiffVel + (1.0f - alpha) * smoothedVel;

            const float err = vve.getEstimatedVelocity() - smoothedVel;
            sumSqVelErr += err * err;
            ++nSamples;

            // printf("t=%u, dt=%.3f, alt=%.2f, prevAlt=%.2f, fdiff=%.2f, smooth=%.2f, est=%.2f\n",
            //        t, dt, d.altitude, prevAlt, fdiffVel, smoothedVel, vve.getEstimatedVelocity());
        }

        const float altErr = std::fabs(vve.getEstimatedAltitude() - d.altitude);
        if (altErr > maxAltErr) maxAltErr = altErr;

        out << t << ',' << d.altitude << ',' << vve.getEstimatedAltitude() << ','
            << vve.getEstimatedVelocity() << ',' << smoothedVel << ','  // replace fdiffVel
            << vve.getInertialVerticalAcceleration() << ','
            << (nSamples ? vve.getEstimatedVelocity() - smoothedVel : 0.0f) << ','
            << altErr << '\n';

        first   = false;
        prevAlt = d.altitude;
        prevT   = t;
    }
    out.close();

    // Make sure we processed data
    TEST_ASSERT_TRUE_MESSAGE(nSamples > 100, "CSV too short or not read");

    // Evaluate overall metrics
    const float rmseVel = std::sqrt(sumSqVelErr / static_cast<float>(nSamples));

    // Allow fairly relaxed errors – tune as experience grows
    const float RMSE_VEL_MAX = 32.0f;     // m/s – typical GPS‑only velocity noise is ~5‑10 m/s
    const float MAX_ALT_ERR  = 100.0f;     // m  – altimeter may drift but estimator should stay close

    char msg[128];
    std::snprintf(msg, sizeof(msg), "Velocity RMSE %.2f > %.2f", rmseVel, RMSE_VEL_MAX);
    TEST_ASSERT_TRUE_MESSAGE(rmseVel <= RMSE_VEL_MAX, msg);

    std::snprintf(msg, sizeof(msg), "Altitude max abs err %.2f > %.2f", maxAltErr, MAX_ALT_ERR);
    TEST_ASSERT_TRUE_MESSAGE(maxAltErr <= MAX_ALT_ERR, msg);
}
