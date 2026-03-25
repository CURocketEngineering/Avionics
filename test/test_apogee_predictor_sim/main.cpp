#include "unity.h"
#include <fstream>
#include <random>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

#include "AirResistanceSimulation.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/ApogeePredictor.h"
#include "data_handling/DataPoint.h"
#include "data_handling/CircularArray.h"

/* ---------- choose prediction method ---------- */
static void (ApogeePredictor::*predictionMethod)(void) = 
&ApogeePredictor::analytic_update
//&ApogeePredictor::simulate_update
//&ApogeePredictor::quad_update
//&ApogeePredictor::poly_update
//&ApogeePredictor::update
;

/* ---------- choose dataset ---------- */
static const char* dataset = 
//"data/MARTHA_3-8_1.3_B2_SingleID_transformed.csv";
//"data/MARTHA_IREC_2025_B2_transformed.csv";
//"data/AA Data Collection - Second Launch Trimmed.csv";
"data/data_transformed.csv";


/* ---------- helpers ---------- */
static std::default_random_engine rng{42};
static std::normal_distribution<float> aclNoise(0.0f, 0.55f);
static std::normal_distribution<float> altNoise(0.0f, 3.0f);

auto safe_stof = [](const std::string &s, float default_val = 0.0f) -> float {
    try { return std::stof(s); } catch (...) { return default_val; }
};
auto safe_stoul = [](const std::string &s, uint32_t default_val = 0) -> uint32_t {
    try { return std::stoul(s); } catch (...) { return default_val; }
};

void SetupOutputCSV(std::ofstream &csv) {
    TEST_ASSERT_TRUE_MESSAGE(csv.is_open(), "Failed to open CSV file for writing");
    csv << "timestamp,true_alt,true_vertical_velocity,"
           "est_alt,est_vertical_velocity,true_acl,est_acl,cd,est_apogee\n";
}

inline float computeVerticalVelocity(float prev_alt, float alt, uint32_t prev_ts, uint32_t ts) {
    return (prev_ts > 0) ? (alt - prev_alt) / ((ts - prev_ts) * 0.001f) : 0.0f;
}

inline AccelerationTriplet buildAccelerationTriplet(uint32_t ts, float ax, float ay, float az) {
    return { DataPoint(ts, ax), DataPoint(ts, ay), DataPoint(ts, az) };
}

inline void writeCsvRow(std::ofstream &csv, uint32_t ts, float alt, float vertical_vel, VerticalVelocityEstimator &vve, ApogeePredictor &apo, float aclZ = 0.0f) 
{
    csv << ts << ','
        << alt << ','
        << vertical_vel << ','
        << vve.getEstimatedAltitude() << ','
        << vve.getEstimatedVelocity() << ','
        << aclZ << ','
        << vve.getInertialVerticalAcceleration() << ','
        << apo.getdragCoefficient() << ',';
    if (apo.isPredictionValid()) csv << apo.getPredictedApogeeAltitude_m();
    else csv << 0.0f;
    csv << '\n';
}

bool parseCsvRow(const std::vector<std::string>& tokens, int idx_ts, int idx_ax, int idx_ay, int idx_az, int idx_alt, uint32_t &ts, float &ax, float &ay, float &az, float &alt) 
{
    if (tokens.size() <= std::max({idx_ts, idx_ax, idx_ay, idx_az, idx_alt})) return false;
    ts  = safe_stoul(tokens[idx_ts]);
    ax  = safe_stof(tokens[idx_ax]);
    ay  = safe_stof(tokens[idx_ay]);
    az  = safe_stof(tokens[idx_az]);
    alt = safe_stof(tokens[idx_alt]);
    return true;
}

/* ---------- Unity fixtures ---------- */
void setUp(void) {}
void tearDown(void) {}

/* ---------- Test 1: Generated CSV ---------- */
void test_apogee_predictor_generates_csv(void (ApogeePredictor::*predictionMethod)(void))
{
    constexpr uint32_t TICK_MS = 10;
    AirResistanceSimulator sim(2000, 55.0f, 1500, TICK_MS, 0.0008f);
    NoiseVariances noise = {1.05F, 10.0F};
    VerticalVelocityEstimator vve(noise);
    ApogeePredictor apo(vve, 0.2f, 1.0f);

    std::ofstream csv("apogee_prediction_generated.csv");
    SetupOutputCSV(csv);

    bool is_post_burnout = false;
    bool check_apogee_prediction = false;
    CircularArray<DataPoint, 10> predicted_apogees(10);

    while (!sim.getHasLanded())
    {
        sim.tick();
        const uint32_t ts   = sim.getCurrentTime();
        const float aclZ    = sim.getInertialVerticalAcl() + 9.81f + aclNoise(rng);
        const float alt     = sim.getAltitude() + altNoise(rng);
        AccelerationTriplet accel = buildAccelerationTriplet(ts, 0.0f, 0.0f, aclZ);

        DataPoint altDp(ts, alt);
        vve.update(accel, altDp);

        if (!is_post_burnout && sim.getLaunchTimestamp() > 0 && sim.getLaunchTimestamp() + 1500 < ts) {
            is_post_burnout = true;
            TEST_ASSERT_TRUE(ts > 1500);
            TEST_ASSERT_FALSE(apo.isPredictionValid());
        }
        if (is_post_burnout) (apo.*predictionMethod)();

        float vertical_vel = sim.getVerticalVel();
        writeCsvRow(csv, ts, alt, vertical_vel, vve, apo, sim.getInertialVerticalAcl());

        if (apo.isPredictionValid()) predicted_apogees.push(DataPoint(ts, apo.getPredictedApogeeAltitude_m()));

        if (sim.getApogeeTimestamp() < ts && sim.getApogeeTimestamp() > 0 && !check_apogee_prediction) {
            check_apogee_prediction = true;
            DataPoint oldest = predicted_apogees.getFromHead(predicted_apogees.getMaxSize() - 1);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, sim.getApogeeAlt(), oldest.data,
                "Predicted apogee was not close to true apogee");
        }
    }
    csv.close();
}

/* ---------- Test 2: Real CSV ---------- */
void test_apogee_predictor_with_real_csv(void (ApogeePredictor::*predictionMethod)(void))
{
    std::ifstream file(dataset);
    TEST_ASSERT_TRUE_MESSAGE(file.is_open(), "Failed to open CSV file");

    std::ofstream csv("apogee_prediction.csv");
    SetupOutputCSV(csv);

    VerticalVelocityEstimator vve;
    ApogeePredictor apo(vve, 0.2f, 0.5f);

    std::string line; std::getline(file, line); // header
    std::stringstream header_ss(line);
    std::string col; std::vector<std::string> headers;
    while (std::getline(header_ss, col, ',')) headers.push_back(col);

    int idx_ts=-1, idx_ax=-1, idx_ay=-1, idx_az=-1, idx_alt=-1;
    for (size_t i=0;i<headers.size();i++) {
        std::string h = headers[i];
        if (h=="time"||h=="TIMESTAMP") idx_ts=i;
        else if(h=="accelx"||h=="ACCELEROMETER_X") idx_ax=i;
        else if(h=="accely"||h=="ACCELEROMETER_Y") idx_ay=i;
        else if(h=="accelz"||h=="ACCELEROMETER_Z") idx_az=i;
        else if(h=="altitude"||h=="ALTITUDE") idx_alt=i;
    }
    TEST_ASSERT_TRUE_MESSAGE(idx_ts>=0 && idx_ax>=0 && idx_ay>=0 && idx_az>=0 && idx_alt>=0, "Missing CSV columns");

    float prev_alt=0.0f; uint32_t prev_ts=0;
    while (std::getline(file,line)) {
        std::stringstream ss(line); std::string token; std::vector<std::string> tokens;
        while(std::getline(ss,token,',')) tokens.push_back(token);
        uint32_t ts; float ax,ay,az,alt;
        if(!parseCsvRow(tokens, idx_ts, idx_ax, idx_ay, idx_az, idx_alt, ts, ax, ay, az, alt)) continue;

        float vertical_vel = computeVerticalVelocity(prev_alt, alt, prev_ts, ts);
        prev_alt=alt; prev_ts=ts;
        AccelerationTriplet accel = buildAccelerationTriplet(ts, ax, ay, az);
        vve.update(accel, DataPoint(ts, alt));
        (apo.*predictionMethod)();
        writeCsvRow(csv, ts, alt, vertical_vel, vve, apo, az);
    }

    file.close(); csv.close();
}

/* ---------- Test 3: Multiple CSVs ---------- */
void test_apogee_predictor_with_multiple_csvs(void (ApogeePredictor::*predictionMethod)(void))
{
    const char* files[] = {
        "data/MARTHA_3-8_1.3_B2_SingleID_transformed.csv",
        "data/MARTHA_IREC_2025_B2_transformed.csv",
        "data/AA Data Collection - Second Launch Trimmed.csv"
    };

    for(const char* filename: files) {
        std::ifstream file(filename);
        TEST_ASSERT_TRUE_MESSAGE(file.is_open(), ("Failed to open CSV: "+std::string(filename)).c_str());

        std::ofstream csv("apogee_prediction_"+std::string(filename).substr(5));
        SetupOutputCSV(csv);

        VerticalVelocityEstimator vve;
        ApogeePredictor apo(vve,0.2f,0.5f);
        float prev_alt=0.0f; uint32_t prev_ts=0;
        float true_apogee=0.0f; uint32_t apogee_ts=0;
        std::vector<std::pair<uint32_t,float>> predicted_apogees;

        std::string line; std::getline(file,line);
        while(std::getline(file,line)) {
            std::stringstream ss(line); std::string token; std::vector<std::string> tokens;
            while(std::getline(ss,token,',')) tokens.push_back(token);
            uint32_t ts; float ax,ay,az,alt;
            if(!parseCsvRow(tokens,0,1,2,3,10,ts,ax,ay,az,alt)) continue;

            if(alt>true_apogee){ true_apogee=alt; apogee_ts=ts; }
            float vertical_vel=computeVerticalVelocity(prev_alt,alt,prev_ts,ts);
            prev_alt=alt; prev_ts=ts;

            AccelerationTriplet accel = buildAccelerationTriplet(ts,ax,ay,az);
            vve.update(accel,DataPoint(ts,alt));
            (apo.*predictionMethod)();
            writeCsvRow(csv,ts,alt,vertical_vel,vve,apo,az);

            if(apo.isPredictionValid()) predicted_apogees.emplace_back(ts,apo.getPredictedApogeeAltitude_m());
        }

        // Find first prediction within N feet of true apogee
        int32_t ts_diff = 0;
        const float meters_to_feet = 3.28084f;
        bool found = false;
        //tolerance for true apogee (0.01 =  1% of true apogee)
        const float tolerance_feet = true_apogee * 0.01f;
        for (const auto& dp : predicted_apogees)
        {
            float diff_feet = std::abs(dp.second - true_apogee) * meters_to_feet;
            if (diff_feet <= tolerance_feet)
            {
                ts_diff = apogee_ts - dp.first;
                found = true;
                break;
            }
        }

        if (found) {
            printf("File: %s | Predicted within %.1f ft of true apogee %.2f m, %d ms before actual apogee\n",
                   filename, tolerance_feet, true_apogee, ts_diff);
        } else {
            printf("File: %s | Predictor never got within %.1f ft of true apogee %.2f m\n",
                   filename, tolerance_feet, true_apogee);
        }

        file.close(); csv.close();
    }
}

/* ---------- Test 4: IREC CSV 15s early ---------- */
void test_apogee_predictor_with_irec_csv_15s_early(void (ApogeePredictor::*predictionMethod)(void))
{
    const char* filename = "data/MARTHA_IREC_2025_B2_transformed.csv";
    std::ifstream file(filename);
    TEST_ASSERT_TRUE_MESSAGE(file.is_open(), "Failed to open IREC CSV file");

    std::ofstream csv("apogee_prediction_IREC.csv");
    SetupOutputCSV(csv);

    VerticalVelocityEstimator vve;
    ApogeePredictor apo(vve, 0.2f, 0.5f);

    float prev_alt = 0.0f;
    uint32_t prev_ts = 0;
    float true_apogee = 0.0f;
    uint32_t apogee_ts = 0;
    std::vector<std::pair<uint32_t, float>> predicted_apogees;

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) tokens.push_back(token);

        uint32_t ts; float ax, ay, az, alt;
        if (!parseCsvRow(tokens, 0, 1, 2, 3, 10, ts, ax, ay, az, alt)) continue;

        if (alt > true_apogee) { true_apogee = alt; apogee_ts = ts; }
        float vertical_vel = computeVerticalVelocity(prev_alt, alt, prev_ts, ts);
        prev_alt = alt; prev_ts = ts;

        AccelerationTriplet accel = buildAccelerationTriplet(ts, ax, ay, az);
        vve.update(accel, DataPoint(ts, alt));
        (apo.*predictionMethod)();

        writeCsvRow(csv, ts, alt, vertical_vel, vve, apo, az);
        predicted_apogees.emplace_back(ts, apo.isPredictionValid() ? apo.getPredictedApogeeAltitude_m() : 0.0f);
    }

    file.close();
    csv.close();

    const float tolerance_m = true_apogee * 0.01f;
    const uint32_t required_early_ms = 15000;
    bool passed = false;
    int32_t early_ms = 0;
    for (const auto& dp : predicted_apogees) {
        float error = fabs(dp.second - true_apogee);
        int32_t time_before_apogee = (int32_t)(apogee_ts - dp.first);
        if (error <= tolerance_m && time_before_apogee >= required_early_ms) {
            passed = true;
            early_ms = time_before_apogee;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(passed, "Apogee predictor did not predict within tolerance at least 15 seconds before apogee");
}

/* ---------- Test 5: Real CSV alt every other value ---------- */
void test_apogee_predictor_with_real_csv_alt_every_other(void (ApogeePredictor::*predictionMethod)(void))
{
    std::ifstream file(dataset);
    TEST_ASSERT_TRUE_MESSAGE(file.is_open(), "Failed to open CSV file");

    std::ofstream csv("apogee_prediction_alt_every_other.csv");
    SetupOutputCSV(csv);

    VerticalVelocityEstimator vve;
    ApogeePredictor apo(vve,0.2f,0.5f);
    CircularArray<DataPoint,10> predicted_apogees(10);

    std::string line; std::getline(file,line);
    std::stringstream header_ss(line); std::string col; std::vector<std::string> headers;
    while(std::getline(header_ss,col,',')) headers.push_back(col);

    int idx_ts=-1, idx_ax=-1, idx_ay=-1, idx_az=-1, idx_alt=-1;
    for(size_t i=0;i<headers.size();i++){
        std::string h=headers[i];
        if(h=="time"||h=="TIMESTAMP") idx_ts=i;
        else if(h=="accelx"||h=="ACCELEROMETER_X") idx_ax=i;
        else if(h=="accely"||h=="ACCELEROMETER_Y") idx_ay=i;
        else if(h=="accelz"||h=="ACCELEROMETER_Z") idx_az=i;
        else if(h=="altitude"||h=="ALTITUDE") idx_alt=i;
    }
    TEST_ASSERT_TRUE_MESSAGE(idx_ts>=0 && idx_ax>=0 && idx_ay>=0 && idx_az>=0 && idx_alt>=0,"Missing CSV columns");

    float prev_alt=0.0f; uint32_t prev_ts=0;
    int loop_count=0; DataPoint altDp;
    while(std::getline(file,line)){
        std::stringstream ss(line); std::string token; std::vector<std::string> tokens;
        while(std::getline(ss,token,',')) tokens.push_back(token);
        uint32_t ts; float ax,ay,az,alt;
        if(!parseCsvRow(tokens,idx_ts,idx_ax,idx_ay,idx_az,idx_alt,ts,ax,ay,az,alt)) continue;

        float vertical_vel=computeVerticalVelocity(prev_alt,alt,prev_ts,ts);
        prev_alt=alt; prev_ts=ts;
        AccelerationTriplet accel = buildAccelerationTriplet(ts,ax,ay,az);

        if(loop_count%2==0){ altDp.data=alt; altDp.timestamp_ms=ts; }
        vve.update(accel,altDp);
        (apo.*predictionMethod)();

        writeCsvRow(csv,ts,alt,vertical_vel,vve,apo,az);
        loop_count++;
    }
    file.close(); csv.close();
}

/* ---------- Test 6: All methods ---------- */
void test_apogee_predictor_all_methods_timeseries(void)
{
    std::ifstream file(dataset);
    TEST_ASSERT_TRUE_MESSAGE(file.is_open(), "Failed to open CSV file");

    std::ofstream csv("apogee_prediction_all_methods.csv");
    TEST_ASSERT_TRUE(csv.is_open());

    csv << "timestamp,true_alt,true_vertical_velocity,"
           "est_alt,est_vertical_velocity,true_acl,est_acl,cd,"
           "analytic_apogee,simulate_apogee,quad_apogee,poly_apogee,default_apogee\n";

    // ✅ Independent estimators
    VerticalVelocityEstimator vve_analytic;
    VerticalVelocityEstimator vve_simulate;
    VerticalVelocityEstimator vve_quad;
    VerticalVelocityEstimator vve_poly;
    VerticalVelocityEstimator vve_default;

    // ✅ Independent predictors
    ApogeePredictor apo_analytic(vve_analytic, 0.2f, 0.5f);
    ApogeePredictor apo_simulate(vve_simulate, 0.2f, 0.5f);
    ApogeePredictor apo_quad(vve_quad, 0.2f, 0.5f);
    ApogeePredictor apo_poly(vve_poly, 0.2f, 0.5f);
    ApogeePredictor apo_default(vve_default, 0.2f, 0.5f);

    // ✅ True apogee tracking
    float true_apogee = 0.0f;
    uint32_t apogee_ts = 0;

    struct Prediction { uint32_t ts; float val; };

    std::vector<Prediction> analytic_preds;
    std::vector<Prediction> simulate_preds;
    std::vector<Prediction> quad_preds;
    std::vector<Prediction> poly_preds;
    std::vector<Prediction> default_preds;

    std::string line;
    std::getline(file, line); // skip header

    float prev_alt = 0.0f;
    uint32_t prev_ts = 0;

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) tokens.push_back(token);

        uint32_t ts;
        float ax, ay, az, alt;

        if (!parseCsvRow(tokens, 0, 1, 2, 3, 10, ts, ax, ay, az, alt)) continue;

        // ✅ Track true apogee
        if (alt > true_apogee) {
            true_apogee = alt;
            apogee_ts = ts;
        }

        float vertical_vel = computeVerticalVelocity(prev_alt, alt, prev_ts, ts);
        prev_alt = alt;
        prev_ts = ts;

        AccelerationTriplet accel = buildAccelerationTriplet(ts, ax, ay, az);
        DataPoint altDp(ts, alt);

        // ✅ Update each estimator independently
        vve_analytic.update(accel, altDp);
        vve_simulate.update(accel, altDp);
        vve_quad.update(accel, altDp);
        vve_poly.update(accel, altDp);
        vve_default.update(accel, altDp);

        // ✅ Run each method
        apo_analytic.analytic_update();
        apo_simulate.simulate_update();
        apo_quad.quad_update();
        apo_poly.poly_update();
        apo_default.update();

        // ✅ Store predictions (FILTER BAD SPIKES)
        auto push_if_valid = [&](ApogeePredictor& apo, std::vector<Prediction>& vec) {
            if (apo.isPredictionValid()) {
                float pred = apo.getPredictedApogeeAltitude_m();

                // Filter unrealistic early spikes
                if (pred > alt) {
                    vec.push_back({ts, pred});
                }
            }
        };

        push_if_valid(apo_analytic, analytic_preds);
        push_if_valid(apo_simulate, simulate_preds);
        push_if_valid(apo_quad, quad_preds);
        push_if_valid(apo_poly, poly_preds);
        push_if_valid(apo_default, default_preds);

        // CSV output
        auto getApogee = [](ApogeePredictor& apo) {
            return apo.isPredictionValid() ? apo.getPredictedApogeeAltitude_m() : NAN;
        };

        csv << ts << ','
            << alt << ','
            << vertical_vel << ','
            << vve_default.getEstimatedAltitude() << ','
            << vve_default.getEstimatedVelocity() << ','
            << az << ','
            << vve_default.getInertialVerticalAcceleration() << ','
            << apo_default.getdragCoefficient() << ','
            << getApogee(apo_analytic) << ','
            << getApogee(apo_simulate) << ','
            << getApogee(apo_quad) << ','
            << getApogee(apo_poly) << ','
            << getApogee(apo_default)
            << '\n';
    }

    file.close();
    csv.close();

    // ✅ Evaluation
    const float tolerance = true_apogee * 0.01f;
    const int N = 5; // consecutive stable points required

    auto evaluate_method = [&](const std::vector<Prediction>& preds, const char* name)
    {
        for (size_t i = 0; i + N < preds.size(); i++)
        {
            bool stable = true;

            for (int j = 0; j < N; j++)
            {
                float error = fabs(preds[i + j].val - true_apogee);
                if (error > tolerance)
                {
                    stable = false;
                    break;
                }
            }

            if (stable)
            {
                int32_t early_ms = (int32_t)(apogee_ts - preds[i].ts);
                printf("%s: STABLE prediction %d ms before apogee\n", name, early_ms);
                return;
            }
        }

        printf("%s: FAILED (no stable prediction)\n", name);
    };

    printf("\n===== APOGEE PREDICTION COMPARISON =====\n");
    printf("True apogee: %.2f m at %u ms\n\n", true_apogee, apogee_ts);

    evaluate_method(analytic_preds, "Analytic");
    evaluate_method(simulate_preds, "Simulate");
    evaluate_method(quad_preds, "Quadratic");
    evaluate_method(poly_preds, "Polynomial");
    evaluate_method(default_preds, "Default");
}

//------------------- wrapper functions -------------------
void test_apogee_predictor_generates_csv(void){
    test_apogee_predictor_generates_csv(predictionMethod);
}

void test_apogee_predictor_with_multiple_csvs(void){
    test_apogee_predictor_with_multiple_csvs(predictionMethod);
}

void test_apogee_predictor_with_real_csv(void){
    test_apogee_predictor_with_real_csv(predictionMethod);
}

void test_apogee_predictor_with_irec_csv_15s_early(void){
    test_apogee_predictor_with_irec_csv_15s_early(predictionMethod);
}

void test_apogee_predictor_with_real_csv_alt_every_other(void){
    test_apogee_predictor_with_real_csv_alt_every_other(predictionMethod);
}


/* ---------- main runner ----------------------- */
int main(void)
{
    UNITY_BEGIN();

    //choose a test or tests to run

    //RUN_TEST(test_apogee_predictor_generates_csv);
    //RUN_TEST(test_apogee_predictor_with_real_csv);
    //RUN_TEST(test_apogee_predictor_with_multiple_csvs);
    RUN_TEST(test_apogee_predictor_with_irec_csv_15s_early);
    //RUN_TEST(test_apogee_predictor_with_real_csv_alt_every_other);
    //RUN_TEST(test_apogee_predictor_all_methods_timeseries);
    return UNITY_END();
}