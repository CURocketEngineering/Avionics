#include "unity.h"
#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include "ArduinoHAL.h"  // Optional: Needed if SensorDataHandler or DataSaver use Arduino specific API

#include <vector>

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

// ---------------------------------------------------------------------
// Mock IDataSaver Implementation
// ---------------------------------------------------------------------
class MockDataSaver : public IDataSaver {
public:
    struct SavedRecord {
        DataPoint data;
        uint8_t sensorName;
    };

    std::vector<SavedRecord> savedRecords;

    // This method will be called by SensorDataHandler.
    virtual int saveDataPoint(const DataPoint& data, uint8_t sensorName) override {
        SavedRecord record = { data, sensorName };
        savedRecords.push_back(record);
        return 0;
    }

    // Resets the record for reuse between tests.
    void reset() {
        savedRecords.clear();
    }
};

// ---------------------------------------------------------------------
// Helper: Create a data point with a given timestamp and value.
// ---------------------------------------------------------------------
DataPoint makeData(uint32_t timestamp, float value = 1.0f) {
    return DataPoint(timestamp, value);
}

// ---------------------------------------------------------------------
// Test Cases
// ---------------------------------------------------------------------

/**
 * Test that when no save interval is set (default saveInterval_ms is 0),
 * each call to addData() results in a call to IDataSaver::saveDataPoint.
 */
void test_addData_without_interval_restriction(void) {
    MockDataSaver mockSaver;
    uint8_t sensorName = 1;
    SensorDataHandler sdh(sensorName, &mockSaver);

    // With default saveInterval_ms (0), every call should trigger a save.
    DataPoint dp1 = makeData(1000, 1.0f);
    DataPoint dp2 = makeData(1001, 2.0f);
    DataPoint dp3 = makeData(1002, 3.0f);

    sdh.addData(dp1);
    sdh.addData(dp2);
    sdh.addData(dp3);

    TEST_ASSERT_EQUAL_UINT32(3, mockSaver.savedRecords.size());
    // Verify that the saved sensor name matches.
    for (auto& record : mockSaver.savedRecords) {
        TEST_ASSERT_EQUAL_UINT8(sensorName, record.sensorName);
    }
}

/**
 * Test that with a save interval configured via restrictSaveSpeed(),
 * addData() only calls saveDataPoint() when the data timestamp advances 
 * by more than the specified interval.
 */
void test_addData_with_save_interval(void) {
    MockDataSaver mockSaver;
    uint8_t sensorName = 2;
    SensorDataHandler sdh(sensorName, &mockSaver);

    // Restrict saving so that at least 50 ms must pass between saves.
    sdh.restrictSaveSpeed(50);

    // First call: timestamp 1000. This should be saved.
    DataPoint dp1 = makeData(1000, 1.0f);
    sdh.addData(dp1);
    TEST_ASSERT_EQUAL_UINT32(1, mockSaver.savedRecords.size());

    // Second call: timestamp 1020. This difference (20 ms) is too small.
    DataPoint dp2 = makeData(1020, 2.0f);
    sdh.addData(dp2);
    TEST_ASSERT_EQUAL_UINT32(1, mockSaver.savedRecords.size());

    // Third call: timestamp 1051. Now, 51 ms have passed since last save.
    DataPoint dp3 = makeData(1051, 3.0f);
    sdh.addData(dp3);
    TEST_ASSERT_EQUAL_UINT32(2, mockSaver.savedRecords.size());

    // Fourth call: timestamp 1100, only 49 ms after the last saved timestamp (1051)
    DataPoint dp4 = makeData(1100, 4.0f);
    sdh.addData(dp4);
    TEST_ASSERT_EQUAL_UINT32(2, mockSaver.savedRecords.size());

    // Fifth call: timestamp 1102 (51 ms after last save at 1051 or 2 ms later than previous call),
    // should trigger another save.
    DataPoint dp5 = makeData(1102, 5.0f);
    sdh.addData(dp5);
    TEST_ASSERT_EQUAL_UINT32(3, mockSaver.savedRecords.size());
}

/**
 * Test that multiple calls with exactly the same timestamp do not bypass the interval check.
 *
 * For an interval larger than zero, if several data points come with the same timestamp,
 * only the first one should be saved.
 */
void test_multiple_data_same_timestamp(void) {
    MockDataSaver mockSaver;
    uint8_t sensorName = 3;
    SensorDataHandler sdh(sensorName, &mockSaver);

    // Set save interval to 20 ms.
    sdh.restrictSaveSpeed(20);

    // Multiple data points with the same timestamp.
    DataPoint dp1 = makeData(5000, 1.0f);
    DataPoint dp2 = makeData(5000, 2.0f);
    DataPoint dp3 = makeData(5000, 3.0f);

    sdh.addData(dp1);
    sdh.addData(dp2);
    sdh.addData(dp3);

    // Even though we called addData() three times,
    // only the first should be saved because 0 ms difference is not > 20.
    TEST_ASSERT_EQUAL_UINT32(1, mockSaver.savedRecords.size());
}

/**
 * Test that after a long delay the data is saved again.
 */
void test_long_delay_resets_save_timer(void) {
    MockDataSaver mockSaver;
    uint8_t sensorName = 4;
    SensorDataHandler sdh(sensorName, &mockSaver);

    // Set a save interval.
    sdh.restrictSaveSpeed(100);

    // First save at timestamp 1000.
    DataPoint dp1 = makeData(1000, 1.0f);
    sdh.addData(dp1);
    TEST_ASSERT_EQUAL_UINT32(1, mockSaver.savedRecords.size());

    // Second update at timestamp 1050, not enough time passed (only 50 ms)
    DataPoint dp2 = makeData(1050, 2.0f);
    sdh.addData(dp2);
    TEST_ASSERT_EQUAL_UINT32(1, mockSaver.savedRecords.size());

    // Update at timestamp 1200 (200 ms later), should be saved.
    DataPoint dp3 = makeData(1200, 3.0f);
    sdh.addData(dp3);
    TEST_ASSERT_EQUAL_UINT32(2, mockSaver.savedRecords.size());
}

// ---------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_addData_without_interval_restriction);
    RUN_TEST(test_addData_with_save_interval);
    RUN_TEST(test_multiple_data_same_timestamp);
    RUN_TEST(test_long_delay_resets_save_timer);
    return UNITY_END();
}
