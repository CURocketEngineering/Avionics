#include "unity.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataPoint.h"

DataSaverSPI* dss;
Adafruit_SPIFlash* flash;

void setUp(void) {
    flash = new Adafruit_SPIFlash();
    dss = new DataSaverSPI(100, flash);
}

void tearDown(void) {
    delete dss;
    delete flash;
}

void test_save_data_point(void) {
    DataPoint dp(500, 1.0);
    int result = dss->saveDataPoint(dp, 1);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL_UINT32(500, dss->getLastTimestamp());
    TEST_ASSERT_EQUAL_UINT32(500, dss->getLastDataPoint().timestamp_ms);
    TEST_ASSERT_EQUAL_FLOAT(1.0, dss->getLastDataPoint().data);

    DataPoint dp2(550, 2.0); // Not enough time has passed
    result = dss->saveDataPoint(dp2, 1);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL_UINT32(500, dss->getLastTimestamp()); // The timestamp should not have changed
    TEST_ASSERT_EQUAL_UINT32(550, dss->getLastDataPoint().timestamp_ms); // The data point should have changed
    TEST_ASSERT_EQUAL_FLOAT(2.0, dss->getLastDataPoint().data);
}

void test_flush_buffer(void) {
    dss->clearInternalState();
    TEST_ASSERT_EQUAL(0, dss->getBufferIndex());
    DataPoint dp(500, 1.0);
    dss->saveDataPoint(dp, 1); // Saves a timestamp and data point (10 bytes)
    int expectedBytesBuffer = 10;
    TEST_ASSERT_EQUAL(expectedBytesBuffer, dss->getBufferIndex());
    TEST_ASSERT_EQUAL(0, dss->getBufferFlushes());

    

    int hits_to_flush = (dss->BUFFER_SIZE - 10) / 5 + 1; // + 1 to trigger flush
    for (int i = 0; i < hits_to_flush; i++) {
        DataPoint dp(500, 1.0);
        dss->saveDataPoint(dp, 1);
        expectedBytesBuffer += 5;
        if (expectedBytesBuffer >= dss->BUFFER_SIZE) {
            expectedBytesBuffer = 5;
        }
        TEST_ASSERT_EQUAL(expectedBytesBuffer, dss->getBufferIndex());
    }

    TEST_ASSERT_EQUAL(1, dss->getBufferFlushes());
}

void test_clearplm_next_write(void){
    // Write some data points to flash to move the nextWriteAddress forward
    for (int i = 0; i < 50; i++) {
        DataPoint dp(500 + i * 100, 1.0);
        dss->saveDataPoint(dp, 1);
    }

    // Capture the nextWriteAddress before clearing post-launch mode
    uint32_t nextWriteBefore = dss->getNextWriteAddress();
    dss->clearPostLaunchMode();
    uint32_t nextWriteAfter = dss->getNextWriteAddress();
    TEST_ASSERT_EQUAL_UINT32(nextWriteBefore, nextWriteAfter);
}

void test_erase_all_data(void) {
    dss->eraseAllData();
    TEST_ASSERT_EQUAL_UINT32(DATA_START_ADDRESS, dss->getNextWriteAddress());
    TEST_ASSERT_EQUAL_UINT32(0, dss->getLastTimestamp());
    TEST_ASSERT_EQUAL_UINT32(0, dss->getLastDataPoint().timestamp_ms);
    TEST_ASSERT_EQUAL_FLOAT(0.0, dss->getLastDataPoint().data);
}

void test_launch_detected(void) {
    dss->launchDetected(1000);
    TEST_ASSERT_TRUE(dss->quickGetPostLaunchMode());
    TEST_ASSERT_NOT_EQUAL(0, dss->getLaunchWriteAddress());
}

void test_record_size(void) {
    Record_t record = {1, 2.0f};
    TEST_ASSERT_EQUAL(5, sizeof(record)); // 1 byte for name, 4 bytes for data
}

void test_timestamp_record_size(void) {
    TimestampRecord_t record = {1, 1000};
    TEST_ASSERT_EQUAL(5, sizeof(record)); // 1 byte for name, 4 bytes for timestamp
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_record_size);
    RUN_TEST(test_timestamp_record_size);
    RUN_TEST(test_save_data_point);
    RUN_TEST(test_flush_buffer);
    RUN_TEST(test_clearplm_next_write);
    RUN_TEST(test_erase_all_data);
    RUN_TEST(test_launch_detected);
    return UNITY_END();
}