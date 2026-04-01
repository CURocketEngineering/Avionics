#include "unity.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataPoint.h"
#include <cstddef>
#include <cstdint>

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
    DataPoint dp(500U, 1.0f);
    int result = dss->saveDataPoint(dp, 1);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL_UINT32(500, dss->getLastTimestamp());
    TEST_ASSERT_EQUAL_UINT32(500, dss->getLastDataPoint().timestamp_ms);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, dss->getLastDataPoint().data);

    DataPoint dp2(550U, 2.0f); // Not enough time has passed
    result = dss->saveDataPoint(dp2, 1);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL_UINT32(500, dss->getLastTimestamp()); // The timestamp should not have changed
    TEST_ASSERT_EQUAL_UINT32(550, dss->getLastDataPoint().timestamp_ms); // The data point should have changed
    TEST_ASSERT_EQUAL_FLOAT(2.0f, dss->getLastDataPoint().data);
}

void test_flush_buffer(void) {
    dss->clearInternalState();
    TEST_ASSERT_EQUAL(0, dss->getBufferIndex());
    DataPoint dp(500U, 1.0f);
    dss->saveDataPoint(dp, 1); // Saves a timestamp and data point (10 bytes)
    size_t expectedBufferSize_bytes = 10;
    TEST_ASSERT_EQUAL(expectedBufferSize_bytes, dss->getBufferIndex());
    TEST_ASSERT_EQUAL(0, dss->getBufferFlushes());

    

    size_t hitsToFlush = (DataSaverSPI::kBufferSize_bytes - 10) / 5 + 1; // + 1 to trigger flush
    for (size_t i = 0; i < hitsToFlush; i++) {
        DataPoint dpLoop(500U, 1.0f);
        dss->saveDataPoint(dpLoop, 1);
        expectedBufferSize_bytes += 5;
        if (expectedBufferSize_bytes >= DataSaverSPI::kBufferSize_bytes) {
            expectedBufferSize_bytes = 5;
        }
        TEST_ASSERT_EQUAL(expectedBufferSize_bytes, dss->getBufferIndex());
    }

    TEST_ASSERT_EQUAL(1, dss->getBufferFlushes());
}

void test_clearplm_next_write(void){
    // Write some data points to flash to move the nextWriteAddress forward
    for (uint32_t i = 0; i < 50U; i++) {
        DataPoint dp(500U + i * 100U, 1.0f);
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
    TEST_ASSERT_EQUAL_UINT32(kDataStartAddress, dss->getNextWriteAddress());
    TEST_ASSERT_EQUAL_UINT32(0, dss->getLastTimestamp());
    TEST_ASSERT_EQUAL_UINT32(0, dss->getLastDataPoint().timestamp_ms);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dss->getLastDataPoint().data);
}

void test_launch_detected(void) {
    dss->launchDetected(1000);
    TEST_ASSERT_TRUE(dss->quickGetPostLaunchMode());
    TEST_ASSERT_NOT_EQUAL(0, dss->getLaunchWriteAddress());
}

void test_next_sector_is_erased_before_crossing_boundary(void) {
    for (size_t i = 0; i < FAKE_MEMORY_SIZE_BYTES; i++) {
        flash->fakeMemory[i] = 0x00;
    }
    dss->clearInternalState();

    size_t const recordsPerFlush = DataSaverSPI::kBufferSize_bytes / sizeof(TimestampRecord_t);
    uint32_t flushesToSectorBoundary = 0;
    uint32_t expectedBoundaryAddress = kDataStartAddress;
    do {
        expectedBoundaryAddress += DataSaverSPI::kBufferSize_bytes;
        if (expectedBoundaryAddress >= FAKE_MEMORY_SIZE_BYTES) {
            expectedBoundaryAddress = kDataStartAddress;
        }
        flushesToSectorBoundary++;
    } while (expectedBoundaryAddress % SFLASH_SECTOR_SIZE != 0U);

    // +1 triggers the Nth flush by overflowing a full buffer with one more record.
    uint32_t const writesNeeded = static_cast<uint32_t>(flushesToSectorBoundary * recordsPerFlush) + 1U;
    for (uint32_t i = 0; i < writesNeeded; i++) {
        TEST_ASSERT_EQUAL(0, dss->saveTimestamp(i));
    }

    TEST_ASSERT_EQUAL_UINT32(flushesToSectorBoundary, dss->getBufferFlushes());
    TEST_ASSERT_EQUAL_HEX8(0xFF, flash->fakeMemory[expectedBoundaryAddress]);
}

void test_pre_erase_skips_protected_launch_sector(void) {
    for (size_t i = 0; i < FAKE_MEMORY_SIZE_BYTES; i++) {
        flash->fakeMemory[i] = 0x00;
    }
    dss->clearInternalState();

    uint32_t const protectedSectorNumber = kDataStartAddress / SFLASH_SECTOR_SIZE + 1U;
    uint32_t const protectedSectorStartAddress = protectedSectorNumber * SFLASH_SECTOR_SIZE;
    uint32_t const launchWriteAddress = protectedSectorStartAddress + 3000U;
    uint32_t const writeAddressBeforeBoundary = protectedSectorStartAddress - DataSaverSPI::kBufferSize_bytes;

    flash->fakeMemory[launchWriteAddress] = 0x5A;
    dss->setPostLaunchStateForTest(writeAddressBeforeBoundary, launchWriteAddress, true);

    uint32_t const recordsPerFlush = DataSaverSPI::kBufferSize_bytes / sizeof(TimestampRecord_t);

    int result = 0;
    for (uint32_t i = 0; i < recordsPerFlush + 1U; i++) {
        result = dss->saveTimestamp(i);
    }

    // This flush writes successfully; only pre-erase is skipped for the
    // protected next sector.
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_FALSE(dss->getIsChipFullDueToPostLaunchProtection());
    TEST_ASSERT_EQUAL_HEX8(0x5A, flash->fakeMemory[launchWriteAddress]);

    for (uint32_t i = 0; i < recordsPerFlush; i++) {
        result = dss->saveTimestamp(1000U + i);
    }

    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_TRUE(dss->getIsChipFullDueToPostLaunchProtection());
    TEST_ASSERT_EQUAL_HEX8(0x5A, flash->fakeMemory[launchWriteAddress]);
}

void test_flush_wraps_using_full_page_write_size(void) {
    dss->clearInternalState();
    TEST_ASSERT_EQUAL(0, dss->saveTimestamp(1U));

    uint32_t const nearEndAddress = FAKE_MEMORY_SIZE_BYTES - DataSaverSPI::kBufferSize_bytes + 1U;
    dss->setPostLaunchStateForTest(nearEndAddress, 0U, false);

    int result = 0;
    uint32_t const recordsPerFlush = DataSaverSPI::kBufferSize_bytes / sizeof(TimestampRecord_t);
    for (uint32_t i = 0; i < recordsPerFlush; i++) {
        result = dss->saveTimestamp(2U + i);
    }

    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL_UINT32(1U, dss->getBufferFlushes());
    TEST_ASSERT_EQUAL_UINT32(kDataStartAddress + DataSaverSPI::kBufferSize_bytes, dss->getNextWriteAddress());
}

void test_record_size(void) {
    Record_t record = {1, 2.0f};
    TEST_ASSERT_EQUAL(5, sizeof(record)); // 1 byte for name, 4 bytes for data
}

void test_timestamp_record_size(void) {
    TimestampRecord_t record = {1, 1000};
    TEST_ASSERT_EQUAL(5, sizeof(record)); // 1 byte for name, 4 bytes for timestamp
}


int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_record_size);
    RUN_TEST(test_timestamp_record_size);
    RUN_TEST(test_save_data_point);
    RUN_TEST(test_flush_buffer);
    RUN_TEST(test_clearplm_next_write);
    RUN_TEST(test_erase_all_data);
    RUN_TEST(test_launch_detected);
    RUN_TEST(test_next_sector_is_erased_before_crossing_boundary);
    RUN_TEST(test_pre_erase_skips_protected_launch_sector);
    RUN_TEST(test_flush_wraps_using_full_page_write_size);
    return UNITY_END();
}
