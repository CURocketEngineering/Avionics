#include "unity.h"
#include <array>
#include <cstdio>
#include <vector>
#include "data_handling/Telemetry.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

// ---------------------------------------------------------------------
// Mock IDataSaver Implementation - copied from test_sensor_data_handler.cpp
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

void test_initialization(void) {
    MockDataSaver mockXAcl, mockYAcl, mockZAcl, altitude;
    uint8_t xAclName = 1;
    uint8_t yAclName = 2;
    uint8_t zAclName = 3;
    uint8_t altName = 4;
    SensorDataHandler xAclData(xAclName, &mockXAcl);
    SensorDataHandler yAclData(yAclName, &mockYAcl);
    SensorDataHandler zAclData(zAclName, &mockZAcl);
    SensorDataHandler altitudeData(altName, &altitude);

    std::array<SensorDataHandler*, 3> accelerationTriplet{};
    accelerationTriplet[0] = &xAclData;
    accelerationTriplet[1] = &yAclData;
    accelerationTriplet[2] = &zAclData;

    SendableSensorData accelerationSsd(accelerationTriplet, 102, 2.0f);
    SendableSensorData altitudeSsd(&altitudeData, 1.0f);
    std::array<SendableSensorData*, 2> ssds{&accelerationSsd, &altitudeSsd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);
}

//This does a lot of tests
void test_a_full_second_of_ticks(void) {
    MockDataSaver mockXAcl, mockYAcl, mockZAcl, altitude, packetCounter;
    uint8_t xAclName = 1;
    uint8_t yAclName = 2;
    uint8_t zAclName = 3;
    uint8_t altName = 8;
    uint8_t packetCounterName = 5;
    SensorDataHandler xAclData(xAclName, &mockXAcl);
    SensorDataHandler yAclData(yAclName, &mockYAcl);
    SensorDataHandler zAclData(zAclName, &mockZAcl);
    SensorDataHandler altitudeData(altName, &altitude);
    SensorDataHandler numberSentPackets(packetCounterName, &packetCounter);
    xAclData.addData(DataPoint(1, 6.767676f)); //01000000 11011000 10010000 11001101
    yAclData.addData(DataPoint(1, 6.969696f)); //01000000 11011111 00000111 11000000
    zAclData.addData(DataPoint(1, 1.234567f)); //00111111 10011110 00000110 01001011
    altitudeData.addData(DataPoint(1, 10000.0f)); //01000110 00011100 01000000 00000000
    numberSentPackets.addData(DataPoint(1, 1));
    uint8_t expectedSentBytes[63] = {   
        // FIRST PACKET
        0, 0, 0, 51,           // Start (4) - bytes 0-3
        0, 0, 1, 244,          // Timestamp 500 (4) - bytes 4-7
        0, 0, 0, 0,            // Counter 0 (4) - bytes 8-11
        102,                   // Label (1) - byte 12
        64, 216, 144, 205,     // X (4) - bytes 13-16
        64, 223, 7, 192,       // Y (4) - bytes 17-20
        63, 158, 6, 75,        // Z (4) - bytes 21-24
        0, 0, 0, 52,           // End (4) - bytes 25-28
        
        // SECOND PACKET
        0, 0, 0, 51,           // Start (4) - bytes 29-32
        0, 0, 3, 232,          // Timestamp 1000 (4) - bytes 33-36
        0, 0, 0, 1,            // Counter 1 (4) - bytes 37-40
        102,                   // Label (1) - byte 41
        64, 216, 144, 205,     // X (4) - bytes 42-45
        64, 223, 7, 192,       // Y (4) - bytes 46-49
        63, 158, 6, 75,        // Z (4) - bytes 50-53
        8,                     // Altitude label (1) - byte 54
        70, 28, 64, 0,         // Altitude (4) - bytes 55-58
        0, 0, 0, 52,           // End (4) - bytes 59-62
};

    std::array<SensorDataHandler*, 3> accelerationTriplet{};
    accelerationTriplet[0] = &xAclData;
    accelerationTriplet[1] = &yAclData;
    accelerationTriplet[2] = &zAclData;

    SendableSensorData accelerationSsd(accelerationTriplet, 102, 2.0f);
    SendableSensorData altitudeSsd(&altitudeData, 1.0f);
    std::array<SendableSensorData*, 2> ssds{&accelerationSsd, &altitudeSsd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)500), true);
    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)1000), true);

    printf("\n");
    for (uint8_t byte : mockRfdSerial.writeCalls) {
        printf(" %03d", byte);
    }
    printf("\n");
    printf("\n");
    for (uint8_t byte : expectedSentBytes) {
        printf(" %03d", byte);
    }
    printf("\n");
    // Test all bytes sent correctly for first second
    for (std::size_t i = 0; i < 51U; ++i) {
        char message[50];
        std::snprintf(message, sizeof(message), "Byte %zu mismatch", i);
        TEST_ASSERT_EQUAL_MESSAGE(expectedSentBytes[i], mockRfdSerial.writeCalls.at(i), message);
    }

    printf("2 SECONDS:\n");
    mockRfdSerial.clearWriteCalls();
    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)1500), true);
    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)2000), true);

    printf("\n");
    for (int byte : mockRfdSerial.writeCalls) {
        printf(" %03d", byte);
    }
    printf("\n");
    printf("\n");
    for (int byte : expectedSentBytes) {
        printf(" %03d", byte);
    }
    printf("\n");
    // Test all bytes sent correctly for second second
    for (std::size_t i = 0; i < 51U; ++i) {
        // Skip timestamp bytes
        if (i >= 4U && i <= 11U) {
            continue;
        }
        if (i >= 33U && i <= 40U) {
            continue;
        }


        char message[50];
        std::snprintf(message, sizeof(message), "Byte %zu mismatch", i);
        TEST_ASSERT_EQUAL_MESSAGE(expectedSentBytes[i], mockRfdSerial.writeCalls.at(i),
                                    message);
    }
}

void test_first_packet_counter_is_zero(void) {
    MockDataSaver mockXAcl, mockYAcl, mockZAcl;
    SensorDataHandler xAclData(1, &mockXAcl);
    SensorDataHandler yAclData(2, &mockYAcl);
    SensorDataHandler zAclData(3, &mockZAcl);

    xAclData.addData(DataPoint(1, 6.767676f));
    yAclData.addData(DataPoint(1, 6.969696f));
    zAclData.addData(DataPoint(1, 1.234567f));

    std::array<SensorDataHandler*, 3> accelerationTriplet{&xAclData, &yAclData, &zAclData};
    SendableSensorData accelerationSsd(accelerationTriplet, 102, 2.0f);
    std::array<SendableSensorData*, 1> ssds{&accelerationSsd};

    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);
    telemetry.tick((uint32_t)500);

    // kPacketCounterIndex = 8, counter should be 0x00000000
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(TelemetryFmt::kPacketCounterIndex));
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(TelemetryFmt::kPacketCounterIndex + 1));
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(TelemetryFmt::kPacketCounterIndex + 2));
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(TelemetryFmt::kPacketCounterIndex + 3));
}

void test_second_packet_counter_is_one(void) {
    MockDataSaver mockXAcl, mockYAcl, mockZAcl;
    SensorDataHandler xAclData(1, &mockXAcl);
    SensorDataHandler yAclData(2, &mockYAcl);
    SensorDataHandler zAclData(3, &mockZAcl);

    xAclData.addData(DataPoint(1, 6.767676f));
    yAclData.addData(DataPoint(1, 6.969696f));
    zAclData.addData(DataPoint(1, 1.234567f));

    std::array<SensorDataHandler*, 3> accelerationTriplet{&xAclData, &yAclData, &zAclData};
    SendableSensorData accelerationSsd(accelerationTriplet, 102, 2.0f);
    std::array<SendableSensorData*, 1> ssds{&accelerationSsd};

    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);
    telemetry.tick((uint32_t)500);
    telemetry.tick((uint32_t)1000);

    // First packet: kHeaderSize_bytes(12) + label(1) + 3 floats(12) + end marker(4) = 29 bytes
    // Second packet counter starts at byte 29 + kPacketCounterIndex
    const std::size_t secondPacketStart = 29U;
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex));
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex + 1));
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex + 2));
    TEST_ASSERT_EQUAL(1, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex + 3));
}

// =====================================================================
// hzToPeriod_ms float frequency tests
// =====================================================================

void test_hz_to_period_integer_values(void) {
    // Existing behavior should be preserved with float input
    TEST_ASSERT_EQUAL_UINT16(500, TelemetryFmt::hzToPeriod_ms(2.0f));   // 2 Hz → 500ms
    TEST_ASSERT_EQUAL_UINT16(1000, TelemetryFmt::hzToPeriod_ms(1.0f));  // 1 Hz → 1000ms
    TEST_ASSERT_EQUAL_UINT16(100, TelemetryFmt::hzToPeriod_ms(10.0f));  // 10 Hz → 100ms
    TEST_ASSERT_EQUAL_UINT16(50, TelemetryFmt::hzToPeriod_ms(20.0f));   // 20 Hz → 50ms
}

void test_hz_to_period_fractional_values(void) {
    TEST_ASSERT_EQUAL_UINT16(2000, TelemetryFmt::hzToPeriod_ms(0.5f));   // 0.5 Hz → 2000ms
    TEST_ASSERT_EQUAL_UINT16(10000, TelemetryFmt::hzToPeriod_ms(0.1f));  // 0.1 Hz → 10000ms
    TEST_ASSERT_EQUAL_UINT16(4000, TelemetryFmt::hzToPeriod_ms(0.25f));  // 0.25 Hz → 4000ms
    TEST_ASSERT_EQUAL_UINT16(5000, TelemetryFmt::hzToPeriod_ms(0.2f));   // 0.2 Hz → 5000ms
}

void test_hz_to_period_zero_returns_fallback(void) {
    TEST_ASSERT_EQUAL_UINT16(1000, TelemetryFmt::hzToPeriod_ms(0.0f));
}

void test_hz_to_period_negative_returns_fallback(void) {
    TEST_ASSERT_EQUAL_UINT16(1000, TelemetryFmt::hzToPeriod_ms(-5.0f));
}

void test_hz_to_period_very_slow_rate_clamps_to_max(void) {
    // 0.01 Hz → 100000ms, but uint16_t max is 65535
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, TelemetryFmt::hzToPeriod_ms(0.01f));
}

void test_hz_to_period_ceil_semantics(void) {
    // 3 Hz → 1000/3 = 333.33... → should ceil to 334
    TEST_ASSERT_EQUAL_UINT16(334, TelemetryFmt::hzToPeriod_ms(3.0f));
    // 7 Hz → 1000/7 = 142.857... → should ceil to 143
    TEST_ASSERT_EQUAL_UINT16(143, TelemetryFmt::hzToPeriod_ms(7.0f));
}

void test_sendable_sensor_data_fractional_hz_period(void) {
    MockDataSaver mock;
    SensorDataHandler sdh(1, &mock);
    sdh.addData(DataPoint(1, 1.0f));

    // 0.5 Hz should give period_ms = 2000
    SendableSensorData ssd(&sdh, 0.5f);
    TEST_ASSERT_EQUAL_UINT16(2000, ssd.period_ms);

    // Should not be sent again until 2000ms have elapsed
    ssd.markWasSent(0);
    TEST_ASSERT_FALSE(ssd.shouldBeSent(1000));
    TEST_ASSERT_FALSE(ssd.shouldBeSent(1999));
    TEST_ASSERT_TRUE(ssd.shouldBeSent(2000));
}

// =====================================================================
// Low power mode tests
// =====================================================================

void test_low_power_mode_initially_inactive(void) {
    MockDataSaver mock;
    SensorDataHandler sdh(1, &mock);
    sdh.addData(DataPoint(1, 1.0f));

    SendableSensorData ssd(&sdh, 10.0f);
    std::array<SendableSensorData*, 1> ssds{&ssd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    TEST_ASSERT_FALSE(telemetry.isLowPowerModeActive());
    TEST_ASSERT_EQUAL_UINT8(0, telemetry.getLowPowerDivisor());
}

void test_low_power_mode_activate_scales_periods(void) {
    MockDataSaver mock1, mock2;
    SensorDataHandler sdh1(1, &mock1);
    SensorDataHandler sdh2(2, &mock2);
    sdh1.addData(DataPoint(1, 1.0f));
    sdh2.addData(DataPoint(1, 2.0f));

    // 10 Hz → 100ms, 2 Hz → 500ms
    SendableSensorData ssd1(&sdh1, 10.0f);
    SendableSensorData ssd2(&sdh2, 2.0f);
    std::array<SendableSensorData*, 2> ssds{&ssd1, &ssd2};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    TEST_ASSERT_EQUAL_UINT16(100, ssd1.period_ms);
    TEST_ASSERT_EQUAL_UINT16(500, ssd2.period_ms);

    telemetry.activateLowPowerMode(4);

    TEST_ASSERT_TRUE(telemetry.isLowPowerModeActive());
    TEST_ASSERT_EQUAL_UINT8(4, telemetry.getLowPowerDivisor());
    // 100 * 4 = 400ms, 500 * 4 = 2000ms
    TEST_ASSERT_EQUAL_UINT16(400, ssd1.period_ms);
    TEST_ASSERT_EQUAL_UINT16(2000, ssd2.period_ms);
}

void test_low_power_mode_deactivate_restores_periods(void) {
    MockDataSaver mock1, mock2;
    SensorDataHandler sdh1(1, &mock1);
    SensorDataHandler sdh2(2, &mock2);
    sdh1.addData(DataPoint(1, 1.0f));
    sdh2.addData(DataPoint(1, 2.0f));

    SendableSensorData ssd1(&sdh1, 10.0f);   // 100ms
    SendableSensorData ssd2(&sdh2, 2.0f);    // 500ms
    std::array<SendableSensorData*, 2> ssds{&ssd1, &ssd2};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    telemetry.activateLowPowerMode(5);
    TEST_ASSERT_EQUAL_UINT16(500, ssd1.period_ms);
    TEST_ASSERT_EQUAL_UINT16(2500, ssd2.period_ms);

    telemetry.deactivateLowPowerMode();
    TEST_ASSERT_FALSE(telemetry.isLowPowerModeActive());
    TEST_ASSERT_EQUAL_UINT16(100, ssd1.period_ms);
    TEST_ASSERT_EQUAL_UINT16(500, ssd2.period_ms);
}

void test_low_power_mode_change_divisor_does_not_compound(void) {
    MockDataSaver mock;
    SensorDataHandler sdh(1, &mock);
    sdh.addData(DataPoint(1, 1.0f));

    SendableSensorData ssd(&sdh, 10.0f);  // 100ms
    std::array<SendableSensorData*, 1> ssds{&ssd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    // Activate with divisor 4: 100 * 4 = 400
    telemetry.activateLowPowerMode(4);
    TEST_ASSERT_EQUAL_UINT16(400, ssd.period_ms);

    // Change to divisor 10: should be 100 * 10 = 1000, NOT 400 * 10 = 4000
    telemetry.activateLowPowerMode(10);
    TEST_ASSERT_EQUAL_UINT16(1000, ssd.period_ms);
}

void test_low_power_mode_deactivate_when_inactive_is_noop(void) {
    MockDataSaver mock;
    SensorDataHandler sdh(1, &mock);
    sdh.addData(DataPoint(1, 1.0f));

    SendableSensorData ssd(&sdh, 10.0f);  // 100ms
    std::array<SendableSensorData*, 1> ssds{&ssd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    // Should not crash or change anything
    telemetry.deactivateLowPowerMode();
    TEST_ASSERT_FALSE(telemetry.isLowPowerModeActive());
    TEST_ASSERT_EQUAL_UINT16(100, ssd.period_ms);
}

void test_low_power_mode_clamps_divisor_below_2(void) {
    MockDataSaver mock;
    SensorDataHandler sdh(1, &mock);
    sdh.addData(DataPoint(1, 1.0f));

    SendableSensorData ssd(&sdh, 10.0f);  // 100ms
    std::array<SendableSensorData*, 1> ssds{&ssd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    // Divisor of 1 should be clamped to 2
    telemetry.activateLowPowerMode(1);
    TEST_ASSERT_EQUAL_UINT16(200, ssd.period_ms);
}

void test_low_power_mode_clamps_period_to_uint16_max(void) {
    MockDataSaver mock;
    SensorDataHandler sdh(1, &mock);
    sdh.addData(DataPoint(1, 1.0f));

    // 0.1 Hz → 10000ms period
    SendableSensorData ssd(&sdh, 0.1f);
    std::array<SendableSensorData*, 1> ssds{&ssd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    TEST_ASSERT_EQUAL_UINT16(10000, ssd.period_ms);

    // Divisor 10: 10000 * 10 = 100000 → clamped to 65535
    telemetry.activateLowPowerMode(10);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, ssd.period_ms);

    // Restore should give back the original 10000ms
    telemetry.deactivateLowPowerMode();
    TEST_ASSERT_EQUAL_UINT16(10000, ssd.period_ms);
}

void test_low_power_mode_affects_tick_behavior(void) {
    MockDataSaver mock;
    SensorDataHandler sdh(1, &mock);
    sdh.addData(DataPoint(1, 42.0f));

    // 2 Hz → 500ms period
    SendableSensorData ssd(&sdh, 2.0f);
    std::array<SendableSensorData*, 1> ssds{&ssd};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    // Normal mode: should send at t=500
    TEST_ASSERT_TRUE(telemetry.tick(500));
    mockRfdSerial.clearWriteCalls();

    // Activate divisor 4: period becomes 500 * 4 = 2000ms
    telemetry.activateLowPowerMode(4);

    // Should NOT send at t=1000 (only 500ms since last send, need 2000ms now)
    TEST_ASSERT_FALSE(telemetry.tick(1000));
    TEST_ASSERT_FALSE(telemetry.tick(1500));
    TEST_ASSERT_FALSE(telemetry.tick(2000));

    // Should send at t=2500 (2000ms since last send at t=500)
    TEST_ASSERT_TRUE(telemetry.tick(2500));
    mockRfdSerial.clearWriteCalls();

    // Deactivate: period restored to 500ms
    telemetry.deactivateLowPowerMode();

    // Should send at t=3000 (500ms since last send at t=2500)
    TEST_ASSERT_TRUE(telemetry.tick(3000));
}

void test_low_power_mode_preserves_relative_priorities(void) {
    MockDataSaver mock1, mock2;
    SensorDataHandler sdh1(1, &mock1);
    SensorDataHandler sdh2(2, &mock2);
    sdh1.addData(DataPoint(1, 1.0f));
    sdh2.addData(DataPoint(1, 2.0f));

    // 10 Hz (100ms) and 2 Hz (500ms) — ratio is 5:1
    SendableSensorData ssd1(&sdh1, 10.0f);
    SendableSensorData ssd2(&sdh2, 2.0f);
    std::array<SendableSensorData*, 2> ssds{&ssd1, &ssd2};
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);

    telemetry.activateLowPowerMode(4);

    // Ratio should still be 5:1 → 400ms and 2000ms
    TEST_ASSERT_EQUAL_UINT16(400, ssd1.period_ms);
    TEST_ASSERT_EQUAL_UINT16(2000, ssd2.period_ms);
    TEST_ASSERT_EQUAL(5, ssd2.period_ms / ssd1.period_ms);
}

int main(void) {
    UNITY_BEGIN();

    // Original tests
    RUN_TEST(test_initialization);
    RUN_TEST(test_a_full_second_of_ticks);
    RUN_TEST(test_first_packet_counter_is_zero);
    RUN_TEST(test_second_packet_counter_is_one);

    // Float frequency conversion tests
    RUN_TEST(test_hz_to_period_integer_values);
    RUN_TEST(test_hz_to_period_fractional_values);
    RUN_TEST(test_hz_to_period_zero_returns_fallback);
    RUN_TEST(test_hz_to_period_negative_returns_fallback);
    RUN_TEST(test_hz_to_period_very_slow_rate_clamps_to_max);
    RUN_TEST(test_hz_to_period_ceil_semantics);
    RUN_TEST(test_sendable_sensor_data_fractional_hz_period);

    // Low power mode tests
    RUN_TEST(test_low_power_mode_initially_inactive);
    RUN_TEST(test_low_power_mode_activate_scales_periods);
    RUN_TEST(test_low_power_mode_deactivate_restores_periods);
    RUN_TEST(test_low_power_mode_change_divisor_does_not_compound);
    RUN_TEST(test_low_power_mode_deactivate_when_inactive_is_noop);
    RUN_TEST(test_low_power_mode_clamps_divisor_below_2);
    RUN_TEST(test_low_power_mode_clamps_period_to_uint16_max);
    RUN_TEST(test_low_power_mode_affects_tick_behavior);
    RUN_TEST(test_low_power_mode_preserves_relative_priorities);

    return UNITY_END();
}