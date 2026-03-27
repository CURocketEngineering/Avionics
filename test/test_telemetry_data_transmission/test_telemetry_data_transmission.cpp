#include "unity.h"
#include <array>
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

    std::array<SendableSensorData*, 2> ssds{
        new SendableSensorData(accelerationTriplet, 102, 2),
        new SendableSensorData(&altitudeData, 1),
    };
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

    std::array<SendableSensorData*, 2> ssds{
        new SendableSensorData(accelerationTriplet, 102, 2),
        new SendableSensorData(&altitudeData, 1),
    };
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
    for (int i = 0; i < 51; i++) {
        char message[50];
        sprintf(message, "Byte %d mismatch", i);
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
    for (int i = 0; i < 51; i++) {
        // Skip timestamp bytes
        if (i >= 4 && i <= 11) {
            continue;
        }
        if (i >= 33 && i <= 40) {
            continue;
        }


        char message[50];
        sprintf(message, "Byte %d mismatch", i);
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
    std::array<SendableSensorData*, 1> ssds{
        new SendableSensorData(accelerationTriplet, 102, 2),
    };

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
    std::array<SendableSensorData*, 1> ssds{
        new SendableSensorData(accelerationTriplet, 102, 2),
    };

    Stream mockRfdSerial;
    Telemetry telemetry(ssds, mockRfdSerial);
    telemetry.tick((uint32_t)500);
    telemetry.tick((uint32_t)1000);

    // First packet: kHeaderSize_bytes(12) + label(1) + 3 floats(12) + end marker(4) = 29 bytes
    // Second packet counter starts at byte 29 + kPacketCounterIndex
    const int secondPacketStart = 29;
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex));
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex + 1));
    TEST_ASSERT_EQUAL(0, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex + 2));
    TEST_ASSERT_EQUAL(1, mockRfdSerial.writeCalls.at(secondPacketStart + TelemetryFmt::kPacketCounterIndex + 3));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_a_full_second_of_ticks);
    RUN_TEST(test_first_packet_counter_is_zero);
    RUN_TEST(test_second_packet_counter_is_one);
    return UNITY_END();
}
