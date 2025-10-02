#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "data_handling/DataPoint.h"
#include "data_handling/SensorDataHandler.h"
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <SoftwareSerial.h>

#define START_BYTE_BASE 50

/**
 * @struct SendableSensorData
 * @brief Wrapper struct to hold data info
 * @param singleSDH: SensorDataHandler*, put just one SDH pointer here
 * @param multiSDH: SensorDataHandler**, put an array of pointers to SDHs
 * @param sendFrequencyHz: desired frequency to send at
 * @param sensorDataBytes: number of bytes per SensorDatahandler
 */
struct SendableSensorData {
    SensorDataHandler* singleSDH;
    SensorDataHandler** multiSDH;
    int multiSDHDataLabel;
    int sendFrequencyHz;
    int sensorDataBytes;

    SendableSensorData(SensorDataHandler* _singleSDH, SensorDataHandler** _multiSDH, int _multiSDHDataLabel, int _sendFrequencyHz, int _sensorDataBytes) {
        singleSDH = _singleSDH;
        multiSDH = _multiSDH;
        multiSDHDataLabel = _multiSDHDataLabel;
        sendFrequencyHz = _sendFrequencyHz;
        sensorDataBytes = _sensorDataBytes;
    }
};

class Telemetry {
    public:
        /**
         * @brief Initialize this object
         * 
         * @param ssdArray array of pointers to SendableSensorData structs
         * @param txPin tx pin that the RFD is connected to
         * @param rxPin rx pin that the RFD is connected to
         */
        Telemetry(SendableSensorData* ssdArray[], int txPin, int rxPin);

        /**
         * @attention MUST BE RUN EVERY LOOP
         * No argument tick function that keeps track of sending data at
         * specified send frequencies.
         */
        void tick();

    private:
        void preparePacket(byte frequency, uint32_t timestamp);
        void addSingleSDHToPacket(SensorDataHandler* sdh, int sensorDataBytes);
        void addSSDToPacket(SendableSensorData* ssd);

        SendableSensorData** ssdArray;
        SoftwareSerial rfdSerialConnection;
        int lastFrequencyHzSent;
        int nextEmptyPacketIndex;
        byte packet[128]; //128 is the maximum packet length.
        // This can be modified if more space is needed,
        // but ideally it matches the max packet size of the telemetry device.
        // The RFD900x has a theoritcal a maximum packet length,
        // but it is unlikely that we will reach it.
};

#endif