#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "data_handling/DataPoint.h"
#include "data_handling/SensorDataHandler.h"
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "ArduinoHAL.h"

#define START_BYTE 51
#define END_BYTE 52

/**
 * @struct SendableSensorData
 * @brief Wrapper struct to hold data info
 * @param singleSDH: SensorDataHandler*, put just one SDH pointer here
 * @param multiSDH: SensorDataHandler**, put an array of pointers to SDHs
 * @param multiSDHLength: length of multiSDHArray; can be 0 if singleSDH 
 * @param multiSDHDataLabel: data label of multiSDHArray; can be 0 if singleSDH 
 * @param sendFrequencyHz: desired frequency to send at
 * Number of bytes per sensor data handler is always assumed to be 4 (ie. a float) for consistency
 */
struct SendableSensorData {
    SensorDataHandler* singleSDH;
    SensorDataHandler** multiSDH;
    int multiSDHLength;
    int multiSDHDataLabel;
    int sendFrequencyHz;
    uint32_t lastSentTimestamp;

    SendableSensorData(SensorDataHandler* _singleSDH, SensorDataHandler** _multiSDH, int _multiSDHLength, int _multiSDHDataLabel, uint8_t _sendFrequencyHz) {
        singleSDH = _singleSDH;
        multiSDH = _multiSDH;
        multiSDHLength = _multiSDHLength;
        multiSDHDataLabel = _multiSDHDataLabel;
        sendFrequencyHz = _sendFrequencyHz;
        lastSentTimestamp = 0;
    }
    
    /**
     * @brief True if the packet should be sent
     */
    bool shouldBeSent(uint32_t time) {
        uint32_t delta = time-lastSentTimestamp;
        return delta >= (1000.0/sendFrequencyHz);
    }

    /**
     * @brief Run when the packet is sent
     */
    void markWasSent(uint32_t time) {
        lastSentTimestamp = time;
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
        Telemetry(SendableSensorData* ssdArray[], int ssdArrayLength, Stream &rfdSerialConnection);

        /**
         * @attention MUST BE RUN EVERY LOOP
         * @brief No argument tick function that handles sending data at
         * specified send frequencies.
         * @return true if a packet was sent after calling this function.
         */
        bool tick(uint32_t currentTime);

    private:
        void preparePacket(uint32_t timestamp);
        void addSingleSDHToPacket(SensorDataHandler* sdh);
        void addSSDToPacket(SendableSensorData* ssd);
        void setPacketToZero();
        void addEndMarker();

        SendableSensorData** ssdArray;
        int ssdArrayLength;
        Stream &rfdSerialConnection;
        int nextEmptyPacketIndex;
        uint8_t packet[120]; //rfd settings indicate that 120 is the max packet size
};

#endif