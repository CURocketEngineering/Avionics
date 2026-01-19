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
 * @brief Bundles one or more SensorDataHandler pointers for telemetry packing.
 * @param singleSDH        Single handler to send when not using an array.
 * @param multiSDH         Array of handlers to send together (optional).
 * @param multiSDHLength   Length of the multiSDH array.
 * @param multiSDHDataLabel Label used for the multi array payload.
 * @param sendFrequencyHz  Desired transmission rate in hertz.
 * @note When to use: define the telemetry mix (single vs. grouped streams) fed
 *       to Telemetry before calling tick().
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

/**
 * @brief Packages sensor data into fixed-size packets and streams over UART.
 * @note When to use: periodic downlink of key channels during flight; call
 *       tick() every loop to honor per-stream send rates.
 */
class Telemetry {
    public:
    
        /**
         * @brief Initialize this object
         * @param ssdArray          Array of pointers to SendableSensorData.
         * @param ssdArrayLength    Number of entries in ssdArray.
         * @param rfdSerialConnection Stream connected to the radio/modem.
         * @note When to use: instantiate once during setup with the telemetry
         *       layout you need for the current flight profile.
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