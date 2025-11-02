#include "data_handling/Telemetry.h"
#include "ArduinoHAL.h"
#include <algorithm>

Telemetry::Telemetry(SendableSensorData* ssdArray[], int ssdArrayLength, Stream &rfdSerialConnection)
    : rfdSerialConnection(rfdSerialConnection)
{
    this->ssdArray = ssdArray;
    this->ssdArrayLength = ssdArrayLength;
    //TODO: it would be nice if we could throw an error at compile time
    //if a user's desired max packet size as specified from what they put in
    //ssdArray is larger than this value.
}

void Telemetry::preparePacket(uint32_t timestamp) {
    this->packet[0] = 0;
    this->packet[1] = 0;
    this->packet[2] = 0;
    this->packet[3] = START_BYTE;
    this->packet[4] = (timestamp >> 24) & 0xFF;
    this->packet[5] = (timestamp >> 16) & 0xFF;
    this->packet[6] = (timestamp >> 8) & 0xFF;
    this->packet[7] = timestamp & 0xFF;
    nextEmptyPacketIndex = 8;
}

void Telemetry::addSingleSDHToPacket(SensorDataHandler* sdh) {
    float floatData = sdh->getLastDataPointSaved().data; 
    uint32_t data;
    memcpy(&data, &floatData, sizeof(data));
    for (int i = 3; i > -1; i--) {
        this->packet[nextEmptyPacketIndex+(3-i)] = (data >> (i*8)) & 0xFF;
    }
    nextEmptyPacketIndex += 4;
}

void Telemetry::addSSDToPacket(SendableSensorData* ssd) {
    if (ssd->singleSDH != nullptr) {
        this->packet[nextEmptyPacketIndex] = ssd->singleSDH->getName();
        nextEmptyPacketIndex += 1;
        this->addSingleSDHToPacket(ssd->singleSDH);
    }
    if (ssd->multiSDH != nullptr) {
        this->packet[nextEmptyPacketIndex] = ssd->multiSDHDataLabel;
        nextEmptyPacketIndex += 1;
        for (int i = 0; i < ssd->multiSDHLength; i++) {
            this->addSingleSDHToPacket(ssd->multiSDH[i]);
        }
    }
}

void Telemetry::setPacketToZero() {
    for (int i = 0; i < 120; i++) { //Completely clear packet
        this->packet[i] = 0;
    }
}

void Telemetry::addEndMarker() {
    this->packet[nextEmptyPacketIndex] = 0;
    this->packet[nextEmptyPacketIndex+1] = 0;
    this->packet[nextEmptyPacketIndex+2] = 0;
    this->packet[nextEmptyPacketIndex+3] = END_BYTE;
}

bool Telemetry::tick(uint32_t currentTime) {
    bool sendingPacketThisTick = false;
    int currentPacketIndex = 0;
    for (int i = 0; i < this->ssdArrayLength; i++) {
        if (ssdArray[i]->shouldBeSent(currentTime)) {
            if (!sendingPacketThisTick) {
                setPacketToZero();
                preparePacket(currentTime);
                addSSDToPacket(ssdArray[i]);
                sendingPacketThisTick = true;
            } else {
                addSSDToPacket(ssdArray[i]);
            }
            ssdArray[i]->markWasSent(currentTime);
        }
    }
    if (sendingPacketThisTick) {
        addEndMarker();
        for (int i = 0; i < nextEmptyPacketIndex; i++) {
            this->rfdSerialConnection.write(this->packet[i]);
        }
    }
    return sendingPacketThisTick;
}