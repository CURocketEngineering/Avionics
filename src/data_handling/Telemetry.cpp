#include "data_handling/Telemetry.h"
#include "ArduinoHAL.h"
#include <algorithm>

Telemetry::Telemetry(SendableSensorData* ssdArray[], int ssdArrayLength, HardwareSerial &rfdSerialConnection)
    : rfdSerialConnection(rfdSerialConnection)
{
    this->ssdArray = ssdArray;
    this->ssdArrayLength = ssdArrayLength;
    this->lastSecond = 0;
    this->lastFrequencyHzSent = 100;
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

bool Telemetry::tick() {
    uint32_t timestamp = millis();
    uint32_t thisSecond = timestamp/1000;
    uint32_t millisOfThisSecond = (timestamp-((timestamp/1000)*1000));
    int thisTicksFrequency = ceil(1000.0/(millisOfThisSecond+1)); //+1 prevents the case (that happens a lot during testing in Native) in which millisOfThisSecond is 0
    bool sendingPacketThisTick = false;
    int currentPacketIndex = 0;
    for (int i = 0; i < this->ssdArrayLength; i++) {
        if (ssdArray[i]->sendFrequencyHz < this->lastFrequencyHzSent && 
            (ssdArray[i]->sendFrequencyHz >= thisTicksFrequency || (ssdArray[i]->sendFrequencyHz == 1 && thisSecond>lastSecond))) {
            this->lastFrequencyHzSent = ssdArray[i]->sendFrequencyHz;
            if (!sendingPacketThisTick) {
                preparePacket(timestamp);
                addSSDToPacket(ssdArray[i]);
                sendingPacketThisTick = true;
            } else {
                addSSDToPacket(ssdArray[i]);
            }
        }
    }
    if (sendingPacketThisTick) {
        if (this->lastFrequencyHzSent == 1) {
            this->lastFrequencyHzSent = 100;
        }
        for (int i = 0; i < nextEmptyPacketIndex; i++) {
            this->rfdSerialConnection.write(this->packet[i]);
        }
    }
    if (thisSecond > lastSecond) lastSecond = thisSecond;
    return sendingPacketThisTick;
}