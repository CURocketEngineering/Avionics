#include "data_handling/Telemetry.h"
#include "ArduinoHAL.h"
#include <SoftwareSerial.h>
#include <algorithm>

Telemetry::Telemetry(SendableSensorData* ssdArray[], int txPin, int rxPin)
    : rfdSerialConnection(txPin, rxPin, false)
{
    this->ssdArray = ssdArray;
    //TODO: it would be nice if we could throw an error at compile time
    //if a user's desired max packet size as specified from what they put in
    //ssdArray is larger than this value.
}

void Telemetry::preparePacket(byte frequency, uint32_t timestamp) {
    this->packet[0] = START_BYTE_BASE+frequency;
    this->packet[1] = (timestamp >> 24) & 0xFF;
    this->packet[2] = (timestamp >> 16) & 0xFF;
    this->packet[3] = (timestamp >> 8) & 0xFF;
    this->packet[4] = timestamp & 0xFF;
    nextEmptyPacketIndex = 5;
}

//TODO: add support for datalabels (ie. remove line 26 from this method and put that before 1 single data handler or before 1 multi data handler)
void Telemetry::addSingleSDHToPacket(SensorDataHandler* sdh, int sensorDataBytes) {
    uint32_t data = sdh->getLastDataPointSaved().data;
    for (int i = 0; i < sensorDataBytes; i++) {
        this->packet[nextEmptyPacketIndex+i] = (data >> (i*8)) & 0xFF;
    }
    nextEmptyPacketIndex += (sensorDataBytes+1);
}

void Telemetry::addSSDToPacket(SendableSensorData* ssd) {
    if (ssd->singleSDH > 0) {
        this->packet[nextEmptyPacketIndex] = ssd->singleSDH->getName();
        nextEmptyPacketIndex += 1;
        this->addSingleSDHToPacket(ssd->singleSDH, ssd->sensorDataBytes);
    }
    if (ssd->multiSDH > 0) {
        this->packet[nextEmptyPacketIndex] = ssd->multiSDHDataLabel;
        nextEmptyPacketIndex += 1;
        for (int i = 0; i < size_t(ssd->multiSDH)/size_t(ssd->multiSDH[0]); i++) {
            this->addSingleSDHToPacket(ssd->multiSDH[i], ssd->sensorDataBytes);
        }
    }
}

void Telemetry::tick() {
    uint32_t timestamp = millis();
    uint32_t millisOfThisSecond = (timestamp-((timestamp/1000)*1000));
    int thisTicksFrequency = 1000/(millisOfThisSecond+10); //plus 10 so that at ~990, the last loop of the second, we are less than one here
    int ssdArrayLen = sizeof(ssdArray)/sizeof(ssdArray[0]);
    bool sendingPacketThisTick = false;
    int currentPacketIndex = 0;
    for (int i = 0; i < ssdArrayLen; i++) {
        if (ssdArray[i]->sendFrequencyHz > this->lastFrequencyHzSent && ssdArray[i]->sendFrequencyHz >= thisTicksFrequency) {
            this->lastFrequencyHzSent = ssdArray[i]->sendFrequencyHz;
            if (!sendingPacketThisTick) {
                preparePacket(ssdArray[i]->sendFrequencyHz, timestamp);
                addSSDToPacket(ssdArray[i]);
                sendingPacketThisTick = true;
            } else {
                addSSDToPacket(ssdArray[i]);
            }
        }
    }
    if (sendingPacketThisTick) {
        if (this->lastFrequencyHzSent == 1) {
            this->lastFrequencyHzSent = 0;
        }
        for (int i = 0; i < nextEmptyPacketIndex; i++) {
            this->rfdSerialConnection.write(this->packet[i]);
        }
    }
}