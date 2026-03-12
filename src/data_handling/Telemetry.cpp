#include "data_handling/Telemetry.h"
#include "ArduinoHAL.h"
#include <algorithm>
#include <cstdint>

// Helpers for checking if the packet has room for more data
std::size_t bytesNeededForSSD(const SendableSensorData* ssd) {
    // Each SSD writes 1 label byte (name/label) plus 4 bytes per float value.
    if (ssd->isSingle()) {
        return 1U + TelemetryFmt::kBytesIn32Bit;
    }
    if (ssd->isMulti()) {
        // label + N floats
        return 1U + (static_cast<std::size_t>(ssd->multiSDHLength) * TelemetryFmt::kBytesIn32Bit);
    }
    return 0U;
}

bool hasRoom(std::size_t nextIndex, std::size_t bytesToAdd) {
    return nextIndex + bytesToAdd <= TelemetryFmt::kPacketCapacity;
}

bool isTimestampNewer(std::uint32_t lhs, std::uint32_t rhs) {
    return static_cast<std::int32_t>(lhs - rhs) > 0;
}

void Telemetry::checkForRadioCommandSequence(std::uint32_t currentTimeMs) {
    if (inCommandMode) {
        return;
    }

    while (rfdSerialConnection.available() > 0) {
        const char receivedChar = static_cast<char>(rfdSerialConnection.read());

        if (receivedChar == TelemetryFmt::kCommandEntryChar) {
            ++commandEntryProgress;
            if (commandEntryProgress >= TelemetryFmt::kCommandEntrySequenceLength) {
                enterCommandMode(currentTimeMs);
            }
        } else {
            commandEntryProgress = 0;
        }
    }
}

void Telemetry::enterCommandMode(std::uint32_t currentTimeMs) {
    inCommandMode = true;
    commandModeEnteredTimestamp = currentTimeMs;
    commandModeLastInputTimestamp = currentTimeMs;
    commandEntryProgress = 0;

    if (commandLine != nullptr) {
        commandLine->switchUART(&rfdSerialConnection);
        commandLine->print(SHELL_PROMPT);
    }
}

void Telemetry::exitCommandMode() {
    inCommandMode = false;

    if (commandLine != nullptr) {
        commandLine->useDefaultUART();
    }
}

bool Telemetry::shouldPauseTelemetryForCommandMode(std::uint32_t currentTimeMs) {
    if (!inCommandMode) {
        return false;
    }

    if (commandLine != nullptr) {
        const std::uint32_t lastInteractionTimestamp = commandLine->getLastInteractionTimestamp();
        if (isTimestampNewer(lastInteractionTimestamp, commandModeLastInputTimestamp)) {
            commandModeLastInputTimestamp = lastInteractionTimestamp;
        }
    }

    if ((currentTimeMs - commandModeLastInputTimestamp) >= TelemetryFmt::kCommandModeInactivityTimeoutMs) {
        exitCommandMode();
        return false;
    }

    return true;
}

void Telemetry::preparePacket(std::uint32_t timestamp) {
    // This write the header of the packet with sync bytes, start byte, and timestamp.
    // Only clear what we own in the header (whole-packet clearing happens in setPacketToZero()).

    // Fill sync bytes with 0
    std::fill_n(&this->packet[0], TelemetryFmt::kSyncZeros, static_cast<std::uint8_t>(0));

    // Set the start byte after the sync bytes
    this->packet[TelemetryFmt::kStartByteIndex] = TelemetryFmt::kStartByteValue;

    // Write the timestamp in big-endian format
    TelemetryFmt::write_u32_be(&this->packet[TelemetryFmt::kTimestampIndex], timestamp);

    // Packet counter (4 bytes, big-endian)
    TelemetryFmt::write_u32_be(&this->packet[TelemetryFmt::kPacketCounterIndex], packetCounter);

    nextEmptyPacketIndex = TelemetryFmt::kHeaderBytes;
}

void Telemetry::addSingleSDHToPacket(SensorDataHandler* sdh) {
    float floatData = sdh->getLastDataPointSaved().data; 
    uint32_t data = 0;
    memcpy(&data, &floatData, sizeof(data)); // Move float data into an uint32_t for bytewise access
    TelemetryFmt::write_u32_be(&this->packet[nextEmptyPacketIndex], data);
    nextEmptyPacketIndex += TelemetryFmt::kBytesIn32Bit;
}

void Telemetry::addSSDToPacket(SendableSensorData* ssd) {
    if (ssd->isSingle()) {
        this->packet[nextEmptyPacketIndex] = ssd->singleSDH->getName();
        nextEmptyPacketIndex += 1;
        this->addSingleSDHToPacket(ssd->singleSDH);
    }
    if (ssd->isMulti()) {
        this->packet[nextEmptyPacketIndex] = ssd->multiSDHDataLabel;
        nextEmptyPacketIndex += 1;
        for (int i = 0; i < ssd->multiSDHLength; i++) { 
            // multiSDHLength comes directly from the array passed in by the client
            // So, we can ignore this raw pointer indexing warning
            this->addSingleSDHToPacket(ssd->multiSDH[i]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
    }
}

void Telemetry::setPacketToZero() {
    for (int i = 0; i < TelemetryFmt::kPacketCapacity; i++) { //Completely clear packet
        this->packet[i] = 0;
    }
}

void Telemetry::addEndMarker() {
    // Adds the following 4 bytes to the end of the packet: 0x00 0x00 0x00 (kEndByteValue)

    std::fill_n(&this->packet[nextEmptyPacketIndex], TelemetryFmt::kSyncZeros, static_cast<std::uint8_t>(0));
    this->packet[nextEmptyPacketIndex+TelemetryFmt::kSyncZeros] = TelemetryFmt::kEndByteValue;
    nextEmptyPacketIndex += TelemetryFmt::kEndMarkerBytes;
}

bool Telemetry::canFitStreamWithEndMarker(const SendableSensorData* ssd) const {
    const std::size_t payloadBytes = bytesNeededForSSD(ssd);
    return hasRoom(nextEmptyPacketIndex, payloadBytes + TelemetryFmt::kEndMarkerBytes);
}

void Telemetry::tryAppendStream(SendableSensorData* stream, std::uint32_t currentTimeMs, bool& packetStarted, bool& payloadAdded) {
    if (!stream->shouldBeSent(currentTimeMs)) {
        return;
    }

    if (!packetStarted) {
        setPacketToZero();
        preparePacket(currentTimeMs);
        packetStarted = true;
    }

    if (!canFitStreamWithEndMarker(stream)) {
        return;
    }

    addSSDToPacket(stream);
    stream->markWasSent(currentTimeMs);
    payloadAdded = true;
}

bool Telemetry::finalizeAndSendPacket() {
    if (nextEmptyPacketIndex <= TelemetryFmt::kHeaderBytes) {
        return false;
    }

    if (!hasRoom(nextEmptyPacketIndex, TelemetryFmt::kEndMarkerBytes)) {
        return false;
    }

    addEndMarker();
    for (std::size_t i = 0; i < nextEmptyPacketIndex; i++) {
        rfdSerialConnection.write(packet[i]);
    }

    packetCounter++;
    return true;
}

bool Telemetry::tick(uint32_t currentTime) {
    // Checks if we should put the telemetry into command mode
    checkForRadioCommandSequence(currentTime);

    if (shouldPauseTelemetryForCommandMode(currentTime)) {
        return false;
    }

    bool packetStarted = false;
    bool payloadAdded = false;

    for (std::size_t i = 0; i < streamCount; i++) {
        // i is safe because streamCount comes from the array passed in by the client
        tryAppendStream(streams[i], currentTime, packetStarted, payloadAdded); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    if (!payloadAdded) {
        return false;
    }

    return finalizeAndSendPacket();
}
