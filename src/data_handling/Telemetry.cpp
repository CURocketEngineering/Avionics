#include "data_handling/Telemetry.h"
#include "ArduinoHAL.h"
#include <algorithm>
#include <cstdint>

// Helpers for checking if the packet has room for more data
std::size_t bytesNeededForSSD(const SendableSensorData* ssd) {
    // Each SSD writes 1 label byte (name/label) plus 4 bytes per float value.
    if (ssd->isSingle()) {
        return 1U + TelemetryFmt::kBytesInU32_bytes;
    }
    if (ssd->isMulti()) {
        // label + N floats
        return 1U + (static_cast<std::size_t>(ssd->multiSDHLength) * TelemetryFmt::kBytesInU32_bytes);
    }
    return 0U;
}

bool hasRoom(std::size_t nextIndex, std::size_t bytesToAdd) {
    return nextIndex + bytesToAdd <= TelemetryFmt::kPacketCapacity_bytes;
}

bool isTimestampNewer(std::uint32_t lhs, std::uint32_t rhs) {
    return static_cast<std::int32_t>(lhs - rhs) > 0;
}

bool isTimestampReachedOrPassed(std::uint32_t current, std::uint32_t target) {
    return static_cast<std::int32_t>(current - target) >= 0;
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
            // Send a debug message to the stream
            rfdSerialConnection.print("Received char '");
            rfdSerialConnection.print(receivedChar);
            rfdSerialConnection.print("' which does not match command entry char '");
            rfdSerialConnection.print(TelemetryFmt::kCommandEntryChar);
            rfdSerialConnection.println("'. Resetting command entry progress.");
            commandEntryProgress = 0;
        }
    }
}

void Telemetry::enterCommandMode(std::uint32_t currentTimeMs) {
    inCommandMode = true;
    commandModeEnteredTimestamp = currentTimeMs;
    commandModeLastInputTimestamp = currentTimeMs;
    commandEntryProgress = 0;
    commandModeTimeoutLocked = false;
    commandModeTimeoutLockDeadlineMs = 0;

    if (commandLine != nullptr) {
        commandLine->switchUART(&rfdSerialConnection);
        commandLine->print(kShellPrompt);
    }
}

void Telemetry::exitCommandMode() {
    inCommandMode = false;
    commandModeTimeoutLocked = false;
    commandModeTimeoutLockDeadlineMs = 0;

    if (commandLine != nullptr) {
        commandLine->useDefaultUART();
    }
}

void Telemetry::lockCommandModeTimeout(std::uint32_t lockDurationMs) {
    if (!inCommandMode || lockDurationMs == 0) {
        return;
    }

    const std::uint32_t nowMs = millis();
    commandModeTimeoutLocked = true;
    commandModeTimeoutLockDeadlineMs = nowMs + lockDurationMs;
    commandModeLastInputTimestamp = nowMs;
}

void Telemetry::unlockCommandModeTimeout() {
    commandModeTimeoutLocked = false;
    commandModeTimeoutLockDeadlineMs = 0;

    if (inCommandMode) {
        commandModeLastInputTimestamp = millis();
    }
}

void Telemetry::forceExitCommandMode() {
    if (!inCommandMode) {
        return;
    }

    exitCommandMode();
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

    // If currentTimeMs was sampled before command input was processed in this loop,
    // avoid unsigned underflow in the inactivity subtraction.
    if (isTimestampNewer(commandModeLastInputTimestamp, currentTimeMs)) {
        return true;
    }

    if (commandModeTimeoutLocked) {
        if (!isTimestampReachedOrPassed(currentTimeMs, commandModeTimeoutLockDeadlineMs)) {
            return true;
        }

        commandModeTimeoutLocked = false;
        commandModeTimeoutLockDeadlineMs = 0;
        commandModeLastInputTimestamp = currentTimeMs;
        return true;
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
    std::fill_n(&this->packet[0], TelemetryFmt::kSyncZeroCount_bytes, static_cast<std::uint8_t>(0));

    // Set the start byte after the sync bytes
    this->packet[TelemetryFmt::kStartByteIndex] = TelemetryFmt::kStartByteValue;

    // Write the timestamp in big-endian format
    TelemetryFmt::writeU32Be(&this->packet[TelemetryFmt::kTimestampIndex], timestamp);

    // Packet counter (4 bytes, big-endian)
    TelemetryFmt::writeU32Be(&this->packet[TelemetryFmt::kPacketCounterIndex], packetCounter);

    nextEmptyPacketIndex = TelemetryFmt::kHeaderSize_bytes;
}

void Telemetry::addSingleSDHToPacket(SensorDataHandler* sdh) {
    float floatData = sdh->getLastDataPointSaved().data; 
    uint32_t data = 0;
    memcpy(&data, &floatData, sizeof(data)); // Move float data into an uint32_t for bytewise access
    TelemetryFmt::writeU32Be(&this->packet[nextEmptyPacketIndex], data);
    nextEmptyPacketIndex += TelemetryFmt::kBytesInU32_bytes;
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
    for (int i = 0; i < TelemetryFmt::kPacketCapacity_bytes; i++) { //Completely clear packet
        this->packet[i] = 0;
    }
}

void Telemetry::addEndMarker() {
    // Adds the following 4 bytes to the end of the packet: 0x00 0x00 0x00 (kEndByteValue)

    std::fill_n(&this->packet[nextEmptyPacketIndex], TelemetryFmt::kSyncZeroCount_bytes, static_cast<std::uint8_t>(0));
    this->packet[nextEmptyPacketIndex+TelemetryFmt::kSyncZeroCount_bytes] = TelemetryFmt::kEndByteValue;
    nextEmptyPacketIndex += TelemetryFmt::kEndMarkerSize_bytes;
}

bool Telemetry::canFitStreamWithEndMarker(const SendableSensorData* ssd) const {
    const std::size_t payloadSize_bytes = bytesNeededForSSD(ssd);
    return hasRoom(nextEmptyPacketIndex, payloadSize_bytes + TelemetryFmt::kEndMarkerSize_bytes);
}

void Telemetry::tryAppendStream(SendableSensorData* stream, std::uint32_t currentTimeMs, bool& payloadAdded) {
    if (!stream->shouldBeSent(currentTimeMs)) {
        return;
    }

    if (!canFitStreamWithEndMarker(stream)) {
        return;
    }

    addSSDToPacket(stream);
    stream->markWasSent(currentTimeMs);
    payloadAdded = true;
}

bool Telemetry::finalizeAndSendPacket() {
    if (nextEmptyPacketIndex <= TelemetryFmt::kHeaderSize_bytes) {
        return false;
    }

    if (!hasRoom(nextEmptyPacketIndex, TelemetryFmt::kEndMarkerSize_bytes)) {
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

    setPacketToZero();
    preparePacket(currentTime);

    bool payloadAdded = false;

    for (std::size_t i = 0; i < streamCount; i++) {
        // i is safe because streamCount comes from the array passed in by the client
        tryAppendStream(streams[i], currentTime, payloadAdded); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    if (!payloadAdded) {
        return false;
    }

    return finalizeAndSendPacket();
}
