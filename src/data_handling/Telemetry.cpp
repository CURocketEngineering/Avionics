#include "data_handling/Telemetry.h"
#include "ArduinoHAL.h"
#include <algorithm>
#include <cstdint>

// Helpers for checking if the packet has room for more data.
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

void Telemetry::checkForRadioCommandSequence(std::uint32_t currentTime_ms) {
    if (inCommandMode_) {
        return;
    }

    while (rfdSerialConnection_.available() > 0) {
        const char receivedChar = static_cast<char>(rfdSerialConnection_.read());

        if (receivedChar == TelemetryFmt::kCommandEntryChar) {
            ++commandEntryProgress_;
            if (commandEntryProgress_ >= TelemetryFmt::kCommandEntrySequenceLength) {
                enterCommandMode(currentTime_ms);
            }
        } else {
            // Send a debug message to the stream
            rfdSerialConnection_.print("Received char '");
            rfdSerialConnection_.print(receivedChar);
            rfdSerialConnection_.print("' which does not match command entry char '");
            rfdSerialConnection_.print(TelemetryFmt::kCommandEntryChar);
            rfdSerialConnection_.println("'. Resetting command entry progress.");
            commandEntryProgress_ = 0;
        }
    }
}

void Telemetry::enterCommandMode(std::uint32_t currentTime_ms) {
    inCommandMode_ = true;
    commandModeEnteredTimestamp_ms_ = currentTime_ms;
    commandModeLastInputTimestamp_ms_ = currentTime_ms;
    commandEntryProgress_ = 0;
    commandModeTimeoutLocked_ = false;
    commandModeTimeoutLockDeadline_ms_ = 0;

    if (commandLine_ != nullptr) {
        commandLine_->switchUART(&rfdSerialConnection_);
        commandLine_->print(kShellPrompt);
    }
}

void Telemetry::exitCommandMode() {
    inCommandMode_ = false;
    commandModeTimeoutLocked_ = false;
    commandModeTimeoutLockDeadline_ms_ = 0;

    if (commandLine_ != nullptr) {
        commandLine_->useDefaultUART();
    }
}

void Telemetry::lockCommandModeTimeout(std::uint32_t lockDuration_ms) {
    if (!inCommandMode_ || lockDuration_ms == 0) {
        return;
    }

    const std::uint32_t now_ms = millis();
    commandModeTimeoutLocked_ = true;
    commandModeTimeoutLockDeadline_ms_ = now_ms + lockDuration_ms;
    commandModeLastInputTimestamp_ms_ = now_ms;
}

void Telemetry::unlockCommandModeTimeout() {
    commandModeTimeoutLocked_ = false;
    commandModeTimeoutLockDeadline_ms_ = 0;

    if (inCommandMode_) {
        commandModeLastInputTimestamp_ms_ = millis();
    }
}

void Telemetry::forceExitCommandMode() {
    if (!inCommandMode_) {
        return;
    }

    exitCommandMode();
}

bool Telemetry::shouldPauseTelemetryForCommandMode(std::uint32_t currentTime_ms) {
    if (!inCommandMode_) {
        return false;
    }

    if (commandLine_ != nullptr) {
        const std::uint32_t lastInteractionTimestamp = commandLine_->getLastInteractionTimestamp();
        if (isTimestampNewer(lastInteractionTimestamp, commandModeLastInputTimestamp_ms_)) {
            commandModeLastInputTimestamp_ms_ = lastInteractionTimestamp;
        }
    }

    // If currentTime_ms was sampled before command input was processed in this loop,
    // avoid unsigned underflow in the inactivity subtraction.
    if (isTimestampNewer(commandModeLastInputTimestamp_ms_, currentTime_ms)) {
        return true;
    }

    if (commandModeTimeoutLocked_) {
        if (!isTimestampReachedOrPassed(currentTime_ms, commandModeTimeoutLockDeadline_ms_)) {
            return true;
        }

        commandModeTimeoutLocked_ = false;
        commandModeTimeoutLockDeadline_ms_ = 0;
        commandModeLastInputTimestamp_ms_ = currentTime_ms;
        return true;
    }

    if ((currentTime_ms - commandModeLastInputTimestamp_ms_) >= TelemetryFmt::kCommandModeInactivityTimeout_ms) {
        exitCommandMode();
        return false;
    }

    return true;
}

void Telemetry::preparePacket(std::uint32_t timestamp_ms) {
    // Write the packet header with sync bytes, start byte, and timestamp.
    // Only clear what we own in the header (full-packet clearing happens in setPacketToZero()).

    // Fill sync bytes with 0
    std::fill_n(&this->packet_[0], TelemetryFmt::kSyncZeroCount_bytes, static_cast<std::uint8_t>(0));

    // Set the start byte after the sync bytes
    this->packet_[TelemetryFmt::kStartByteIndex] = TelemetryFmt::kStartByteValue;

    // Write the timestamp in big-endian format
    TelemetryFmt::writeU32Be(&this->packet_[TelemetryFmt::kTimestampIndex], timestamp_ms);

    // Packet counter (4 bytes, big-endian)
    TelemetryFmt::writeU32Be(&this->packet_[TelemetryFmt::kPacketCounterIndex], packetCounter_);

    nextEmptyPacketIndex_ = TelemetryFmt::kHeaderSize_bytes;
}

void Telemetry::addSingleSDHToPacket(SensorDataHandler* sdh) {
    float floatData = sdh->getLastDataPointSaved().data; 
    uint32_t data = 0;
    memcpy(&data, &floatData, sizeof(data)); // Move float data into an uint32_t for bytewise access
    TelemetryFmt::writeU32Be(&this->packet_[nextEmptyPacketIndex_], data);
    nextEmptyPacketIndex_ += TelemetryFmt::kBytesInU32_bytes;
}

void Telemetry::addSSDToPacket(SendableSensorData* ssd) {
    if (ssd->isSingle()) {
        this->packet_[nextEmptyPacketIndex_] = ssd->singleSDH->getName();
        nextEmptyPacketIndex_ += 1;
        this->addSingleSDHToPacket(ssd->singleSDH);
    }
    if (ssd->isMulti()) {
        this->packet_[nextEmptyPacketIndex_] = ssd->multiSDHDataLabel;
        nextEmptyPacketIndex_ += 1;
        for (size_t i = 0; i < ssd->multiSDHLength; i++) { 
            // multiSDHLength comes directly from the array passed in by the client
            // So, we can ignore this raw pointer indexing warning
            this->addSingleSDHToPacket(ssd->multiSDH[i]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
    }
}

void Telemetry::setPacketToZero() {
    for (size_t i = 0; i < TelemetryFmt::kPacketCapacity_bytes; i++) { // Completely clear packet
        this->packet_[i] = 0;
    }
}

void Telemetry::addEndMarker() {
    // Add the following 4 bytes to the end of the packet: 0x00 0x00 0x00 (kEndByteValue).

    std::fill_n(&this->packet_[nextEmptyPacketIndex_], TelemetryFmt::kSyncZeroCount_bytes, static_cast<std::uint8_t>(0));
    this->packet_[nextEmptyPacketIndex_+TelemetryFmt::kSyncZeroCount_bytes] = TelemetryFmt::kEndByteValue;
    nextEmptyPacketIndex_ += TelemetryFmt::kEndMarkerSize_bytes;
}

bool Telemetry::canFitStreamWithEndMarker(const SendableSensorData* ssd) const {
    const std::size_t payloadSize_bytes = bytesNeededForSSD(ssd);
    return hasRoom(nextEmptyPacketIndex_, payloadSize_bytes + TelemetryFmt::kEndMarkerSize_bytes);
}

void Telemetry::tryAppendStream(SendableSensorData* stream, std::uint32_t currentTime_ms, bool& payloadAdded) {
    if (!stream->shouldBeSent(currentTime_ms)) {
        return;
    }

    if (!canFitStreamWithEndMarker(stream)) {
        return;
    }

    addSSDToPacket(stream);
    stream->markWasSent(currentTime_ms);
    payloadAdded = true;
}

bool Telemetry::finalizeAndSendPacket() {
    if (nextEmptyPacketIndex_ <= TelemetryFmt::kHeaderSize_bytes) {
        return false;
    }

    if (!hasRoom(nextEmptyPacketIndex_, TelemetryFmt::kEndMarkerSize_bytes)) {
        return false;
    }

    addEndMarker();
    for (std::size_t i = 0; i < nextEmptyPacketIndex_; i++) {
        rfdSerialConnection_.write(packet_[i]);
    }

    packetCounter_++;
    return true;
}

bool Telemetry::tick(uint32_t currentTime_ms) {
    // Checks if we should put the telemetry into command mode
    checkForRadioCommandSequence(currentTime_ms);

    if (shouldPauseTelemetryForCommandMode(currentTime_ms)) {
        return false;
    }

    setPacketToZero();
    preparePacket(currentTime_ms);

    bool payloadAdded = false;

    for (std::size_t i = 0; i < streamCount_; i++) {
        // i is safe because the stream count comes from the array passed in by the client.
        tryAppendStream(streams_[i], currentTime_ms, payloadAdded); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    if (!payloadAdded) {
        return false;
    }

    return finalizeAndSendPacket();
}
