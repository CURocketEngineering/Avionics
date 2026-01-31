#include "data_handling/Telemetry.h"
#include "ArduinoHAL.h"
#include <algorithm>

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

void Telemetry::preparePacket(std::uint32_t timestamp) {
    // This write the header of the packet with sync bytes, start byte, and timestamp.
    // Only clear what we own in the header (whole-packet clearing happens in setPacketToZero()).

    // Fill sync bytes with 0
    std::fill_n(&this->packet[0], TelemetryFmt::kSyncZeros, static_cast<std::uint8_t>(0));

    // Set the start byte after the sync bytes
    this->packet[TelemetryFmt::kStartByteIndex] = TelemetryFmt::kStartByteValue;

    // Write the timestamp in big-endian format
    TelemetryFmt::write_u32_be(&this->packet[TelemetryFmt::kTimestampIndex], timestamp);

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

bool Telemetry::tick(uint32_t currentTime) {
    bool sendingPacketThisTick = false;

    for (std::size_t i = 0; i < streamCount; i++) {
        // i is safe because streamCount comes from the array passed in by the client
        if (streams[i]->shouldBeSent(currentTime)) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

            if (!sendingPacketThisTick) {
                setPacketToZero();
                preparePacket(currentTime);
                sendingPacketThisTick = true;
            }

            // Compute how many bytes we need for this stream's payload.
            const std::size_t payloadBytes = bytesNeededForSSD(streams[i]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            const std::size_t totalBytesIfAdded = payloadBytes + TelemetryFmt::kEndMarkerBytes;

            // Only add if it fits (payload + end marker).
            if (hasRoom(nextEmptyPacketIndex, totalBytesIfAdded)) {
                addSSDToPacket(streams[i]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                streams[i]->markWasSent(currentTime); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            } else {
                // Not enough room. Skip this stream for now.
                // It will be sent on the next tick as long as the packet isn't filled before reaching it again.
                // If we have too many high-frequency streams, the stream as the end of the list may be starved.
            }
        }
    }

    // Only send if we actually added any payload beyond the header.
    if (sendingPacketThisTick && nextEmptyPacketIndex > TelemetryFmt::kHeaderBytes) {
        // Ensure end marker itself fits
        if (hasRoom(nextEmptyPacketIndex, TelemetryFmt::kEndMarkerBytes)) {
            addEndMarker();

            // Send used portion
            for (std::size_t i = 0; i < nextEmptyPacketIndex; i++) {
                rfdSerialConnection.write(packet[i]);
            }
            return true;
        }

        // If somehow we can't fit the end marker, drop the packet.
        // (This shouldn't happen with the checks above.)
    }

    return false;
}
