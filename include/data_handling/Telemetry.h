#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "ArduinoHAL.h"
#include "data_handling/SensorDataHandler.h"

/**
 * @file Telemetry.h
 * @brief Packs SensorDataHandler values into a fixed-size byte packet and streams over a Stream (UART).
 *
 * Design goals:
 * - Keep Telemetry as a normal class (non-templated) so implementation lives in Telemetry.cpp.
 * - Allow callers to provide the list of telemetry streams as either:
 *   - pointer + count (most general)
 *   - std::array (compile-time sized, convenient)
 *   - C array (compile-time sized, convenient)
 *
 * Lifetime rule:
 * - Telemetry stores a non-owning pointer to the caller-provided stream list.
 *   The stream list must outlive the Telemetry instance (use static storage in embedded code).
 */

namespace TelemetryFmt {

/** Maximum packet size (bytes). Must match your radio/modem configuration. */
constexpr std::size_t kPacketCapacity = 120;

/** Header markers: 3 sync zeros followed by a start byte. */
constexpr std::size_t kSyncZeros = 3;

/** Number of bytes in a packed 32-bit value. */
constexpr std::size_t kU32Bytes = 4;

/** Header layout: [0..2]=0, [3]=START, [4..7]=timestamp (big-endian). */
constexpr std::size_t kStartByteIndex = kSyncZeros;          // 3
constexpr std::size_t kTimestampIndex = kStartByteIndex + 1; // 4
constexpr std::size_t kHeaderBytes    = kSyncZeros + 1 + kU32Bytes;

/** End marker layout: 3 zeros followed by an end byte. */
constexpr std::size_t kEndMarkerBytes = kSyncZeros + 1;

/** Start-of-packet marker byte value. */
constexpr std::uint8_t kStartByteValue = 51;

/** End-of-packet marker byte value. */
constexpr std::uint8_t kEndByteValue = 52;

/** 32-bit helper constants */
constexpr std::size_t kBytesIn32Bit = 4;
constexpr unsigned kBitsPerByte = 8;
constexpr std::uint8_t kAllOnesByte = 0xFF;

/** Assumptions used by float packing. */
static_assert(sizeof(std::uint32_t) == 4, "Expected 32-bit uint32_t");
static_assert(sizeof(float) == 4, "Expected 32-bit float");

/**
 * @brief Write a 32-bit value in big-endian order to dst[0..3].
 */
inline void write_u32_be(std::uint8_t* dst, std::uint32_t v) {

    for (std::size_t i = 0; i < kBytesIn32Bit; ++i) {
        const unsigned shift = static_cast<unsigned>((kBytesIn32Bit - 1 - i) * kBitsPerByte);
        dst[i] = static_cast<std::uint8_t>(v >> shift);
    }
}


/**
 * @brief Convert frequency (Hz) to period (ms), using integer math.
 *
 * Uses ceil(1000 / Hz). If Hz == 0, returns 1000ms as a safe fallback.
 */
inline std::uint16_t hz_to_period_ms(std::uint16_t hz) {
    return (hz == 0) ? 1000u
                     : static_cast<std::uint16_t>((1000u + hz - 1u) / hz);
}

} // namespace TelemetryFmt

// Backwards-compatible names if existing code uses START_BYTE / END_BYTE.
static const std::uint8_t START_BYTE = TelemetryFmt::kStartByteValue;
static const std::uint8_t END_BYTE   = TelemetryFmt::kEndByteValue;

/**
 * @brief Declares one telemetry "stream" to include in packets.
 *
 * A stream can be either:
 * - single SDH: write the SDH name followed by its float value
 * - multi SDH group: write a group label followed by N float values
 *
 * This type does not own any SensorDataHandler objects.
 * Pointers must remain valid while Telemetry uses them.
 *
 * For multi groups, you can provide either:
 * - raw pointer + count (most general)
 * - std::array (preferred when size is fixed at compile time)
 */
struct SendableSensorData {
    // --- Payload configuration ---

    /** Single-value stream. If non-null, this stream will send this SDH. */
    SensorDataHandler* singleSDH;

    /** Multi-value group. Pointer to an external array of SDHs. */
    SensorDataHandler* const* multiSDH;

    /** Length of multiSDH array. */
    // Shall not be changed after construction
    // Used to iterate over multiSDH
    const std::size_t multiSDHLength;

    /** Label written once before the multi SDH values. */
    std::uint8_t multiSDHDataLabel;

    // --- Scheduling state ---

    /** Minimum time between sends for this stream. */
    std::uint16_t periodMs;

    /** Last send time in ms (same time base as tick()). */
    std::uint32_t lastSentTimestamp;

    /**
     * @brief Create a single SDH stream.
     * @param sdh SensorDataHandler to send (non-owning).
     * @param sendFrequencyHz Desired send rate in Hz.
     */
    SendableSensorData(SensorDataHandler* sdh, std::uint16_t sendFrequencyHz)
        : singleSDH(sdh),
          multiSDH(0),
          multiSDHLength(0),
          multiSDHDataLabel(0),
          periodMs(TelemetryFmt::hz_to_period_ms(sendFrequencyHz)),
          lastSentTimestamp(0) {}

    /**
     * @brief Create a multi SDH stream from a std::array.
     *
     * This is convenient when the group size is fixed at compile time.
     * The std::array passed in must outlive this SendableSensorData object.
     */
    template <std::size_t M>
    SendableSensorData(const std::array<SensorDataHandler*, M>& sdhList,
                       std::uint8_t label,
                       std::uint16_t sendFrequencyHz)
        : singleSDH(0),
          multiSDH(sdhList.data()),
          multiSDHLength(M),
          multiSDHDataLabel(label),
          periodMs(TelemetryFmt::hz_to_period_ms(sendFrequencyHz)),
          lastSentTimestamp(0) {}

    /**
     * @brief Return true if enough time has elapsed such that this stream wants to be sent again.
     */
    bool shouldBeSent(std::uint32_t now) const {
        return (now - lastSentTimestamp) >= periodMs;
    }

    /**
     * @brief Update internal state after sending.
     */
    void markWasSent(std::uint32_t now) {
        lastSentTimestamp = now;
    }

    /** @brief Convenience: true if configured as a single SDH stream. */
    bool isSingle() const { return singleSDH != 0; }

    /** @brief Convenience: true if configured as a multi SDH stream. */
    bool isMulti() const { return (multiSDH != 0) && (multiSDHLength != 0); }
};

/**
 * @brief Packetizes telemetry streams and sends them out over a Stream.
 *
 * Usage pattern:
 * - Construct Telemetry with a stable list of SendableSensorData pointers.
 * - Call tick(currentTimeMs) every loop.
 *
 * Lifetime rule:
 * - Telemetry stores a pointer to the provided list of streams (non-owning).
 *   The list must outlive Telemetry.
 */
class Telemetry {
public:

    /**
     * @brief Construct from std::array (convenient and compile-time sized).
     * The std::array must outlive the Telemetry instance.
     */
    template <std::size_t N>
    Telemetry(const std::array<SendableSensorData*, N>& streams,
              Stream& rfdSerialConnection)
        : streams(streams.data()),
          streamCount(N),
          rfdSerialConnection(rfdSerialConnection),
          nextEmptyPacketIndex(0),
          packet{} {}
    /**
     * @brief Call every loop to send due telemetry streams.
     * @param currentTimeMs Current time in milliseconds.
     * @return true if a packet was sent on this tick.
     */
    bool tick(std::uint32_t currentTimeMs);

private:
    // Packet building helpers
    void preparePacket(std::uint32_t timestamp);
    void addSingleSDHToPacket(SensorDataHandler* sdh);
    void addSSDToPacket(SendableSensorData* ssd);
    void setPacketToZero();
    void addEndMarker();
    bool hasRoom(std::size_t nextIndex, std::size_t bytesToAdd);

    // Non-owning view of the stream list
    SendableSensorData* const* streams;

    // Number of streams in the list
    // Shall not be changed after construction
    // Used to iterate over streams
    const std::size_t streamCount;

    // Output
    Stream& rfdSerialConnection;

    // Packet state
    std::size_t nextEmptyPacketIndex;
    std::array<std::uint8_t, TelemetryFmt::kPacketCapacity> packet;
};

#endif
