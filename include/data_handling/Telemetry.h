#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "ArduinoHAL.h"
#include "UARTCommandHandler.h"
#include "data_handling/SensorDataHandler.h"

/**
 * @file Telemetry.h
 * @brief Packs SensorDataHandler values into a fixed-size byte packet and streams them over a Stream (UART).
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
constexpr std::size_t kPacketCapacity_bytes = 120;

/** Header markers: 3 sync zeros followed by a start byte. */
constexpr std::size_t kSyncZeroCount_bytes = 3;

/** Number of bytes in a packed 32-bit value. */
constexpr std::size_t kBytesInU32_bytes = 4;

/** Header layout: [0..2]=0, [3]=START, [4..7]=timestamp (big-endian). */
constexpr std::size_t kStartByteIndex = kSyncZeroCount_bytes;          // 3
constexpr std::size_t kTimestampIndex = kStartByteIndex + 1; // 4
constexpr std::size_t kPacketCounterIndex = kTimestampIndex + kBytesInU32_bytes; // 8
constexpr std::size_t kHeaderSize_bytes = kSyncZeroCount_bytes + 1 + kBytesInU32_bytes + kBytesInU32_bytes; // 12 (CHANGED from 8 to 12)

/** End marker layout: 3 zeros followed by an end byte. */
constexpr std::size_t kEndMarkerSize_bytes = kSyncZeroCount_bytes + 1;

/** Start-of-packet marker byte value. */
constexpr std::uint8_t kStartByteValue = 51;

/** End-of-packet marker byte value. */
constexpr std::uint8_t kEndByteValue = 52;

/** 32-bit helper constants */
constexpr unsigned kBitsPerByte_bits = 8;
constexpr std::uint32_t kCommandModeInactivityTimeout_ms = 10000;
constexpr std::size_t kCommandEntrySequenceLength = 3;
constexpr char kCommandEntryChar = 'c';

/** Assumptions used by float packing. */
static_assert(sizeof(std::uint32_t) == 4, "Expected 32-bit uint32_t");
static_assert(sizeof(float) == 4, "Expected 32-bit float");

/**
 * @brief Write a 32-bit value in big-endian order to dst[0..3].
 */
inline void writeU32Be(std::uint8_t* dst, std::uint32_t v) {

    for (std::size_t i = 0; i < kBytesInU32_bytes; ++i) {
        const unsigned shift = static_cast<unsigned>((kBytesInU32_bytes - 1 - i) * kBitsPerByte_bits);
        dst[i] = static_cast<std::uint8_t>(v >> shift);
    }
}


/**
 * @brief Convert frequency (Hz) to period (ms).
 *
 * Accepts a float so sub-1 Hz rates can be specified directly
 * (e.g., 0.5f Hz → 2000ms, 0.1f Hz → 10000ms).
 *
 * Uses ceil semantics. If frequency_hz <= 0, returns 1000ms as a safe fallback.
 * Result is clamped to uint16_t range (max 65535ms ≈ 0.015 Hz).
 */
inline std::uint16_t hzToPeriod_ms(float frequency_hz) {
    if (frequency_hz <= 0.0f) {
        return static_cast<std::uint16_t>(1000u);
    }

    // 1000.0 / freq, rounded up
    const float period = 1000.0f / frequency_hz;

    // Ceiling: if there's a fractional part, round up
    auto result = static_cast<std::uint32_t>(period);
    if (static_cast<float>(result) < period) {
        ++result;
    }

    // Clamp to uint16_t max
    if (result > UINT16_MAX) {
        return static_cast<std::uint16_t>(UINT16_MAX);
    }

    return static_cast<std::uint16_t>(result);
}

} // namespace TelemetryFmt

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
    std::uint16_t period_ms;

    /** Last send time in ms (same time base as tick()). */
    std::uint32_t lastSentTimestamp_ms;

    /**
     * @brief Create a single SDH stream.
     * @param sdh SensorDataHandler to send (non-owning).
     * @param sendFrequency_hz Desired send rate in Hz (supports fractional, e.g. 0.5f).
     */
    SendableSensorData(SensorDataHandler* sdh, float sendFrequency_hz)
        : singleSDH(sdh),
          multiSDH(0),
          multiSDHLength(0),
          multiSDHDataLabel(0),
          period_ms(TelemetryFmt::hzToPeriod_ms(sendFrequency_hz)),
          lastSentTimestamp_ms(0) {}

    /**
     * @brief Create a multi SDH stream from a std::array.
     *
     * This is convenient when the group size is fixed at compile time.
     * The std::array passed in must outlive this SendableSensorData object.
     */
    template <std::size_t M>
    SendableSensorData(const std::array<SensorDataHandler*, M>& sdhList,
                       std::uint8_t label,
                       float sendFrequency_hz)
        : singleSDH(0),
          multiSDH(sdhList.data()),
          multiSDHLength(M),
          multiSDHDataLabel(label),
          period_ms(TelemetryFmt::hzToPeriod_ms(sendFrequency_hz)),
          lastSentTimestamp_ms(0) {}

    /**
     * @brief Return true if enough time has elapsed such that this stream wants to be sent again.
     */
    bool shouldBeSent(std::uint32_t now_ms) const {
        return (now_ms - lastSentTimestamp_ms) >= period_ms;
    }

    /**
     * @brief Update internal state after sending.
     */
    void markWasSent(std::uint32_t now_ms) {
        lastSentTimestamp_ms = now_ms;
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
 * - Call tick(currentTime_ms) every loop.
 *
 * Low power mode:
 * - Call registerLowPowerCommand(cli) during setup to add the "lowpower"/"lp" command.
 * - Ground station can then toggle low power mode via the Avionics Shell:
 *     lp <divisor>   — Enable: multiply all stream periods by divisor (2–255)
 *     lp off         — Disable: restore original stream rates
 *     lp status      — Print current state
 * - Divisor-based scaling preserves relative stream priorities.
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
              Stream& rfdSerialConnection,
              CommandLine* commandLine = nullptr)
        : streams_(streams.data()),
          streamCount_(N),
          rfdSerialConnection_(rfdSerialConnection),
          commandLine_(commandLine),
          nextEmptyPacketIndex_(0),
          packet_{},
          lowPowerOriginalPeriods_{} {}

    /**
     * @brief Call every loop to send due telemetry streams.
     * @param currentTime_ms Current time in milliseconds.
     * @return true if a packet was sent on this tick.
     */
    bool tick(std::uint32_t currentTime_ms);

    /**
     * @brief True if telemetry is currently paused for radio command mode.
     */
    bool isInCommandMode() const { return inCommandMode_; }

    /**
     * @brief Optional command line interface to drive while telemetry manages command mode.
     */
    void setCommandLine(CommandLine* newCommandLine) { commandLine_ = newCommandLine; }

    /**
     * @brief Temporarily disable command-mode inactivity timeout.
     * @param lockDuration_ms Duration before auto-unlock fallback.
     */
    void lockCommandModeTimeout(std::uint32_t lockDuration_ms);

    /**
     * @brief Re-enable command-mode inactivity timeout immediately.
     */
    void unlockCommandModeTimeout();

    /**
     * @brief Immediately exit command mode if currently active.
     */
    void forceExitCommandMode();

    // ── Low Power Mode ──────────────────────────────────────────────────

    /**
     * @brief Register the "lowpower" / "lp" command with a CommandLine.
     * @param cli The CommandLine instance to register on.
     *
     * Must be called during setup, after the CommandLine is constructed.
     * Telemetry must outlive the CommandLine (typical for static-lifetime objects).
     */
    void registerLowPowerCommand(CommandLine& cli);

    /**
     * @brief Activate low power mode with the given divisor.
     *
     * Each stream's period_ms is multiplied by divisor.
     * If already active, original rates are restored first before
     * applying the new divisor (avoids compounding).
     *
     * @param divisor Rate divisor, clamped to [2, 255].
     */
    void activateLowPowerMode(std::uint8_t divisor);

    /**
     * @brief Deactivate low power mode, restoring original stream rates.
     * Safe to call when not active (no-op).
     */
    void deactivateLowPowerMode();

    /** @brief True if low power mode is currently active. */
    bool isLowPowerModeActive() const { return lowPowerActive_; }

    /** @brief Current low power divisor (0 if inactive). */
    std::uint8_t getLowPowerDivisor() const { return lowPowerDivisor_; }

private:
    // Packet building helpers
    void preparePacket(std::uint32_t timestamp_ms);
    void addSingleSDHToPacket(SensorDataHandler* sdh);
    void addSSDToPacket(SendableSensorData* ssd);
    void setPacketToZero();
    void addEndMarker();
    void checkForRadioCommandSequence(std::uint32_t currentTime_ms);
    void enterCommandMode(std::uint32_t currentTime_ms);
    void exitCommandMode();
    bool shouldPauseTelemetryForCommandMode(std::uint32_t currentTime_ms);
    bool canFitStreamWithEndMarker(const SendableSensorData* ssd) const;
    void tryAppendStream(SendableSensorData* stream, std::uint32_t currentTime_ms, bool& payloadAdded);
    bool finalizeAndSendPacket();

    // Low power mode helpers
    void saveLowPowerOriginalPeriods();
    void restoreLowPowerOriginalPeriods();
    void applyLowPowerDivisor(std::uint8_t div);

    // Non-owning view of the stream list
    SendableSensorData* const* streams_;

    // Number of streams in the list
    // Shall not be changed after construction
    // Used to iterate over streams
    const std::size_t streamCount_;

    // Output
    Stream& rfdSerialConnection_;
    CommandLine* commandLine_;

    // Packet state
    std::uint32_t packetCounter_ = 0;
    std::size_t nextEmptyPacketIndex_;
    std::array<std::uint8_t, TelemetryFmt::kPacketCapacity_bytes> packet_;

    // Command mode handling
    bool inCommandMode_ = false; 
    std::uint32_t commandModeEnteredTimestamp_ms_ = 0;
    std::uint32_t commandModeLastInputTimestamp_ms_ = 0;
    std::size_t commandEntryProgress_ = 0;
    bool commandModeTimeoutLocked_ = false;
    std::uint32_t commandModeTimeoutLockDeadline_ms_ = 0;

    // Low power mode state
    static constexpr std::size_t kMaxStreams = 32;
    std::uint16_t lowPowerOriginalPeriods_[kMaxStreams];
    bool lowPowerActive_ = false;
    std::uint8_t lowPowerDivisor_ = 0;
};

#endif