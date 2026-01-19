#ifndef DATA_SAVER_H
#define DATA_SAVER_H

#include "data_handling/DataPoint.h"
#include <cstdint>

 
/**
 * @brief Abstract interface for persisting timestamped data points.
 * @note When to use: derive from this class to implement a specific persistence
 *       backend (e.g., SD card, SPI flash, telemetry).
 */
class IDataSaver {
    public:
        /**
         * @brief Persist a data point with a source identifier.
         * @param dp   Data point to store.
         * @param name 8-bit identifier (sensor/channel) saved alongside the
         *             value.
         * @note When to use: primary write path for subclasses; implement in
         *       each concrete saver.
         */
        virtual int saveDataPoint(const DataPoint& dp, uint8_t name) = 0;

        /**
         * @brief Convenience overload to construct and save a data point.
         * @param data          Measurement value.
         * @param timestamp_ms  Timestamp of the measurement in milliseconds.
         * @param name          8-bit source identifier.
         * @note When to use: callers that have raw values but not a DataPoint
         *       object.
         */
        virtual int saveDataPoint(float data, uint32_t timestamp_ms, uint8_t name) final {
            return saveDataPoint(DataPoint(timestamp_ms, data), name);
        }

        /**
         * @brief Optional hook for initialization.
         * @note When to use: override if the saver needs hardware/filesystem
         *       setup before use.
         */
        virtual bool begin() {
            return true;
        }

        /**
         * @brief Notification that launch has been detected.
         * @param launchTimestamp_ms Timestamp of launch, in milliseconds.
         * @note When to use: override to change write policy once launch is
         *       confirmed (e.g., lock buffers or mark critical data).
         */
        virtual void launchDetected(uint32_t launchTimestamp_ms){
            // Default implementation does nothing
        }

};



#endif