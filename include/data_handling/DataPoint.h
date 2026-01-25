#ifndef DATAPOINT_H
#define DATAPOINT_H

#include <cstdint>

/**
 * @brief Timestamped float measurement container.
 * @note When to use: represent a single sensor or derived value tagged with the
 *       millisecond timestamp at which it was produced.
 */
class DataPoint {
public:
    uint32_t timestamp_ms;  // Will roll over every 49.7 days
    float data; // Data size: 4 bytes

    /**
     * @brief Default construct an empty data point.
     * @note When to use: create a placeholder value before assigning real
     *       readings.
     */
    DataPoint() 
    : timestamp_ms(0), data(0) {}

    /**
     * @brief Construct a populated data point.
     * @param timestamp_ms Measurement time in milliseconds.
     * @param data         Scalar value associated with the timestamp.
     * @note When to use: wrap a raw reading for storage, transport, or
     *       comparison.
     */
    DataPoint(uint32_t timestamp_ms, float data) 
    : timestamp_ms(timestamp_ms), data(data) {};

    // Overload > operator
    friend bool operator>(const DataPoint& lhs, const DataPoint& rhs) {
        return lhs.data > rhs.data;
    }

    // Overload >= operator
    friend bool operator>=(const DataPoint& lhs, const DataPoint& rhs) {
        return lhs.data >= rhs.data;
    }

    // Overload < operator
    friend bool operator<(const DataPoint& lhs, const DataPoint& rhs) {
        return lhs.data < rhs.data;
    }

    // Overload <= operator
    friend bool operator<=(const DataPoint& lhs, const DataPoint& rhs) {
        return lhs.data <= rhs.data;
    }
};

#endif