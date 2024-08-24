#ifndef DATAPOINT_H
#define DATAPOINT_H

#include <cstdint>

// timestamp ms, float data pair
class DataPoint {
public:
    uint32_t timestamp_ms;  // Will roll over every 49.7 days
    float data; // Data size: 4 bytes

    // Default constructor
    DataPoint() 
    : timestamp_ms(0), data(0) {}

    // Constructor
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