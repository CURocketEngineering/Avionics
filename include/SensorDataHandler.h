// Orignally written by Ethan Anderson -- 2/17/2024
// DataHandler.h

#ifndef SensorDataHandler_H
#define SensorDataHandler_H

#include <cstdint>
#include <cstdlib>
#include <vector>
#include <string>
#include <Arduino.h>

#define MAX_DATA_POINTS_READ_ARRAY 16
#define MAX_DATA_POINTS_TEMPORAL_ARRAY 255

// Holds a single data point
// Contains a timestamp in milliseconds and a float data point
// The timestamp should be from some arbitrary start time
// The millis() function in Arduino is a good example
class DataPoint {
public:
    uint32_t timestamp_ms;  // Will roll over every 49.7 days
    float data; // Data size: 4 bytes

    // Default constructor
    DataPoint() : timestamp_ms(0), data(0) {}

    // Constructor
    DataPoint(uint32_t timestamp_ms, float data);
};

// Internal class for storing data in a circular array
// This class is used to store data from a sensor as it comes in
class ReadCircularArray {
public:
    std::vector<DataPoint> data;
    uint8_t head; // Head is the index of the latest data point
    uint8_t maxSize;  // The maximum number of data points that can be stored

    ReadCircularArray();
    DataPoint getLatestData();
};

// Internal class for storing data in a circular array
// Data is stored here from the ReadCircularArray at a fixed interval
class TemporalCircularArray {
public:
    std::vector<DataPoint> data;
    uint8_t head;
    uint8_t maxSize; // The maximum number of data points that can be stored
                     // Will equal size_ms / interval_ms
    uint16_t interval_ms;  // The interval between each data point

    TemporalCircularArray(uint16_t interval_ms, uint16_t size_ms);
    DataPoint getLatest();
    DataPoint getMedian();


    /*
     * @param milliseconds: The number of milliseconds older than the latest data point
     * @return: The data point closest to the specified number of milliseconds
     *         If the specified number of milliseconds is greater than the size of the array, the oldest data point is returned
     */
    DataPoint getHistoricalData(uint16_t milliseconds);
};

// This class is used to store data from a sensor
// Stores data in a pair of circular arrays
// The temporal array stores data with evenly spaced timestamps
//
// @param temporalInterval_ms: The interval between each data point in the temporal array
// @param temporalSize_ms: The size of the temporal array in milliseconds
// @param name: The name of sensor, 3 char limit
class SensorData {
public:
    // Constructor
    SensorData(uint16_t temporalInterval_ms, uint16_t temporalSize_ms, String name);
    // No name constructor
    SensorData(uint16_t temporalInterval_ms, uint16_t temporalSize_ms);

    uint16_t getInterval_ms(); 
    uint16_t getMaxSize(); 

    
    // Returns true when the temporal array is updated
    // Returns false if the data was only put into the read array
    // @param data: The data point to add
    // @param SD_serial: Which serial to save the data to, default is nullptr to not save
    bool addData(DataPoint data, HardwareSerial *SD_serial);

    // Returns the median of the temporal array
    // This the value with the middle most value, not time
    // If there are an even number of elements, the right middle element is returned
    DataPoint getTemporalArrayMedian();

    // Returns the latest data point from the read array
    DataPoint getLatestData();

    /*
    * Returns the newest datapoint that is at least milliseconds old
    */
    DataPoint getHistoricalData(uint16_t milliseconds) {
        return temporalArray.getHistoricalData(milliseconds);
    }

    /*
    * Returns the count-th newest datapoint
    * e.g. getHistoricalDataCount(0) returns the latest data point
    */
    DataPoint getHistoricalDataCount(uint16_t count) {
        // Head and count are both unsigned
        return temporalArray.data[(temporalArray.head + temporalArray.maxSize - count) % temporalArray.maxSize];
    }

    // Sets the minimum time between each data point that is saved to the SD card
    void restrictSaveSpeed(uint16_t interval_ms);

    bool isFull(){
        return readArray.data.size() == readArray.maxSize;
    }

    String name;
    char error; // Error code, 0 is no error
private:
    ReadCircularArray readArray;
    TemporalCircularArray temporalArray;

    uint16_t saveInterval_ms; // The minimum time between each data point that is saved to the SD card
    uint32_t lastSaveTime_ms; // The last time a data point was saved to the SD card
};

void dataToSDCardSerial(String name, uint32_t timestamp_ms, float data, HardwareSerial &SD_serial);


// Testing scripts
void test_DataPoint();
void test_ReadCircularArray();
void test_TemporalCircularArray();
void test_SensorData();

void test_DataHandler();

#endif // DATAHANDLER_H