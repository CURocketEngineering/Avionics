// Written by Ethan Anderson -- 2/17/2024

// Setting up the Serial port for debugging if the Arduino.h library is included
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define DEBUG Serial
#endif

#include "SensorDataHandler.h"
#include <algorithm>

#ifdef DEBUG
#define assert(condition) \
    do { \
        if (!(condition)) { \
            DEBUG.print("Assertion failed in "); \
            DEBUG.print(__FILE__); \
            DEBUG.print(" ("); \
            DEBUG.print(__LINE__); \
            DEBUG.print("): "); \
            DEBUG.println(__func__); \
        } \
    } while(0)
#else
#define assert(condition) ((void)0)
#endif

// Check if floats are equal
bool floatEqual(float a, float b){
    return fabs(a - b) < 0.0001;
}

DataPoint::DataPoint(uint32_t timestamp_ms, float data){
    this->timestamp_ms = timestamp_ms;
    this->data = data;
}

void addDatatoCircularArray(std::vector<DataPoint>& array,
                            uint8_t &head,
                            uint8_t &maxSize,
                            DataPoint data){
    if (array.size() < maxSize){
        array.push_back(data);
        // If this is the first data point, now the array has 
        // a size of 1, so the head should be 0
        // I.e. don't increment the head if the array size is 1
        if (array.size() > 1){
            head++;
        }
    }
    else{
        array[head] = data;
        head = (head + 1) % maxSize;
    }
}

ReadCircularArray::ReadCircularArray(){
    head = 0;
    maxSize = MAX_DATA_POINTS_READ_ARRAY;
    data.reserve(maxSize);
    data.clear();
}

DataPoint ReadCircularArray::getLatestData(){
    if (data.empty()){
        return DataPoint();
    }
    return data[head];
}

TemporalCircularArray::TemporalCircularArray(uint16_t interval_ms, uint16_t size_ms){
    head = 0;

    // Checking if size_ms / interval_ms is larger than the maximum size
    if (size_ms / interval_ms > MAX_DATA_POINTS_TEMPORAL_ARRAY){
        maxSize = MAX_DATA_POINTS_TEMPORAL_ARRAY;
        #ifdef DEBUG
        DEBUG.println("TemporalCircularArray size is too large. Setting to MAX_DATA_POINTS_TEMPORAL_ARRAY");
        #endif
    }
    else{
        maxSize = size_ms / interval_ms;
    }

    this->interval_ms = interval_ms;
    data.reserve(maxSize);
    data.clear();
}


DataPoint TemporalCircularArray::getLatest(){
    if (data.empty()){
        return DataPoint();
    }
    return data[head];
}

/*
* Gets the median (by data) of the data array
* Sorts by data and returns the middle element
# If there are an even number of elements, the right middle element is returned
* @return DataPoint median
*/
DataPoint TemporalCircularArray::getMedian(){
    if(data.empty()){
        return DataPoint();
    }
    else{
        // Sorting the data array and save that to new array
        std::vector<DataPoint> dataCopy = this->data;
        std::sort(dataCopy.begin(), dataCopy.end(), [](DataPoint a, DataPoint b) -> bool {return a.data < b.data;});
        return dataCopy[dataCopy.size() / 2];
    }
}

SensorData::SensorData(uint16_t temporalInterval_ms, uint16_t temporalSize_ms) : temporalArray(temporalInterval_ms, temporalSize_ms){
    readArray = ReadCircularArray();
}

/*
* Adds a datapoint to the read array, if the data is old enough, it is also added to the temporal array
*/
bool SensorData::addData(DataPoint data){
    addDatatoCircularArray(readArray.data, readArray.head, readArray.maxSize, data);
    if(data.timestamp_ms - temporalArray.getLatest().timestamp_ms >= temporalArray.interval_ms){
        addDatatoCircularArray(temporalArray.data, temporalArray.head, temporalArray.maxSize, data);
        return true;
    }
    else{
        return false;
    }
}

DataPoint SensorData::getTemporalArrayMedian(){
    return temporalArray.getMedian();
}

/*
* Get the latest data from the readArray
* @return DataPoint
*/
DataPoint SensorData::getLatestData(){
    return readArray.getLatestData();
}


/*
* Testing Scripts are below
*/


void test_DataPoint(){
    DataPoint dataPoint = DataPoint(1000, 5.0);
    assert(floatEqual(dataPoint.timestamp_ms, 1000));
    assert(floatEqual(dataPoint.data, 5.0));
}

void test_ReadCircularArray(){
    ReadCircularArray readArray = ReadCircularArray();
    DataPoint dataPoint = DataPoint(1000, 5.0);
    addDatatoCircularArray(readArray.data, readArray.head, readArray.maxSize, dataPoint);
    assert(floatEqual(readArray.getLatestData().timestamp_ms, 1000));
    assert(floatEqual(readArray.getLatestData().data, 5.0));
}

void test_TemporalCircularArray(){
    TemporalCircularArray temporalArray = TemporalCircularArray(100, 1000);
    assert(temporalArray.interval_ms == 100);
    assert(temporalArray.maxSize == 10);
    assert(temporalArray.head == 0);

    DataPoint dataPoint = DataPoint(1000, 5.0);
    addDatatoCircularArray(temporalArray.data, temporalArray.head, temporalArray.maxSize, dataPoint);
    assert(temporalArray.data.size() == 1);
    assert(temporalArray.head == 0); // Still 0 because the array size is only 1
    assert(temporalArray.getLatest().timestamp_ms == 1000);
    assert(floatEqual(temporalArray.getLatest().data, 5.0));
    assert(temporalArray.getMedian().timestamp_ms == 1000);
    assert(floatEqual(temporalArray.getMedian().data, 5.0));

    DataPoint dataPoint2 = DataPoint(2000, 10.0);
    addDatatoCircularArray(temporalArray.data, temporalArray.head, temporalArray.maxSize, dataPoint2);
    assert(temporalArray.getLatest().timestamp_ms == 2000);
    assert(temporalArray.getLatest().data == 10.0);
    assert(temporalArray.getMedian().timestamp_ms == 2000);
    assert(temporalArray.getMedian().data == 10);

    DataPoint dataPoint3 = DataPoint(3000, 15.0);
    addDatatoCircularArray(temporalArray.data, temporalArray.head, temporalArray.maxSize, dataPoint3);
    assert(temporalArray.getLatest().timestamp_ms == 3000);
    assert(temporalArray.getLatest().data == 15.0);
    assert(temporalArray.getMedian().timestamp_ms == 2000);
    assert(temporalArray.getMedian().data == 10.0);
}




void test_SensorData(){
    SensorData sensorData = SensorData(100, 1000);
    //                             timestamp, data
    DataPoint dataPoint = DataPoint(20, 5.0);
    assert(sensorData.addData(dataPoint) == false);
    assert(sensorData.getLatestData().timestamp_ms == 20);
    assert(floatEqual(sensorData.getLatestData().data, 5.0));
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 0); // No data in temporal array
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 0)); // No data in temporal array

    DataPoint dataPoint2 = DataPoint(40, 10.0);
    assert(sensorData.addData(dataPoint2) == false);
    assert(sensorData.getLatestData().timestamp_ms == 40);
    assert(sensorData.getLatestData().data == 10.0);
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 0); // No data in temporal array
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 0)); // No data in temporal array

    // First data point that is old enough to be added to the temporal array
    DataPoint dataPoint3 = DataPoint(101, 15.0);
    assert(sensorData.addData(dataPoint3) == true);
    assert(sensorData.getLatestData().timestamp_ms == 101);
    assert(sensorData.getLatestData().data == 15.0);
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 101);
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 15.0));

    // Adding another data point to the read array
    DataPoint dataPoint4 = DataPoint(102, 20.0);
    assert(sensorData.addData(dataPoint4) == false);
    assert(sensorData.getLatestData().timestamp_ms == 102);
    assert(sensorData.getLatestData().data == 20.0);
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 101);
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 15.0));

    // Adding a data point that will be added to the temporal array
    // Now, the values of 15.0 and 25.0 will be in the temporal array, the median is the 25
    DataPoint dataPoint5 = DataPoint(202, 25.0);
    assert(sensorData.addData(dataPoint5) == true);
    assert(sensorData.getLatestData().timestamp_ms == 202);
    assert(sensorData.getLatestData().data == 25.0);
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 202);
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 25));

    // Add one final point to the temporal array that has a really low value to put the 15 back in the median
    DataPoint dataPoint6 = DataPoint(302, 1.0);
    assert(sensorData.addData(dataPoint6) == true);
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 101);
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 15));

    // Adding 20 more data points with acending values which all go into the temporal array
    for (int i = 0; i < 20; i++){
        DataPoint dataPoint = DataPoint(400 + i * 101, i);
        sensorData.addData(dataPoint);
    }

    // The temporal array only holds 10 values, so the median should be #15
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 400 + 15 * 101);
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 15));
}

void test_DataHandler(){
    test_DataPoint();
    test_ReadCircularArray();
    test_TemporalCircularArray();
    test_SensorData();
    #ifdef DEBUG
    DEBUG.println("All tests finished");
    #endif
    exit(0);
}
