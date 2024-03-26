// Orignally written by Ethan Anderson -- 2/17/2024

// Setting up the Serial port for debugging if the Arduino.h library is included
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define DEBUG Serial
#endif

#include "SensorDataHandler.h"
#include <algorithm>
#include <SPI.h>
#include <SD.h>

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
        head = (head + 1) % maxSize;
        array[head] = data;
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
/*
 * Returns the newest data point in the temporal array that is at least <milliseconds> older than the newest data point
 * @param milliseconds: The number of milliseconds older than the latest data point
 * @return: The data point closest to the specified number of milliseconds
 *         If the specified number of milliseconds is greater than the size of the array, the oldest data point is returned
 */
DataPoint TemporalCircularArray::getHistoricalData(uint16_t milliseconds){
    if(data.empty()){
        return DataPoint();
    }
    else{

        uint32_t targetTimestamp = data[head].timestamp_ms - milliseconds;
        uint8_t index = head;

        // The oldestIndex depends on the how much of the array is filled
        uint8_t oldestIndex;
        if (data.size() < maxSize){
            oldestIndex = 0;
        }
        else{
            oldestIndex = (head + 1) % maxSize;
        }

        // If the oldest value is newer than the target, then just return the oldest value
        // Also if the milliseconds to go backwards is greater than the number of milliseconds since the oldest value
        // then just return the oldest value
        if (data[oldestIndex].timestamp_ms > targetTimestamp || data[head].timestamp_ms < milliseconds){
            return data[oldestIndex];
        }

        // Moving the index back until the timestamp is less than the target
        while(data[index].timestamp_ms > targetTimestamp){
            // To move to to the previous index, we need to subtract 1
            // If the index is 0, then we need to wrap around to the end of the array
            if (index == 0){
                index = maxSize - 1;
            }
            else{
                index--;
            }
        }
        return data[index];
    }
}

// Constructor without name 
SensorData::SensorData(uint16_t temporalInterval_ms, uint16_t temporalSize_ms) : temporalArray(temporalInterval_ms, temporalSize_ms){
    this->name = "NA";
    readArray = ReadCircularArray();
}

SensorData::SensorData(uint16_t temporalInterval_ms, uint16_t temporalSize_ms, String name) : temporalArray(temporalInterval_ms, temporalSize_ms){
    this->name = name;
    readArray = ReadCircularArray();
}

bool dataToSDCard(String name, uint16_t timestamp_ms, float data){
    // Append the data to the file
    File dataFile = SD.open(name + ".csv", FILE_WRITE);
    if (dataFile){
        dataFile.println(String(timestamp_ms) + "," + String(data));
        dataFile.close();
        return true;
    }
    else{
        #ifdef DEBUG
        DEBUG.println("Error opening file");
        #endif
        return false;
    }
}

/*
* Adds a datapoint to the read array, if the data is old enough, it is also added to the temporal array
*/
bool SensorData::addData(DataPoint data){
    addDatatoCircularArray(readArray.data, readArray.head, readArray.maxSize, data);

    // If a full cycle of the read array was just completed, then save all the data in the read array to the SD card
    for (int i = 0; i < readArray.maxSize; i++){
        dataToSDCard(name, readArray.data[i].timestamp_ms, readArray.data[i].data);
    }


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
#ifdef TESTING


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

    // Checking the historical data collection method
    assert(temporalArray.getHistoricalData(0).timestamp_ms == 3000);
    assert(temporalArray.getHistoricalData(100).timestamp_ms == 2000);
    assert(temporalArray.getHistoricalData(200).timestamp_ms == 2000);
    assert(temporalArray.getHistoricalData(1000).timestamp_ms == 2000);
    assert(temporalArray.getHistoricalData(1100).timestamp_ms == 1000);
    // A super large number should return the oldest data
    assert(temporalArray.getHistoricalData(10000).timestamp_ms == 1000);

    TemporalCircularArray temporalArray2 = TemporalCircularArray(100, 1000);
    // Inserting 20 points and checking that each index is as expected
    for (int i = 0; i < 20; i++){
        DataPoint dataPoint = DataPoint(100 + i * 100, i);
        addDatatoCircularArray(temporalArray2.data, temporalArray2.head, temporalArray2.maxSize, dataPoint);
    }
    assert(temporalArray2.data.size() == 10);

    // Checking every index for the correct value
    assert(temporalArray2.data[0].data == 10);
    assert(temporalArray2.data[1].data == 11);
    assert(temporalArray2.data[2].data == 12);
    assert(temporalArray2.data[3].data == 13);
    assert(temporalArray2.data[4].data == 14);
    assert(temporalArray2.data[5].data == 15);
    assert(temporalArray2.data[6].data == 16);
    assert(temporalArray2.data[7].data == 17);
    assert(temporalArray2.data[8].data == 18);
    assert(temporalArray2.data[9].data == 19);
    assert(temporalArray2.getLatest().data == 19);
}


void test_SensorData_historical_data(){
    SensorData sensorData = SensorData(100, 1000);
    DataPoint dataPoint = DataPoint(20, 5.0);

    // Adding 20 data points that will all go into the temporal array
    for (int i = 0; i < 20; i++){
        DataPoint dataPoint = DataPoint(100 + i * 100, i);
        assert(sensorData.addData(dataPoint) == true);
    }

    // Only the 10 most recent data points are still in the temporal array
    // because this sensorData is only setup to hold 10 data points b/c of the interval and size
    // i.e. 1000 / 100 = 10

    // The values that should be in the temporal array are:
    // (1100, 10), (1200, 11), (1300, 12), (1400, 13), (1500, 14), (1600, 15), (1700, 16), (1800, 17), (1900, 18), (2000, 19)
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 1600);
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 15.0));

    assert(sensorData.getHistoricalData(0).timestamp_ms == 2000);
    assert(sensorData.getHistoricalData(100).timestamp_ms == 1900);
    assert(sensorData.getHistoricalData(200).timestamp_ms == 1800);
    assert(sensorData.getHistoricalData(250).timestamp_ms == 1700);
    assert(sensorData.getHistoricalData(10000).timestamp_ms == 1100);

    // Doing a fresh sensorData
    SensorData sensorData2 = SensorData(100, 1000);
    // Adding 20 data points but only half should go into the temporal array
    for (int i = 0; i < 20; i++){
        // A spacing of 50ms between each data point, so only every other data point will go into the temporal array
        DataPoint dataPoint = DataPoint(i * 50, i);
        sensorData2.addData(dataPoint);
    }
    Serial.println("Checking half of the data points are in the temporal array");
    // The values that should be in the temporal array are:
    // (100, 2), (200, 4), (300, 6), (400, 8), (500, 10), (600, 12), (700, 14), (800, 16), (900, 18)
    assert(sensorData2.getTemporalArrayMedian().timestamp_ms == 500);
    assert(floatEqual(sensorData2.getTemporalArrayMedian().data, 10.0));
    Serial.println("done");



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

    // Adding 20 more data points with ascending values which all go into the temporal array
    for (int i = 0; i < 20; i++){
        DataPoint dataPoint = DataPoint(400 + i * 100, i);
        sensorData.addData(dataPoint);
    }

    assert(sensorData.getLatestData().timestamp_ms == 400 + 19 * 100);
    assert(floatEqual(sensorData.getLatestData().data, 19));

    // The temporal array only holds 10 values, so the median should be #15
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 400 + 15 * 100);
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 15));

    test_SensorData_historical_data();
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

#endif
