// Originally written by Ethan Anderson -- 2/17/2024

#include "data_handling/SensorDataHandler.h"

SensorDataHandler::SensorDataHandler(std::string name, IDataSaver* ds) {
    this->name = name;
    this->dataSaver = ds;
    this->saveInterval_ms = 0;
    this->lastSaveTime_ms = 0;
}

void SensorDataHandler::restrictSaveSpeed(uint16_t interval_ms){
    this->saveInterval_ms = interval_ms;
}


int SensorDataHandler::addData(DataPoint data){
    // Check if the data is old enough to be saved based on the interval
    if (data.timestamp_ms - lastSaveTime_ms > saveInterval_ms){
        dataSaver->saveDataPoint(data, name);
        lastSaveTime_ms = data.timestamp_ms;
    }

    return 0;
}

/*
* Testing Scripts are below
*/
//#define TESTING
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
    Serial.println("Starting SensorData historical data tests");
    SensorData sensorData = SensorData(100, 1000, "test2");
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
    SensorData sensorData2 = SensorData(100, 1000, "test3");
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


    // Verify getHistoricalDataCount 
    assert(sensorData2.getHistoricalDataCount(0).timestamp_ms == 900); // The most recent data point
    assert(sensorData2.getHistoricalDataCount(1).timestamp_ms == 800);
    assert(sensorData2.getHistoricalDataCount(2).timestamp_ms == 700);
    assert(sensorData2.getHistoricalDataCount(3).timestamp_ms == 600);
    assert(sensorData2.getHistoricalDataCount(4).timestamp_ms == 500);
    assert(sensorData2.getHistoricalDataCount(5).timestamp_ms == 400);
    assert(sensorData2.getHistoricalDataCount(6).timestamp_ms == 300);
    assert(sensorData2.getHistoricalDataCount(7).timestamp_ms == 200);
    assert(sensorData2.getHistoricalDataCount(8).timestamp_ms == 100);
    assert(sensorData2.getHistoricalDataCount(0).timestamp_ms - sensorData2.getHistoricalDataCount(1).timestamp_ms == 100);

    // Add one more datapoint and see if it all updates well
    DataPoint dataPoint2 = DataPoint(1200, 19);
    sensorData2.addData(dataPoint2);

    assert(sensorData2.getHistoricalDataCount(0).timestamp_ms == 1200); // The most recent data point
    assert(sensorData2.getHistoricalDataCount(1).timestamp_ms == 900);

    // Add another data point that is old enough to be added to the temporal array
    DataPoint dataPoint3 = DataPoint(1300, 20);
    sensorData2.addData(dataPoint3);
    assert(sensorData2.getHistoricalDataCount(0).timestamp_ms == 1300);
    assert(sensorData2.getHistoricalDataCount(1).timestamp_ms == 1200);


    Serial.println("done");



}


void test_SensorData(){
    Serial.println("Starting SensorData tests");
    SensorData sensorData = SensorData(100, 1000, "test");
    //                             timestamp, data
    Serial.println("Constructed a SensorData Object");
    DataPoint dataPoint = DataPoint(20, 5.0);
    assert(sensorData.addData(dataPoint) == false);
    Serial.println("Added Data");
    assert(sensorData.getLatestData().timestamp_ms == 20);
    assert(floatEqual(sensorData.getLatestData().data, 5.0));
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 0); // No data in temporal array
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 0)); // No data in temporal array

    Serial.println("c1");

    DataPoint dataPoint2 = DataPoint(40, 10.0);
    assert(sensorData.addData(dataPoint2) == false);
    assert(sensorData.getLatestData().timestamp_ms == 40);
    assert(sensorData.getLatestData().data == 10.0);
    assert(sensorData.getTemporalArrayMedian().timestamp_ms == 0); // No data in temporal array
    assert(floatEqual(sensorData.getTemporalArrayMedian().data, 0)); // No data in temporal array

    Serial.println("c2");

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
