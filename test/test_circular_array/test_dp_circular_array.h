#include "unity.h"
#include "data_handling/CircularArray.h"
#include "data_handling/SensorDataHandler.h"

void test_CircularArray_MaxSize(void) {
    CircularArray<DataPoint, 5> circularArray(5);
    TEST_ASSERT_EQUAL_INT(5, circularArray.getMaxSize());
}

void test_CircularArray_EmptyState(void) {
    CircularArray<DataPoint, 5> circularArray(5);
    TEST_ASSERT_FALSE(circularArray.isFull());
    TEST_ASSERT_EQUAL_INT(0, circularArray.getHead());
}

void test_CircularArray_Median(void) {
    CircularArray<DataPoint, 5> circularArray(5);

    TEST_ASSERT_EQUAL_FLOAT(0, circularArray.getMedian().data);

    circularArray.push(DataPoint(1, 1.0));
 
    TEST_ASSERT_EQUAL_FLOAT(1.0, circularArray.getMedian().data);

    circularArray.push(DataPoint(2, 2.0));
    circularArray.push(DataPoint(3, 3.0));
    circularArray.push(DataPoint(4, 4.0));
    circularArray.push(DataPoint(5, 5.0));

    TEST_ASSERT_EQUAL_FLOAT(3.0, circularArray.getMedian().data);

    // Check that median is based on data, not timestamps
    circularArray.push(DataPoint(6, 0.0));

    // Median should remain 3.0 because data values are 0, 1, 2, 3, 4
    TEST_ASSERT_EQUAL_FLOAT(3.0, circularArray.getMedian().data);
}

void test_CircularArray_Misc(void) {
    CircularArray<DataPoint, 5> circularArray(5);

    circularArray.push(DataPoint(1, 1.0));
    circularArray.push(DataPoint(2, 2.0));
    circularArray.push(DataPoint(3, 3.0));
    circularArray.push(DataPoint(4, 4.0));
    circularArray.push(DataPoint(5, 5.0));
    circularArray.push(DataPoint(6, 0.0));

    TEST_ASSERT_EQUAL_FLOAT(0.0, circularArray.getFromHead(0).data);
    TEST_ASSERT_EQUAL_FLOAT(5.0, circularArray.getFromHead(1).data);
    TEST_ASSERT_EQUAL_FLOAT(4.0, circularArray.getFromHead(2).data);
    TEST_ASSERT_EQUAL_FLOAT(3.0, circularArray.getFromHead(3).data);
    TEST_ASSERT_EQUAL_FLOAT(2.0, circularArray.getFromHead(4).data);

    TEST_ASSERT_TRUE(circularArray.isFull());
}
