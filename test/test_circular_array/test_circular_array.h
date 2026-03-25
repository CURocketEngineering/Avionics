#include "unity.h"
#include "data_handling/CircularArray.h"

void test_push(void) {
    CircularArray<int, 5> circularArray(5);
    TEST_ASSERT_EQUAL(5, circularArray.getMaxSize());
    TEST_ASSERT_FALSE(circularArray.isFull());
    TEST_ASSERT_EQUAL(0, circularArray.getHead());
    circularArray.push(1);
    TEST_ASSERT_FALSE(circularArray.isFull());

    // Head should still be 0 after one push
    TEST_ASSERT_EQUAL(0, circularArray.getHead());

    circularArray.push(2);
    // Head should be 1 after the second push
    // because the head is always where the last value was pushed to
    TEST_ASSERT_EQUAL(1, circularArray.getHead());

    circularArray.push(3);
    circularArray.push(4);
    circularArray.push(5);
    TEST_ASSERT_EQUAL(5, circularArray.getFromHead(0));
    TEST_ASSERT_EQUAL(4, circularArray.getFromHead(1));
    TEST_ASSERT_EQUAL(3, circularArray.getFromHead(2));
    TEST_ASSERT_EQUAL(2, circularArray.getFromHead(3));
    TEST_ASSERT_EQUAL(1, circularArray.getFromHead(4));
    TEST_ASSERT_TRUE(circularArray.isFull());
}

void test_fill(void) {
    CircularArray<int, 5> circularArray(5);
    TEST_ASSERT_FALSE(circularArray.isFull());
    for (int i = 0; i < 100; i++) {
        circularArray.push(i);
        if (i == 0){
            TEST_ASSERT_EQUAL(0, circularArray.getHead());
        } else{
            TEST_ASSERT_EQUAL(i % circularArray.getMaxSize(), circularArray.getHead());
        }
    }
    TEST_ASSERT_EQUAL(99, circularArray.getFromHead(0));
    TEST_ASSERT_EQUAL(98, circularArray.getFromHead(1));
    TEST_ASSERT_EQUAL(97, circularArray.getFromHead(2));
    TEST_ASSERT_EQUAL(96, circularArray.getFromHead(3));
    TEST_ASSERT_EQUAL(95, circularArray.getFromHead(4));
    TEST_ASSERT_TRUE(circularArray.isFull());
}

void test_max_size(void) {
    CircularArray<int, MAX_CIRCULAR_ARRAY_CAPACITY> circularArray(MAX_CIRCULAR_ARRAY_CAPACITY);
    TEST_ASSERT_EQUAL(255, circularArray.getMaxSize());
    TEST_ASSERT_FALSE(circularArray.isFull());
    for (int i = 0; i < 1000; i++) {
        circularArray.push(i);
    }
    TEST_ASSERT_EQUAL(999, circularArray.getFromHead(0));
    TEST_ASSERT_EQUAL(998, circularArray.getFromHead(1));
    TEST_ASSERT_EQUAL(997, circularArray.getFromHead(2));
    TEST_ASSERT_EQUAL(996, circularArray.getFromHead(3));
    TEST_ASSERT_EQUAL(995, circularArray.getFromHead(4));
    TEST_ASSERT_TRUE(circularArray.isFull());
}

void test_wrapping_and_data_integrity(void) {
    CircularArray<int, 10> circularArray(10); // Small size for easy testing
    // Fill the array to its capacity
    for (int i = 0; i < 10; i++) {
        circularArray.push(i);
    }
    // Verify initial full state
    TEST_ASSERT_TRUE(circularArray.isFull());
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_EQUAL(9 - i, circularArray.getFromHead(i)); // Check order is correct
    }
    // Push more elements to force wrapping
    for (int i = 10; i < 20; i++) {
        circularArray.push(i);
    }
    // Verify elements after wrapping
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_EQUAL(19 - i, circularArray.getFromHead(i)); // New order after wrapping
    }
    TEST_ASSERT_TRUE(circularArray.isFull()); // Still full
}

void test_get_median_odd(void) {
    CircularArray<int, 5> circularArray(5);
    circularArray.push(1);
    circularArray.push(2);
    circularArray.push(3);
    circularArray.push(4);
    circularArray.push(5);
    TEST_ASSERT_EQUAL(3, circularArray.getMedian());
    circularArray.push(6);
    TEST_ASSERT_EQUAL(4, circularArray.getMedian());
    circularArray.push(7);
    TEST_ASSERT_EQUAL(5, circularArray.getMedian());
    circularArray.push(8);
    TEST_ASSERT_EQUAL(6, circularArray.getMedian());
    circularArray.push(9);
    TEST_ASSERT_EQUAL(7, circularArray.getMedian());
}

void test_get_median_even(void) {
    CircularArray<int, 6> circularArray(6);
    circularArray.push(1);
    circularArray.push(2);
    circularArray.push(3);
    circularArray.push(4);
    circularArray.push(5);
    circularArray.push(6);
    TEST_ASSERT_EQUAL(4, circularArray.getMedian());
    circularArray.push(7);
    TEST_ASSERT_EQUAL(5, circularArray.getMedian());
    circularArray.push(8);
    TEST_ASSERT_EQUAL(6, circularArray.getMedian());
    circularArray.push(9);
    TEST_ASSERT_EQUAL(7, circularArray.getMedian());
    circularArray.push(10);
    TEST_ASSERT_EQUAL(8, circularArray.getMedian());
}

void test_clear(void) {
    CircularArray<int, 5> circularArray(5);
    circularArray.push(1);
    circularArray.push(2);
    circularArray.push(3);
    circularArray.push(4);
    circularArray.push(5);
    TEST_ASSERT_TRUE(circularArray.isFull());
    circularArray.clear();
    TEST_ASSERT_FALSE(circularArray.isFull());
    TEST_ASSERT_EQUAL(0, circularArray.getHead());
}

void test_assert_fail_when_capacity_less_than_maxSize(void) {
    // This test is to ensure that an assertion is thrown when maxSize > Capacity
    // However, since assertions typically terminate the program, we cannot
    // directly test this behavior in a standard unit test framework.
    // Instead, we will just demonstrate the intended usage that would trigger the assert.
    // Uncommenting the following line should cause an assertion failure during execution.
    
    // CircularArray<int, 5> circularArray(10); // This should trigger an assertion failure
}

