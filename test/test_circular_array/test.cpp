#include "test_circular_array.h"
#include "test_dp_circular_array.h"

void setUp(void) {

}

void tearDown(void) {
    // Teardown code, called after each test if necessary
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_CircularArray_MaxSize);
    RUN_TEST(test_CircularArray_EmptyState);
    RUN_TEST(test_CircularArray_Median);
    RUN_TEST(test_CircularArray_Misc);
    RUN_TEST(test_push);
    RUN_TEST(test_fill);
    RUN_TEST(test_max_size);
    RUN_TEST(test_wrapping_and_data_integrity);
    RUN_TEST(test_get_median_odd);
    RUN_TEST(test_get_median_even);
    RUN_TEST(test_clear);
    RUN_TEST(test_assert_fail_when_capacity_less_than_maxSize);
    UNITY_END();
    return 0;
}