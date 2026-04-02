#include <cstddef>

#include "unity.h"
#include "data_handling/DataPoint.h"
#include "state_estimation/BaseStateMachine.h"
#include "state_estimation/States.h"

namespace {

class TestBaseStateMachine : public BaseStateMachine {
    public:
        explicit TestBaseStateMachine(FlightState initialState = STATE_UNARMED)
            : BaseStateMachine(initialState) {}

        int update(const AccelerationTriplet& accel, const DataPoint& alt) override {
            (void)accel;
            (void)alt;
            return 0;
        }

        bool transitionTo(FlightState newState) {
            return changeState(newState);
        }
};

std::size_t singleCallbackCount = 0;

void singleCallback() {
    singleCallbackCount++;
}

constexpr std::size_t kOverflowTestSlots = TestBaseStateMachine::kMaxStateEntryCallbacks + 1;
std::size_t callbackCounts[kOverflowTestSlots] = {};

template <std::size_t Index>
void indexedCallback() {
    callbackCounts[Index]++;
}

BaseStateMachine::StateEntryCallback overflowCallbacks[kOverflowTestSlots] = {
    &indexedCallback<0>,  &indexedCallback<1>,  &indexedCallback<2>,  &indexedCallback<3>,
    &indexedCallback<4>,  &indexedCallback<5>,  &indexedCallback<6>,  &indexedCallback<7>,
    &indexedCallback<8>,  &indexedCallback<9>,  &indexedCallback<10>, &indexedCallback<11>,
    &indexedCallback<12>, &indexedCallback<13>, &indexedCallback<14>, &indexedCallback<15>,
    &indexedCallback<16>, &indexedCallback<17>, &indexedCallback<18>, &indexedCallback<19>,
    &indexedCallback<20>, &indexedCallback<21>, &indexedCallback<22>, &indexedCallback<23>,
    &indexedCallback<24>, &indexedCallback<25>, &indexedCallback<26>, &indexedCallback<27>,
    &indexedCallback<28>, &indexedCallback<29>, &indexedCallback<30>, &indexedCallback<31>,
    &indexedCallback<32>,
};

void resetCallbackCounts() {
    for (std::size_t i = 0; i < kOverflowTestSlots; i++) {
        callbackCounts[i] = 0;
    }
}

void test_callbacks_fire_once_per_state_entry(void) {
    TestBaseStateMachine stateMachine(STATE_UNARMED);
    singleCallbackCount = 0;

    TEST_ASSERT_TRUE(stateMachine.registerOnStateEntry(STATE_ASCENT, singleCallback));

    TEST_ASSERT_TRUE(stateMachine.transitionTo(STATE_ASCENT));
    TEST_ASSERT_EQUAL_UINT32(1, singleCallbackCount);

    TEST_ASSERT_FALSE(stateMachine.transitionTo(STATE_ASCENT));
    TEST_ASSERT_EQUAL_UINT32(1, singleCallbackCount);

    TEST_ASSERT_TRUE(stateMachine.transitionTo(STATE_DESCENT));
    TEST_ASSERT_EQUAL_UINT32(1, singleCallbackCount);

    TEST_ASSERT_TRUE(stateMachine.transitionTo(STATE_ASCENT));
    TEST_ASSERT_EQUAL_UINT32(2, singleCallbackCount);
}

void test_duplicate_callback_registration_is_rejected(void) {
    TestBaseStateMachine stateMachine(STATE_UNARMED);

    TEST_ASSERT_TRUE(stateMachine.registerOnStateEntry(STATE_DESCENT, singleCallback));
    TEST_ASSERT_FALSE(stateMachine.registerOnStateEntry(STATE_DESCENT, singleCallback));
}

void test_nullptr_callback_registration_is_rejected(void) {
    TestBaseStateMachine stateMachine(STATE_UNARMED);

    TEST_ASSERT_FALSE(stateMachine.registerOnStateEntry(STATE_DESCENT, nullptr));
}

void test_callback_registration_capacity_overflow_is_deterministic(void) {
    TestBaseStateMachine stateMachine(STATE_UNARMED);
    resetCallbackCounts();

    for (std::size_t i = 0; i < TestBaseStateMachine::kMaxStateEntryCallbacks; i++) {
        TEST_ASSERT_TRUE(stateMachine.registerOnStateEntry(STATE_ASCENT, overflowCallbacks[i]));
    }

    TEST_ASSERT_FALSE(stateMachine.registerOnStateEntry(
        STATE_ASCENT,
        overflowCallbacks[TestBaseStateMachine::kMaxStateEntryCallbacks]));

    TEST_ASSERT_TRUE(stateMachine.transitionTo(STATE_ASCENT));

    for (std::size_t i = 0; i < TestBaseStateMachine::kMaxStateEntryCallbacks; i++) {
        TEST_ASSERT_EQUAL_UINT32(1, callbackCounts[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(0, callbackCounts[TestBaseStateMachine::kMaxStateEntryCallbacks]);
}

}  // namespace

void setUp(void) {}

void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_callbacks_fire_once_per_state_entry);
    RUN_TEST(test_duplicate_callback_registration_is_rejected);
    RUN_TEST(test_nullptr_callback_registration_is_rejected);
    RUN_TEST(test_callback_registration_capacity_overflow_is_deterministic);
    return UNITY_END();
}
