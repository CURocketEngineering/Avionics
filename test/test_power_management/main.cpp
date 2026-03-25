// test_power_management/main.cpp
#include "unity.h"
#include "PowerManagement.h"
#include "ArduinoHAL.h"
#include <cmath>

// Simple analogRead mock with settable return value
static int g_analog_value = 0;

void setUp(void)  { g_analog_value = 0; }
void tearDown(void) {}

// Test 1: check numeric voltage conversion
void test_readvoltage(void)
{

    uint8_t testPins[3] = {HAL_HIGH_VOLTAGE_ADC_PIN, HAL_MID_VOLTAGE_ADC_PIN, 0};
    float ADCValues[3] = {};
    for (int i = 0; i < 3; i++){
        ADCValues[i] = static_cast<float>(analogRead(testPins[i]));
    }
    const float factor = 134.33333f;

    for (int i = 0; i < 3; i++){
        const float vPin = (ADCValues[i] / (static_cast<float>((1<<12) - 1))) * 3.3f;
        const float expected = vPin * factor;
        BatteryVoltage bv(testPins[i], factor, 12, 3.0f);
        const float measured = bv.readVoltage();
        TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, measured);
    }
}

// Test 2: isLow() returns true at 0V and false at full-scale
void test_isLow_behavior(void)
{
    // Zero reading -> 0V battery
    BatteryVoltage bv_low(HAL_LOW_VOLTAGE_ADC_PIN, 3.3f, 12, /*voltageThreshold*/2.0f);
    TEST_ASSERT_TRUE(bv_low.isLow());

    // Full-scale reading -> large battery voltage; threshold small -> not low
    BatteryVoltage bv_high(HAL_HIGH_VOLTAGE_ADC_PIN, 3.3f, 12, /*voltageThreshold*/0.1f);  // Pin 191 analog read return 0 in HAL
    TEST_ASSERT_FALSE(bv_high.isLow());
}

void test_voltage_calculation_accuracy(void)
{
    // Test with a known ADC value and factor
    g_analog_value = 2048; // Mid-scale for 12-bit ADC
    const float factor = 100.0f; // Arbitrary scaling factor
    BatteryVoltage bv(194, factor, 12, 3.0f);

    const float measured = bv.readVoltage();
    const float vPin = (static_cast<float>(g_analog_value) / (static_cast<float>((1<<12) - 1))) * 3.3f;
    const float expected = vPin * factor;

    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, measured);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_readvoltage);
    RUN_TEST(test_isLow_behavior);
    return UNITY_END();
}
