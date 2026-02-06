#ifndef PowerManagement_H
#define PowerManagement_H

#include "Arduino.h"
#include "pins.h"

/**
 * @brief Simple ADC-based battery voltage helper.
 * This class provides a way to read the battery voltage using an ADC pin and a scaling factor.
 * It also includes a simple "isAlive" check to determine if the voltage is above a certain threshold.
 * 
 * The correct pin can be found in the hardware scematics under "ADC_VOLTAGE"
 * and the scaling factor can be calculated based on the voltage divider used in the hardware design. 
 * For MARTHA 1.4, the voltage divider is 200k and 1k5, so the factor is (200k + 1k5) / 1k5 = 134.33333.
 * 
 * The voltage at the pin is calculated as:
 * vPin = (rawValue / (2^numAdcBits - 1)) * vRef
 * where rawValue is the ADC reading, numAdcBits is the resolution of the ADC, and vRef is the reference voltage for the ADC (3.3V for MARTHA 1.4).
 * The battery voltage is then calculated as:
 * vBat = vPin * factor
 */
class BatteryVoltage {
    public:
        /**
         * @param adcPin The ADC pin connected to the battery voltage divider.
         * @param factor The scaling factor to convert the pin voltage to battery voltage.
         * @param numAdcBits The resolution of the ADC (e.g., 12 for 12-bit ADC).
         */
        BatteryVoltage(uint8_t adcPin, float factor, int numAdcBits)
            : pin(adcPin), factor(factor), numAdcBits(numAdcBits) {analogReadResolution(numAdcBits);}
    
        /**
         * @brief Sample the ADC and convert to battery voltage.
         * @return Converted voltage reading.
         */
        float readVoltage() {
            // Set the pin for reading
            pinMode(pin, INPUT);
            uint32_t rawValue = analogRead(pin);  // Should give a value between 0 and 2^numAdcBits - 1
            float vRef = 3.3f; // reference voltage for the ADC on MARTHA 1.4

            // Convert raw ADC value to voltage at the pin, then apply the factor to get battery voltage
            float vPin = (static_cast<float>(rawValue) / (static_cast<float>(1 << numAdcBits) - 1)) * vRef;
            float vBat = vPin * factor;

            return vBat;
        }
      
        /**
         * @brief Check whether voltage exceeds a minimal threshold.
         * @return true if voltage is above the survival threshold.
         * @note When to use: lightweight go/no-go checks before more detailed
         *       power analysis.
         */
        bool isAlive(){
            return readVoltage() > .3; // Not sure what a good cutoff is 
    }
    
    private:
        uint8_t pin;
        float factor;
        int numAdcBits;
    };
    
    #endif // PowerManagement_H