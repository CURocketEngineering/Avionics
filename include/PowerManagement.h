#ifndef PowerManagement_H
#define PowerManagement_H

#include "Arduino.h"
#include "pins.h"

/**
 * @brief Simple ADC-based battery voltage helper.
 * @note When to use: quick health checks of the main battery using an analog
 *       pin and known divider scaling.
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