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
        BatteryVoltage(uint8_t adcPin, float factor, int numAdcBits)
            : pin(adcPin), factor(factor), numAdcBits(numAdcBits) {analogReadResolution(numAdcBits);}
    
        /**
         * @brief Sample the ADC and convert to battery voltage.
         * @return Converted voltage reading.
         * @note When to use: periodic health checks or brownout warnings.
         */
        float readVoltage() {
            // Set the pin for reading
            pinMode(pin, INPUT);
            uint32_t rawValue = analogRead(pin);  // Should give a value between 0 and 4095
            float vRef = 3.3f;
            // Conversion notes ->  respect to Vref you would do Vref * analogRead()/2^12. 
            // After that you can use the voltage divider equation to get the battery voltage level
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