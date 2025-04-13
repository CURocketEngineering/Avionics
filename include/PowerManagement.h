#ifndef PowerManagement_H
#define PowerManagement_H

#include "Arduino.h"
#include "pins.h"

class BatteryVoltage {
    public:
        BatteryVoltage(uint8_t adcPin, float conversionFactor)
            : pin(adcPin), factor(conversionFactor) {}
    
        float readVoltage() {
            // Set the pin for reading
            pinMode(pin, INPUT);
            uint32_t rawValue = 0;
            rawValue = analogRead(pin);  // Should give a value between 0 and 1023, not sure how to convert this to the true voltage. 
            // Conversion notes ->  respect to Vref you would do Vref * analogRead()/2^12. 
            // After that you can use the voltage divider equation to get the battery voltage level
            // Vref is 134.33? 
            // float voltage = pow((rawValue/2),12) * factor; 
            Serial.println(rawValue);
            return rawValue;
        }
      
        bool isAlive(){
            return readVoltage() > .3; // Not sure what a good cutoff is 
    }
    
    private:
        uint8_t pin;
        float factor;
    };
    
    #endif // PowerManagement_H