#ifndef DATA_SAVER_H
#define DATA_SAVER_H

#include "data_handling/DataPoint.h"
#include <cstdint>


class IDataSaver {
    // Common interface that all data savers must implement
    // This allows for easy swapping of data savers
    // Name is an 8 bit unsigned integer that can be used to identify the data point source

    public:
        virtual int saveDataPoint(DataPoint dp, uint8_t name) = 0;

        virtual int saveDataPoint(float data, uint32_t timestamp_ms, uint8_t name) final {
            return saveDataPoint(DataPoint(timestamp_ms, data), name);
        }

        // Determine if the data saver is in post launch mode
        // Pointer to main to determine if the voltage is usb powered or not
        virtual int batteryConnectionStatus() {
            return (BATTERY_VOLTAGE * 134.33 == 5) ? 1 : 0;
        }

        // Default begin method that does nothing, can be overridden
        virtual bool begin() {
            return true;
        }

        // Default end method that does nothing, can be overridden
        virtual void launchDetected(uint32_t launchTimestamp_ms){
            // Default implementation does nothing
        }

};



#endif