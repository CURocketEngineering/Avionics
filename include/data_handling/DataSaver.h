#ifndef DATA_SAVER_H
#define DATA_SAVER_H

#include "data_handling/DataPoint.h"
#include <string>
#include <iostream>
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

};

class printSaver : public IDataSaver {
    // This is useful for debugging
    // Just prints what would otherwise be saved to the terminal 

    public:
        int saveDataPoint(DataPoint dp, uint8_t name) override {
            std::cout << name << " " << dp.timestamp_ms << " " << dp.data << std::endl;
            return 0;
        }
};


#endif