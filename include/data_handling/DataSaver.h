#ifndef DATA_SAVER_H
#define DATA_SAVER_H

#include "data_handling/DataPoint.h"
#include <string>
#include <iostream>
#include <cstdint>


class IDataSaver {
    // Common interface that all data savers must implement
    // This allows for easy swapping of data savers

    public:
        virtual int saveDataPoint(DataPoint dp, std::string name) = 0;

        // Overloaded function, which converts the data and timestamp to a DataPoint
        virtual int saveDataPoint(float data, uint32_t timestamp_ms, std::string name) final {
            return saveDataPoint(DataPoint(timestamp_ms, data), name);
        }
};

class printSaver : public IDataSaver {
    // This is useful for debugging

    public:
        int saveDataPoint(DataPoint dp, std::string name) override {
            std::cout << name << " " << dp.timestamp_ms << " " << dp.data << std::endl;
            return 0;
        }
};


#endif