#ifndef DATA_SAVER_PRINT_H
#define DATA_SAVER_PRINT_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include <iostream>

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