#ifndef DATA_SAVER_PRINT_H
#define DATA_SAVER_PRINT_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include <iostream>

/**
 * @brief IDataSaver that prints samples to stdout for debugging.
 * @note When to use: desktop/unit tests where observing the stream matters
 *       more than persisting it.
 */
class printSaver : public IDataSaver {

    public:
        /**
         * @brief Print the data point to the console.
         * @param dp   Data point to display.
         * @param name 8-bit channel identifier.
         * @note When to use: inspect saver traffic interactively while
         *       troubleshooting.
         */
        int saveDataPoint(const DataPoint& dp, uint8_t name) override {
            std::cout << name << " " << dp.timestamp_ms << " " << dp.data << std::endl;
            return 0;
        }
};

#endif