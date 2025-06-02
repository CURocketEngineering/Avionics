#pragma once 

#include <iostream>
#include <utility>
#include <vector>

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

class DataSaverMock: public IDataSaver {
    // Mock class if a data saver is ever required but not the focus of the test
    public:
        // Store the calls and their parameters
        std::vector<std::pair<DataPoint, uint8_t>> saveDataPointCalls;

        // Mock saveDataPoint function
        int saveDataPoint(DataPoint dp, uint8_t name) override {
            saveDataPointCalls.emplace_back(dp, name);
            return 0; // Return success
        }

        // Clear the stored calls
        void clear() {
            saveDataPointCalls.clear();
        }

        // Assertions
        void assertSaveDataPointCalledWith(const DataPoint& expectedDp, uint8_t expectedName) {
            bool found = false;
            for (const auto& call : saveDataPointCalls) {
                if (call.first.timestamp_ms == expectedDp.timestamp_ms &&
                    call.first.data == expectedDp.data &&
                    call.second == expectedName) {
                    found = true;
                    break;
                }
            }
        }
};