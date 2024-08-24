#pragma once 

#include <iostream>
#include <gmock/gmock.h>

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

class DataSaverMock: public IDataSaver {
    // Mock class if a data saver is ever required but not the focus of the test
    public:
        MOCK_METHOD(int, saveDataPoint, (DataPoint dp, uint8_t name), (override));
};