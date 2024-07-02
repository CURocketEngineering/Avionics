#pragma once 

#include <iostream>
#include <gmock/gmock.h>

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

class DataSaverMock: public IDataSaver {
    public:
        MOCK_METHOD(int, saveDataPoint, (DataPoint dp, std::string name), (override));
};