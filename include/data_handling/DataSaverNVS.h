#ifndef NVS_H
#define NVS_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

#include <Arduino.h>
#include <Preferences.h>


extern Preferences preferences;

class DataSaverNVS : public IDataSaver 
{
    public:
        // void setup(const int baud);
        // void close_nvs();
        // void nvs_write_int(u8_t itr, const char* key);
        // void nvs_read_int(const char* key);
        // void nvs_write_str(const String str, const char* key);
        // void nvs_read_str(const char* key);

        virtual int saveDataPoint(DataPoint dp, uint8_t name) override;
        
};

#endif