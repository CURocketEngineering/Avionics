#ifndef DATA_SAVER_SD_SERIAL_H
#define DATA_SAVER_SD_SERIAL_H

#include <array>
#include <cstdlib>

#include "ArduinoHAL.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

class DataSaverSDSerial: public IDataSaver {
    // Given data points, will write the data over uart to a serial data logger 

    public:
        // Interval must be less than 65535 ms or about 1 minute
        DataSaverSDSerial(HardwareSerial &SD_serial);
        
        using IDataSaver::saveDataPoint; // Allow the use of the other saveDataPoint overload
        virtual int saveDataPoint(DataPoint dp, uint8_t name) override;

    private:
        HardwareSerial &SD_serial;

};

#endif