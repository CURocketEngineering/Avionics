#ifndef DATACOMPRESSOR_H
#define DATACOMPRESSOR_H

#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"
#include <array>
#include <cstdlib>

class DataSaverSPI: public IDataSaver {
    // Given data points, will write the data and sometimes write the timestamp to the SPI bus
    // The timestamp is only written if the timestamp is greater than the last timestamp by the timestampInterval_ms

    public:
        // Interval must be less than 65535 ms or about 1 minute
        DataSaverSPI(uint16_t timestampInterval_ms, int mosi, int miso, int sck, int cs);
        
        using IDataSaver::saveDataPoint; // Allow the use of the other saveDataPoint overload
        virtual int saveDataPoint(DataPoint dp, uint8_t name) override;

        
        uint32_t getLastTimestamp() {
            return lastTimestamp_ms;
        }

        DataPoint getLastDataPoint(){
            return lastDataPoint;
        }

    private:
        uint16_t timestampInterval_ms; // The interval at which timestamps are written
        uint32_t lastTimestamp_ms; // The last timestamp written
        DataPoint lastDataPoint; // The last data point written

        int mosi;
        int miso;
        int sck;
        int cs;
};

#endif