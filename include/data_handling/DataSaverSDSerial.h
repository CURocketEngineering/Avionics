#ifndef DATA_SAVER_SD_SERIAL_H
#define DATA_SAVER_SD_SERIAL_H

#include <array>
#include <cstdlib>

#include "ArduinoHAL.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

/**
 * @brief IDataSaver implementation that streams CSV packets over UART.
 * @note When to use: log data to an external serial data logger when file
 *       systems (SD/SPI flash) are unavailable or you need live passthrough.
 */
class DataSaverSDSerial: public IDataSaver {
    // Given data points, will write the data over uart to a serial data logger 

    public:
        /**
         * @brief Create a saver that streams CSV over UART to an external data logger.
         * @param SD_serial Hardware serial interface connected to the logger.
         * @note When to use: quick logging to a serial-equipped recorder when
         *       file systems are unavailable.
         */
        DataSaverSDSerial(HardwareSerial &SD_serial);
        
        using IDataSaver::saveDataPoint; // Allow the use of the other saveDataPoint overload

        /**
         * @brief Write a timestamped value to the serial logger.
         * @param dataPoint Data point to transmit.
         * @param name      8-bit channel identifier transmitted alongside data.
         * @note When to use: continuous logging after the external logger is ready.
         */
        virtual int saveDataPoint(const DataPoint& dataPoint, uint8_t name) override;

    private:
        HardwareSerial &SD_serial;

};

#endif
