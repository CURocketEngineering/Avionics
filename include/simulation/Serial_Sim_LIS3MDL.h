#ifndef SERIAL_SIM_LIS3MDL_H
#define SERIAL_SIM_LIS3MDL_H

#include "Serial_Sim.h"

#ifndef LIS3MDL_DATARATE_155_HZ
#define LIS3MDL_DATARATE_155_HZ 0x06
#endif

#ifndef LIS3MDL_RANGE_4_GAUSS
#define LIS3MDL_RANGE_4_GAUSS   0x01
#endif

#ifndef LIS3MDL_CONTINUOUSMODE
#define LIS3MDL_CONTINUOUSMODE  0x00
#endif

#ifndef LIS3MDL_MEDIUMMODE
#define LIS3MDL_MEDIUMMODE      0x01
#endif

/**
 * @brief Mock LIS3MDL magnetometer sourcing readings from SerialSim.
 * @note When to use: magnetometer-dependent code paths during desktop or HIL
 *       testing without the physical sensor.
 */
class Adafruit_LIS3MDL {
public:
    Adafruit_LIS3MDL (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization
    bool begin_I2C(int addr) { return true; } // Mock successful initialization
    bool begin_I2C() { return true; } // Mock successful initialization

    void setDataRate(int rate) { dataRate_ = rate; }
    void setRange(int range) { this->range_ = range; }
    void setOperationMode(int mode) { operationMode_ = mode; }
    void setPerformanceMode(int mode) { performanceMode_ = mode; }
    void setIntThreshold(int threshold) { intThreshold_ = static_cast<uint16_t>(threshold); }
    void configInterrupt(bool a, bool b, bool c) {}
    void configInterrupt(bool a, bool b, bool c, bool d, bool e, bool f) {}
    int getPerformanceMode() { return performanceMode_; }
    int getOperationMode() { return operationMode_; }
    int getRange() { return range_; }
    uint16_t getIntThreshold() { return intThreshold_; }



    int getDataRate() { return dataRate_; }

    void getEvent(sensors_event_t *mag) {
        SerialSim::getInstance().updateMag(mag);

    }

private:
    int dataRate_ = LIS3MDL_DATARATE_155_HZ;
    int range_ = LIS3MDL_RANGE_4_GAUSS;
    int operationMode_ = LIS3MDL_CONTINUOUSMODE;
    int performanceMode_ = LIS3MDL_MEDIUMMODE;
    uint16_t intThreshold_ = 0;

};

#endif // SERIAL_SIM_LIS3MDL_H
