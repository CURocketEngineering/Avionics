#ifndef SERIAL_SIM_LIS3MDL_H
#define SERIAL_SIM_LIS3MDL_H

#include "Serial_Sim.h"

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

    void setDataRate(int rate) {}
    void setRange(int range) {}
    void setOperationMode(int mode) {}
    void setPerformanceMode(int mode) {}
    void setIntThreshold(int threshold) {}
    void configInterrupt(bool a, bool b, bool c) {}
    void configInterrupt(bool a, bool b, bool c, bool d, bool e, bool f) {}
    int getPerformanceMode() { return 0; } // Mock as 0
    int getOperationMode() { return 0; } // Mock as 0
    int getRange() { return 4; } // Mock as 4 Gauss
    uint16_t getIntThreshold() { return 0; } // Mock as 0



    int getDataRate() { return 155; } // Mock as 155 Hz

    void getEvent(sensors_event_t *mag) {
        SerialSim::getInstance().updateMag(mag);

    }

};

#endif // SERIAL_SIM_LIS3MDL_H
