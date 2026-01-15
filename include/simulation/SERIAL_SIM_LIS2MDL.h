#ifndef SERIAL_SIM_LIS2MDL_H
#define SERIAL_SIM_LIS2MDL_H

#include "Serial_Sim.h"

class Adafruit_LIS2MDL {
public:
    Adafruit_LIS2MDL (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization SPI
    bool begin(int addr) { return true; } // Mock successful initialization I2C

    void setDataRate(int rate) {}
    void enableInterrupts(bool a) {}

    int getDataRate() { return 100; } // Mock as 155 Hz

    void getEvent(sensors_event_t *mag) {
        SerialSim::getInstance().updateMag(mag);

    }

};

#endif // SERIAL_SIM_LIS3MDL_H
