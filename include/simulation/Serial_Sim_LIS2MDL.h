#ifndef SERIAL_SIM_LIS2MDL_H
#define SERIAL_SIM_LIS2MDL_H

#include "Serial_Sim.h"

#ifndef LIS2MDL_RATE_100_HZ
#define LIS2MDL_RATE_100_HZ 0x06
#endif

class Adafruit_LIS2MDL {
public:
    Adafruit_LIS2MDL (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization SPI
    bool begin(int addr) { return true; } // Mock successful initialization I2C

    void setDataRate(int rate) { dataRate = rate; }
    void enableInterrupts(bool a) {}

    int getDataRate() { return dataRate; }

    void getEvent(sensors_event_t *mag) {
        SerialSim::getInstance().updateMag(mag);

    }

private:
    int dataRate = LIS2MDL_RATE_100_HZ;

};

#endif // SERIAL_SIM_LIS2MDL_H
