#ifndef SERIAL_SIM_LIS3MDL_H
#define SERIAL_SIM_LIS3MDL_H

#include "Serial_Sim.h"

class SerialSimLIS3MDL {
public:
    SerialSimLIS3MDL (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization

    void setDataRate(int rate) {}
    void setRange(int range) {}
    void setOperationMode(int mode) {}
    void setPerformanceMode(int mode) {}

    int getDataRate() { return 155; } // Mock as 155 Hz

    void getEvent(sensors_event_t *mag) {
        SerialSim::getInstance().updateMag(mag);

    }

};

#endif // SERIAL_SIM_LIS3MDL_H
