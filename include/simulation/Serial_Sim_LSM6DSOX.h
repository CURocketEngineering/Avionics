#ifndef SERIAL_SIM_LSM6DSOX_H
#define SERIAL_SIM_LSM6DSOX_H

#include "Serial_Sim.h"

class Adafruit_LSM6DSOX {
public:
    Adafruit_LSM6DSOX (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization

    void setAccelRange(int range) {}
    void setGyroRange(int range) {}
    void setAccelDataRate(int rate) {}
    void setGyroDataRate(int rate) {}

    int getAccelRange() { return 16; } // Mock as 16G
    int getGyroRange() { return 2000; } // Mock as 2000 DPS
    int getAccelDataRate() { return 104; } // Mock as 104 Hz
    int getGyroDataRate() { return 104; } // Mock as 104 Hz

    void getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp) {
        SerialSim::getInstance().updateAcl(accel);
        SerialSim::getInstance().updateGyro(gyro);

    }


};

#endif // SERIAL_SIM_LSM6DSOX_H
