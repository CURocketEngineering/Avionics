#ifndef SERIAL_SIM_LSM6DSOX_H
#define SERIAL_SIM_LSM6DSOX_H

#include "Serial_Sim.h"

#ifndef LSM6DS_ACCEL_RANGE_16_G
#define LSM6DS_ACCEL_RANGE_16_G    0x03
#endif

#ifndef LSM6DS_GYRO_RANGE_2000_DPS
#define LSM6DS_GYRO_RANGE_2000_DPS 0x03
#endif

#ifndef LSM6DS_RATE_104_HZ
#define LSM6DS_RATE_104_HZ         0x04
#endif

/**
 * @brief Mock LSM6DSOX IMU backed by SerialSim accelerometer/gyro data.
 * @note When to use: firmware simulation or CI builds where the real IMU is
 *       not available but API compatibility is required.
 */
class Adafruit_LSM6DSOX {
public:
    Adafruit_LSM6DSOX (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization
    bool begin_I2C(int addr) { return true; } // Mock successful initialization
    bool begin_I2C() { return true; } // Mock successful initialization

    void setAccelRange(int range) { accelRange_ = range; }
    void setGyroRange(int range) { gyroRange_ = range; }
    void setAccelDataRate(int rate) { accelDataRate_ = rate; }
    void setGyroDataRate(int rate) { gyroDataRate_ = rate; }

    int getAccelRange() { return accelRange_; }
    int getGyroRange() { return gyroRange_; }
    int getAccelDataRate() { return accelDataRate_; }
    int getGyroDataRate() { return gyroDataRate_; }

    void getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp) {
        SerialSim::getInstance().updateAcl(accel);
        SerialSim::getInstance().updateGyro(gyro);

    }

private:
    int accelRange_ = LSM6DS_ACCEL_RANGE_16_G;
    int gyroRange_ = LSM6DS_GYRO_RANGE_2000_DPS;
    int accelDataRate_ = LSM6DS_RATE_104_HZ;
    int gyroDataRate_ = LSM6DS_RATE_104_HZ;

};

#endif // SERIAL_SIM_LSM6DSOX_H
