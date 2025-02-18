#ifndef SERIAL_SIM_H
#define SERIAL_SIM_H

#include <string>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>

#define LSM6DS_ACCEL_RANGE_16_G         0x03  
#define LSM6DS_GYRO_RANGE_2000_DPS      0x03  
#define LSM6DS_RATE_104_HZ              0x04  
// LIS3MDL (Magnetometer) Define
#define LIS3MDL_DATARATE_155_HZ         0x06  
#define LIS3MDL_RANGE_4_GAUSS           0x01  
#define LIS3MDL_CONTINUOUSMODE          0x00  
#define LIS3MDL_MEDIUMMODE              0x01  
// BMP3 (Barometric Pressure Sensor) Define
#define BMP3_OVERSAMPLING_8X            0x03  
#define BMP3_OVERSAMPLING_4X            0x02  
#define BMP3_IIR_FILTER_COEFF_3         0x03  
#define BMP3_ODR_100_HZ                 0x05 

HardwareSerial SUART1(PB7, PB6);

class SerialSim {
public:
    static SerialSim& getInstance() {
        static SerialSim instance;  // Single instance
        return instance;
    }

    void begin() {
        SUART1.begin(115200);
        while (!SUART1);
        SUART1.write('S'); 
        SUART1.write('T'); 
        SUART1.write('A'); 
        SUART1.write('R'); 
        SUART1.write('T');

    }

    void readIncomingData()
    {
        incomingData = SUART1.readStringUntil('\n');
    }

    bool serialAvalible(void){
        return SUART1.available();
    }

    void updateTimeStamp(float &timestamp)
    {
        timestamp = parseNextFloat(incomingData);
    }

    void updateAcl(sensors_event_t *accel) {
        accel->acceleration.x = parseNextFloat(incomingData);
        accel->acceleration.y = parseNextFloat(incomingData);
        accel->acceleration.z = parseNextFloat(incomingData);

    }

    void updateGyro(sensors_event_t *gyro) {
        gyro->gyro.x = parseNextFloat(incomingData);
        gyro->gyro.y = parseNextFloat(incomingData);
        gyro->gyro.z = parseNextFloat(incomingData);
    }

    void updateMag(sensors_event_t *mag) {
        mag->magnetic.x = parseNextFloat(incomingData);
        mag->magnetic.y = parseNextFloat(incomingData);
        mag->magnetic.z = parseNextFloat(incomingData);
    }

    void updateAlt(float &alt){
        alt = parseNextFloat(incomingData);
    }

    void updatePres(float &pres){
        pres = parseNextFloat(incomingData);
    }

    void updateTemp(sensors_event_t &temp){
        temp.temperature = incomingData.toFloat();
    }

    void ack(){
        incomingData = "";
        SUART1.write('A');
    }


private:

    // Private constructor to enforce singleton pattern
    SerialSim() {}

    // Private copy constructor and assignment operator to prevent copying
    SerialSim(const SerialSim&) = delete;
    SerialSim& operator=(const SerialSim&) = delete;

    String incomingData;

    float parseNextFloat(String& data) {
        float value = data.substring(0, data.indexOf(',')).toFloat();  // Extract value as float
        data = data.substring(data.indexOf(',') + 1);  // Update data by removing the parsed value
        return value;
    }

};

#endif // SERIAL_SIM_H