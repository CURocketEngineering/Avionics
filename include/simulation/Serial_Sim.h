#ifndef SERIAL_SIM_H
#define SERIAL_SIM_H

#include "ArduinoHAL.h"

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


class SerialSim {
public:
    static SerialSim& getInstance() {
        static SerialSim instance;  // Single instance
        return instance;
    }

    void begin(HardwareSerial *Stream) {
        serial = Stream;
        serial->begin(115200);
        while (!serial);
        serial->write('S'); 
        serial->write('T'); 
        serial->write('A'); 
        serial->write('R'); 
        serial->write('T');

    }

    void readIncomingData()
    {
        incomingData = serial->readStringUntil('\n');
    }

    bool serialAvalible(void){
        return serial->available();
    }

    void update(){
        _timestamp = parseNextFloat(incomingData);
        accel_x = parseNextFloat(incomingData);
        accel_y = parseNextFloat(incomingData);
        accel_z = parseNextFloat(incomingData);
        gyro_x =parseNextFloat(incomingData);
        gyro_y = parseNextFloat(incomingData);
        gyro_z = parseNextFloat(incomingData);
        magnetic_x = parseNextFloat(incomingData);
        magnetic_y = parseNextFloat(incomingData);
        magnetic_z = parseNextFloat(incomingData);
        _alt = parseNextFloat(incomingData);
        _pres = parseNextFloat(incomingData);
        _temp = incomingData.toFloat();
    }

    void updateTimeStamp(float &timestamp)
    {
        timestamp = _timestamp;
    }

    void updateAcl(sensors_event_t *accel) {
        accel->acceleration.x = accel_x;
        accel->acceleration.y = accel_y;
        accel->acceleration.z = accel_z;

    }

    void updateGyro(sensors_event_t *gyro) {
        gyro->gyro.x = gyro_x;
        gyro->gyro.y = gyro_y;
        gyro->gyro.z = gyro_z;
    }

    void updateMag(sensors_event_t *mag) {
        mag->magnetic.x = magnetic_x;
        mag->magnetic.y = magnetic_y;
        mag->magnetic.z = magnetic_z;
    }

    void updateAlt(float &alt){
        alt = _alt;
    }

    void updatePres(float &pres){
        pres = _pres;
    }

    void updateTemp(sensors_event_t &temp){
        temp.temperature = _temp;
    }

    void ack(){
        incomingData = "";
        serial->write('A');
    }


private:

    // Private constructor to enforce singleton pattern
    SerialSim() {}

    // Private copy constructor and assignment operator to prevent copying
    SerialSim(const SerialSim&) = delete;
    SerialSim& operator=(const SerialSim&) = delete;

    String incomingData;

    HardwareSerial *serial;

    float _timestamp;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float magnetic_x;
    float magnetic_y;
    float magnetic_z;
    float _alt;
    float _pres;
    float _temp;


    float parseNextFloat(String& data) {
        float value = data.substring(0, data.indexOf(',')).toFloat();  // Extract value as float
        data = data.substring(data.indexOf(',') + 1);  // Update data by removing the parsed value
        return value;
    }

};

#endif // SERIAL_SIM_H