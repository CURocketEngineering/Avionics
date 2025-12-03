#ifndef SERIAL_SIM_H
#define SERIAL_SIM_H

#include "Adafruit_Sensor.h"
#include "ArduinoHAL.h"
#include "state_estimation/BurnoutStateMachine.h"
#include "state_estimation/StateMachine.h"

#define LSM6DS_ACCEL_RANGE_16_G         0x03  
#define LSM6DS_GYRO_RANGE_2000_DPS      0x03  
#define LSM6DS_RATE_104_HZ              0x04  
// LIS3MDL (Magnetometer 1.3) Define
#define LIS3MDL_DATARATE_155_HZ         0x06  
#define LIS3MDL_RANGE_4_GAUSS           0x01  
#define LIS3MDL_CONTINUOUSMODE          0x00  
#define LIS3MDL_MEDIUMMODE              0x01  
// LIS2MDL (Magnetometer 1.4) Define
#define LIS2MDL_RATE_100_HZ             0x06
// BMP3 (Barometric Pressure Sensor) Define
#define BMP3_OVERSAMPLING_8X            0x03  
#define BMP3_OVERSAMPLING_4X            0x02  
#define BMP3_IIR_FILTER_COEFF_3         0x03  
#define BMP3_ODR_100_HZ                 0x05 

class SerialSim {
public:
    static SerialSim& getInstance() {
        static SerialSim instance;  // Singleton
        return instance;
    }

    void begin(Stream *inStream, BaseStateMachine *stateMachine) {
        serial = inStream;
        this->stateMachine = stateMachine;

        // Handshake: send "START\n" until we get ACK (0x06)
        uint8_t ack = 0;
        while (ack != 0x06) {
            serial->write("START\n");
            delay(100);
            if (serial->available() > 0) {
                ack = serial->read();
            }
        }
    }

    // Non‐blocking update: 
    // 1) Read as many characters as available
    // 2) If we detect a newline, parse that line
    void update() {
        int availableBytes = serial->available();

        while (serial->available() > 0) {
            char c = static_cast<char>(serial->read());

            // If we get a newline, parse the line
            if (c == '\n') {
                // We have a full line in _partialLine
                handleIncomingLine(_partialLine);
                _partialLine = "";  // Clear for the next line
            } 
            else {
                _partialLine += c;
            }
        }
    }

    bool serialAvailable(void){
        return serial->available() > 0;
    }

    // These next update functions provide the sensor data to calling code
    void updateTimeStamp(float &timestamp) {
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

private:
    SerialSim() {}  // Private constructor for singleton
    SerialSim(const SerialSim&) = delete;
    SerialSim& operator=(const SerialSim&) = delete;

    // Called once a full line of data is received
    void handleIncomingLine(String &line) {
        Serial.println("Handling line: " + line);
        if (line.length() < 5) {
            Serial.println("Line too short");
            return; // not enough data to parse anything meaningful
        }

        // We'll parse each comma‐separated float in the line
        _timestamp   = parseNextFloat(line);
        accel_x      = parseNextFloat(line);
        accel_y      = parseNextFloat(line);
        accel_z      = parseNextFloat(line);
        _alt         = parseNextFloat(line);
        // gyro_x       = parseNextFloat(line);
        // gyro_y       = parseNextFloat(line);
        // gyro_z       = parseNextFloat(line);
        // magnetic_x   = parseNextFloat(line);
        // magnetic_y   = parseNextFloat(line);
        // magnetic_z   = parseNextFloat(line);
       
        // _pres        = parseNextFloat(line);
        // _temp        = line.toFloat(); // The remaining chunk is the last float

        Serial.println("Parsed Data!");

        // After parsing the line, send an ACK
        ack();
    }

    float parseNextFloat(String &data) {
        int commaIndex = data.indexOf(',');
        if (commaIndex == -1) {
            // If there's no comma, parse as much as we can
            float val = data.toFloat();
            data = "";
            return val;
        }

        // Parse substring up to the comma
        float value = data.substring(0, commaIndex).toFloat();
        // Remove parsed portion (including the comma)
        data = data.substring(commaIndex + 1);
        return value;
    }

    // Send ACK (0x06) alongside the current state to the serial port
    void ack(){
        serial->write(stateMachine->getState());
        serial->write(0x06);
    }

private:
    Stream* serial = nullptr;
    BaseStateMachine* stateMachine = nullptr;
    String  _partialLine;  // used to accumulate characters until newline

    // Parsed sensor data
    float _timestamp = 0;
    float accel_x = 0, accel_y = 0, accel_z = 0;
    float gyro_x  = 0, gyro_y  = 0, gyro_z  = 0;
    float magnetic_x = 0, magnetic_y = 0, magnetic_z = 0;
    float _alt  = 0;
    float _pres = 0;
    float _temp = 0;
};

#endif // SERIAL_SIM_H
