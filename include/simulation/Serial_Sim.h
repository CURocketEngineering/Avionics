#ifndef SERIAL_SIM_H
#define SERIAL_SIM_H

#include "Adafruit_Sensor.h"
#include "ArduinoHAL.h"
#include "state_estimation/BurnoutStateMachine.h"
#include "state_estimation/StateMachine.h"

/**
 * @brief Serial-based sensor/flight simulation singleton for hardware-in-the-loop.
 * @note When to use: feed prerecorded or live PC-side simulation data into
 *       firmware during development without real sensors.
 */
class SerialSim {
public:
    static SerialSim& getInstance() {
        static SerialSim instance;  // Singleton
        return instance;
    }

    void begin(Stream *inStream, BaseStateMachine *stateMachine) {
        serial_ = inStream;
        this->stateMachine_ = stateMachine;

        // Handshake: send "START\n" until we get ACK (0x06)
        uint8_t ack = 0;
        while (ack != 0x06) {
            serial_->write("START\n");
            delay(100);
            if (serial_->available() > 0) {
                ack = serial_->read();
            }
        }
    }

    // Non‐blocking update: 
    // 1) Read as many characters as available
    // 2) If we detect a newline, parse that line
    void update() {
        while (serial_->available() > 0) {
            char c = static_cast<char>(serial_->read());

            // If we get a newline, parse the line
            if (c == '\n') {
                // We have a full line in partialLine_
                handleIncomingLine(partialLine_);
                partialLine_ = "";  // Clear for the next line
            } 
            else {
                partialLine_ += c;
            }
        }
    }

    bool serialAvailable(void){
        return serial_->available() > 0;
    }

    // These next update functions provide the sensor data to calling code
    void updateTimeStamp(float &timestamp) {
        timestamp = timestamp_;
    }

    void updateAcl(sensors_event_t *accel) {
        accel->acceleration.x = accelX_;
        accel->acceleration.y = accelY_;
        accel->acceleration.z = accelZ_;
    }

    void updateGyro(sensors_event_t *gyro) {
        gyro->gyro.x = gyroX_;
        gyro->gyro.y = gyroY_;
        gyro->gyro.z = gyroZ_;
    }

    void updateMag(sensors_event_t *mag) {
        mag->magnetic.x = magneticX_;
        mag->magnetic.y = magneticY_;
        mag->magnetic.z = magneticZ_;
    }

    void updateAlt(float &alt){
        alt = alt_;
    }

    void updatePres(float &pres){
        pres = pres_;
    }

    void updateTemp(sensors_event_t &temp){
        temp.temperature = temp_;
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
        timestamp_   = parseNextFloat(line);
        accelX_      = parseNextFloat(line);
        accelY_      = parseNextFloat(line);
        accelZ_      = parseNextFloat(line);
        alt_         = parseNextFloat(line);
        // gyroX_       = parseNextFloat(line);
        // gyroY_       = parseNextFloat(line);
        // gyroZ_       = parseNextFloat(line);
        // magneticX_   = parseNextFloat(line);
        // magneticY_   = parseNextFloat(line);
        // magneticZ_   = parseNextFloat(line);
       
        // pres_        = parseNextFloat(line);
        // temp_        = line.toFloat(); // The remaining chunk is the last float

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

    // Send ACK alongside the current state to the serial_ port
    void ack(){
        serial_->write(stateMachine_->getState());
        // Ack will be a series of 0xaa, 0xbb, 0xcc
        serial_->write(0xaa);
        serial_->write(0xbb);
        serial_->write(0xcc);
    }

private:
    Stream* serial_ = nullptr;
    BaseStateMachine* stateMachine_ = nullptr;
    String  partialLine_;  // used to accumulate characters until newline

    // Parsed sensor data
    float timestamp_ = 0;
    float accelX_ = 0, accelY_ = 0, accelZ_ = 0;
    float gyroX_  = 0, gyroY_  = 0, gyroZ_  = 0;
    float magneticX_ = 0, magneticY_ = 0, magneticZ_ = 0;
    float alt_  = 0;
    float pres_ = 0;
    float temp_ = 0;
};

#endif // SERIAL_SIM_H
