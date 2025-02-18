#ifndef SERIAL_SIM_H
#define SERIAL_SIM_H

#include <string>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>

HardwareSerial SUART1(PB7, PB6);

SerialSim SimSerialSingleton;

class SerialSim {
public:
    SerialSim(){}

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
        idx = 0;
    }

    bool serialAvalible(void){
        return SUART1.available();
    }

    void updateTimeStamp(float &timestamp)
    {
        timestamp = parseNextFloat(incomingData);
    }

    void updateAcl(sensors_event_t &accel) {
        accel.acceleration.x = parseNextFloat(incomingData);
        accel.acceleration.y = parseNextFloat(incomingData);
        accel.acceleration.z = parseNextFloat(incomingData);

    }

    void updateGyro(sensors_event_t &gyro) {
        gyro.gyro.x = parseNextFloat(incomingData);
        gyro.gyro.y = parseNextFloat(incomingData);
        gyro.gyro.z = parseNextFloat(incomingData);
    }

    void updateMag(sensors_event_t &mag) {
        mag.magnetic.x = parseNextFloat(incomingData);
        mag.magnetic.y = parseNextFloat(incomingData);
        mag.magnetic.z = parseNextFloat(incomingData);
    }

    void updateAltPres(float &alt, float pres){
        alt = parseNextFloat(incomingData);
        pres = parseNextFloat(incomingData);
    }

    void updateTemp(sensors_event_t &temp){
        temp.temperature = incomingData.toFloat();
    }


private:
    String incomingData;
    int idx;

    float parseNextFloat(String& data) {
        float value = data.substring(0, data.indexOf(',')).toFloat();  // Extract value as float
        data = data.substring(data.indexOf(',') + 1);  // Update data by removing the parsed value
        return value;
    }

};

#endif // SERIAL_SIM_LSM6DSOX_H