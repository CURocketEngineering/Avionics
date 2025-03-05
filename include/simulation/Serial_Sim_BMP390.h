#ifndef SERIAL_SIM_BMP3_H
#define SERIAL_SIM_BMP3_H

#include "Serial_Sim.h"

class Adafruit_BMP3XX  {
public:
    Adafruit_BMP3XX  (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization

    void setTemperatureOversampling(int oversampling) {}
    void setPressureOversampling(int oversampling) {}
    void setIIRFilterCoeff(int coeff) {}
    void setOutputDataRate(int rate) {}
    void setConversionDelay(int delay) {}
    void startConversion() {}
    bool updateConversion(){return true;}


    float getAlt(){
        float alt;
        SerialSim::getInstance().updateAlt(alt);
        return alt;
    }

    float getPressure(){
        float pressure;
        SerialSim::getInstance().updatePres(pressure);
        return pressure;
    }

    float getTemperature(){
        sensors_event_t temp;
        float temp_f;

        SerialSim::getInstance().updateTemp(temp);
        temp_f = temp.temperature;
        return temp_f;
    }


};

#endif // SERIAL_SIM_BMP3_H
