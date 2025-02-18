#ifndef SERIAL_SIM_BMP3_H
#define SERIAL_SIM_BMP3_H

#include "Serial_Sim.h"

class SerialSimBMP390 {
public:
    SerialSimBMP390 (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization

    void setTemperatureOversampling(int oversampling) {}
    void setPressureOversampling(int oversampling) {}
    void setIIRFilterCoeff(int coeff) {}
    void setOutputDataRate(int rate) {}
    void setConversionDelay(int delay) {}
    void startConversion() {}
    bool updateConversion(){return true;}



    float getPressure(){
        float alt;
        float pressure;
        SimSerial->updateAltPres(alt, pressure);
        return pressure;
    }

    float getTemperature(){
        sensors_event_t temp;
        float temp_f;

        SimSerial->updateTemp(temp);
        temp_f = temp.temperature;

        return temp_f;
    }

private:
    SerialSim *SimSerial = SimSerialSingleton;
};

#endif // SERIAL_SIM_BMP3_H
