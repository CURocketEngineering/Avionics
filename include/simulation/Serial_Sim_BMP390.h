#ifndef SERIAL_SIM_BMP390_H
#define SERIAL_SIM_BMP390_H

#include "Serial_Sim.h"

#ifndef BMP3_NO_OVERSAMPLING
#define BMP3_NO_OVERSAMPLING        0x00
#endif

#ifndef BMP3_OVERSAMPLING_2X
#define BMP3_OVERSAMPLING_2X        0x01
#endif

#ifndef BMP3_OVERSAMPLING_4X
#define BMP3_OVERSAMPLING_4X        0x02
#endif

#ifndef BMP3_OVERSAMPLING_8X
#define BMP3_OVERSAMPLING_8X        0x03
#endif

#ifndef BMP3_OVERSAMPLING_16X
#define BMP3_OVERSAMPLING_16X       0x04
#endif

#ifndef BMP3_OVERSAMPLING_32X
#define BMP3_OVERSAMPLING_32X       0x05
#endif

#ifndef BMP3_IIR_FILTER_COEFF_3
#define BMP3_IIR_FILTER_COEFF_3     0x02
#endif

#ifndef BMP3_ODR_100_HZ
#define BMP3_ODR_100_HZ             0x01
#endif

#ifndef BMP3_ODR_50_HZ
#define BMP3_ODR_50_HZ              0x02
#endif

/**
 * @brief Mock BMP3XX sensor backed by SerialSim data.
 * @note When to use: compile flight code on a host without real hardware while
 *       exercising baro-dependent logic.
 */
class Adafruit_BMP3XX  {
public:
    Adafruit_BMP3XX  (){}

    bool begin_SPI(int cs) { return true; } // Mock successful initialization
    bool begin_I2C(int addr) { return true; } // Mock successful initialization
    bool begin_I2C() { return true; } // Mock successful initialization

    void setTemperatureOversampling(int oversampling) { temperatureOversampling_ = oversampling; }
    void setPressureOversampling(int oversampling) { pressureOversampling_ = oversampling; }
    void setIIRFilterCoeff(int coeff) { iirFilterCoeff_ = coeff; }
    void setOutputDataRate(int rate) { outputDataRate_ = rate; }
    void setConversionDelay(int delay) {}
    void startConversion() {}
    bool updateConversion(){return true;}

    float temperature = 0;
    float pressure = 0;

    bool performReading() {
        // Mock successful reading, update everthing from SerialSim
        sensors_event_t temp;
        float pressure;
        float altitude;
        SerialSim::getInstance().updateTemp(temp);
        SerialSim::getInstance().updatePres(pressure);
        SerialSim::getInstance().updateAlt(altitude);
        this->temperature = temp.temperature;
        this->pressure = pressure;
        return true;
    }


    float getAlt(){
        float alt;
        SerialSim::getInstance().updateAlt(alt);
        return alt;
    }

    float readAltitude(float seaLevelPressure = 1013.25) {
        // Mock altitude calculation
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


private:
    int temperatureOversampling_ = BMP3_NO_OVERSAMPLING;
    int pressureOversampling_ = BMP3_OVERSAMPLING_2X;
    int iirFilterCoeff_ = BMP3_IIR_FILTER_COEFF_3;
    int outputDataRate_ = BMP3_ODR_100_HZ;

};

#endif // SERIAL_SIM_BMP390_H
