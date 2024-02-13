#ifndef AHRS_H
#define AHRS_H

#include <Adafruit_AHRS_FusionInterface.h>
#include <Adafruit_AHRS_NXPFusion.h>

//proper units and directions + coherence

typedef std::map<std::string, float> AHRSMap;

class AHRS : public Adafruit_AHRS_FusionInterface
{
public:
    AHRS(){

    };
    //All that we'll ultimately end up using will probably be gvy
    Adafruit_NXPSensorFusion interface; //debating also switching to one of the alternative AHRSs for simplicity sake/speed sake
    //I just noticed that magnetometer and accelerometer XYZ aren't the same are we using raw data or are we accounting for them in telem?

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        interface.update(gx, gy, gz, ax, ay, az, my*-1, mx, mz); //align physically with the sensors //will have to change with a new board
    }

    void begin(float sampleFrequency = 115200.0F)
    {
        interface.begin(sampleFrequency);
        //interface.setRotationVector(0, 0, 0);//why is the arduino/pio library different per system 
        //this will cause an error for anyone compiling not on my laptop, how can i get this without modifying the library but keeping the interface object???
        //i might not be able to, so the object sync might have to be moved back when i can do that next
        //other solution is duplicating library functionality to the project files and editing them there
    };

    void getRotationVector(float *x, float *y, float *z)
    {
        //interface.getRotationVector(x, y, z);
    }

    void setRotationVector(float x, float y, float z)
    {
        //interface.setRotationVector(x, y, z);
    }

    float getRoll() { return interface.getRoll(); }
    float getPitch() { return interface.getPitch(); }
    float getYaw() { return interface.getYaw(); }

    void getQuaternion(float *w, float *x, float *y, float *z)
    {
        interface.getQuaternion(w, x, y, z);
    }

    void setQuaternion(float w, float x, float y, float z)
    {
        interface.setQuaternion(w, x, y, z);
    }

    void getLinearAcceleration(float *x, float *y, float *z) const
    {
        interface.getLinearAcceleration(x, y, z);
    } // in g
    void getGravityVector(float *x, float *y, float *z)
    {
        interface.getGravityVector(x, y, z);
    } // in g

    typedef struct
    {
        float q0; // w
        float q1; // x
        float q2; // y
        float q3; // z
    } Quaternion_t;
};

#endif