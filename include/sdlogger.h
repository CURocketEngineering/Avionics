#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>

#include <string>
#include <vector>

#include "telemetry.h"
#include "ahrs.h"

class SDLogger {
    private:
        const char *logFile;
        const char *telemFile;

        const char *gyroscopeFile = "/gyroscope.curedf";
        const char *accelerometerFile = "/accelerometer.curedf";
        const char *magnetometerFile = "/magnetometer.curedf";
        const char *barometerFile = "/barometer.curedf";

        TelemetryData lastData;

        void readFile(fs::FS &fs, const char * path);
        void writeFile(fs::FS &fs, const char * path, const char * message);
        bool appendFile(fs::FS &fs, const char * path, const char * message);
        void renameFile(fs::FS &fs, const char * path1, const char * path2);
        void deleteFile(fs::FS &fs, const char * path);

        bool appendFileData(fs::FS &fs, const char * path, const SensorData &data);
    public:
        SDLogger();
        SDLogger(std::string logFP, std::string telemFP);
        void setup();
        bool writeLog(std::string log);
        bool writeData(TelemetryData data, AHRSMap ahrsData, int flightStatus);
        void close();
};

#endif