 #ifndef SDLOGGER_H
 #define SDLOGGER_H
 #include <Arduino.h>
 #include <FS.h> // Add missing include path for FS library
 #include <SPI.h>
 #include <SD.h>
 #include <string>
 #include <vector>
 #include "telemetry.h"
 #include "ahrs.h"

 struct KFData {
    float acceleration;
    float velocity;
    float drift;
 };

 class SDLogger {
     private:
         const char *logFile;
         const char *telemFile;
         const char *gyroscopeFile = "/gyroscope.curedf";
         const char *accelerometerFile = "/accelerometer.curedf";
         const char *magnetometerFile = "/magnetometer.curedf";
         const char *barometerFile = "/barometer.curedf";
         TelemetryData lastData;
         void writeFile(fs::FS &fs, const char * path, const char * message);
         bool appendFile(fs::FS &fs, const char * path, const char * message);
         void renameFile(fs::FS &fs, const char * path1, const char * path2);
         void deleteFile(fs::FS &fs, const char * path);
         bool appendFileData(fs::FS &fs, const char * path, const SensorData &data);
     public:
         void readFile(fs::FS &fs, const char * path);
         SDLogger();
         SDLogger(std::string logFP, std::string telemFP);
         void setup();
         bool writeLog(std::string log);
         bool writeData(TelemetryData data, KFData kfData, double vAccel, double predApogee, std::string flightStatus);
         void close();
         bool isInitialized() { return SD.begin(10); }
 };
 #endif