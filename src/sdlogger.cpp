#include "sdlogger.h"

SDLogger::SDLogger() {
    logFile = "/logs.txt";
    telemFile = "/telemetry.csv";

    // gyroscopeFile = "/gyroscope.curedf";
    // accelerometerFile = "/accelerometer.curedf";
    // magnetometerFile = "/magnetometer.curedf";
    // barometerFile = "/barometer.curedf";
}

SDLogger::SDLogger(std::string logFP = "/logs.txt", std::string telemFP = "/telemetry.csv") {
    logFile = logFP.c_str();
    telemFile = telemFP.c_str();
}

void SDLogger::setup() {
  Serial.print("Initializing SD card...");

  pinMode(10, OUTPUT);
  while(!SD.begin(10)) {
    Serial.println("Initialization failed!");
    sleep(5);
  }
  Serial.println("Initialization done.");

  Serial.println("Finished SD Setup!");
}

bool SDLogger::writeLog(std::string log) {
    const char* c_log = (log + '\n').c_str();
    appendFile(SD, logFile, c_log);
    return true;
}

bool SDLogger::writeData(TelemetryData data, AHRSMap ahrsData, int flightStatus) {
    if(data.timestamp == lastData.timestamp) {
        return false;
    } else {
        // if(data.sensorData["acceleration"].timestamp != lastData.sensorData["acceleration"].timestamp) {
        //     appendFileData(SD, accelerometerFile, data.sensorData["acceleration"]);
        // }
        // if(data.sensorData["gyro"].timestamp != lastData.sensorData["gyro"].timestamp) {
        //     appendFileData(SD, gyroscopeFile, data.sensorData["gyro"]);
        // }
        // if(data.sensorData["magnetometer"].timestamp != lastData.sensorData["magnetometer"].timestamp) {
        //     appendFileData(SD, magnetometerFile, data.sensorData["magnetometer"]);
        // }
        // if(data.sensorData["pressure"].timestamp != lastData.sensorData["pressure"].timestamp) {
        //     appendFileData(SD, barometerFile, data.sensorData["temperature"]);
        //     appendFileData(SD, barometerFile, data.sensorData["pressure"]);
        //     appendFileData(SD, barometerFile, data.sensorData["altitude"]);
        // }

        appendFile(SD, telemFile, (std::to_string(data.timestamp) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["acceleration"].acceleration.x) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["acceleration"].acceleration.y) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["acceleration"].acceleration.z) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["gyro"].gyro.x) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["gyro"].gyro.y) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["gyro"].gyro.z) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["magnetometer"].magnetic.x) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["magnetometer"].magnetic.y) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["magnetometer"].magnetic.z) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["temperature"].temperature) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(data.sensorData["pressure"].pressure) + ",").c_str());

        appendFile(SD, telemFile, (std::to_string(ahrsData["rx"]) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(ahrsData["ry"]) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(ahrsData["rz"]) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(ahrsData["gx"]) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(ahrsData["gy"]) + ",").c_str());
        appendFile(SD, telemFile, (std::to_string(ahrsData["gz"]) + "\n").c_str()); //is this why it was double spaced

        appendFile(SD, telemFile, (std::to_string(flightStatus) + ",").c_str());

        appendFile(SD, telemFile, (std::to_string(data.sensorData["altitude"].altitude) + "\n").c_str());

        return true;
    }
}

void SDLogger::readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void SDLogger::writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

bool SDLogger::appendFile(fs::FS &fs, const char * path, const char * message){
    // Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        // Serial.println("Failed to open file for appending");
        return false;
    }
    if(file.print(message)){
        // Serial.println("Message appended");
        return true;
    } else {
        // Serial.println("Append failed");
        return false;
    }
    file.close();
}

void SDLogger::renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void SDLogger::deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

bool SDLogger::appendFileData(fs::FS &fs, const char *path, const SensorData &data) {
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        // Serial.println("Failed to open file for appending");
        return false;
    }

    // Convert SensorData to bytes
    const byte* bytes = reinterpret_cast<const byte*>(&data);
    size_t dataSize = sizeof(data);

    // Append the bytes to the file
    bool successful = file.write(bytes, dataSize) == dataSize;
    file.close();
    
    return successful;
}


// void createDir(fs::FS &fs, const char * path){
//     Serial.printf("Creating Dir: %s\n", path);
//     if(fs.mkdir(path)){
//         Serial.println("Dir created");
//     } else {
//         Serial.println("mkdir failed");
//     }
// }

// void removeDir(fs::FS &fs, const char * path){
//     Serial.printf("Removing Dir: %s\n", path);
//     if(fs.rmdir(path)){
//         Serial.println("Dir removed");
//     } else {
//         Serial.println("rmdir failed");
//     }
// }
