#include "ArduinoHAL.h"          // for Serial
#include "data_handling/DataSaverBigSD.h"

static SdFat sd;                 // single global SdFat instance

using SdFile_t = File32;

/* ------------------------------------------------------------------ */
DataSaverBigSD::DataSaverBigSD(uint8_t csPin) : _csPin(csPin) {}

/* ----------  begin()  ---------- */
bool DataSaverBigSD::begin()
{
    Serial.print("Initialising SD (SdFat)… ");
    pinMode(_csPin, OUTPUT);
    if (!sd.begin(_csPin, SD_SCK_MHZ(25))) {
        Serial.println("failed!");
        return false;
    }
    Serial.println("done.");

    _filePath = nextFreeFilePath();          // e.g. “/stream‑3.csv”

    SdFile_t f = sd.open(_filePath.c_str(), O_WRITE | O_CREAT);
    if (!f) {
        Serial.println("Could not create data file!");
        return false;
    }
    // f.println("idx,name,value");          // optional header
    f.close();

    Serial.print("Logging to ");
    Serial.println(_filePath.c_str());

    _ready = true;
    return true;
}


/* ------------------------------------------------------------------ */
std::string DataSaverBigSD::nextFreeFilePath()
{
    char path[32];
    uint16_t n = 0;
    while (true) {
        snprintf(path, sizeof(path), "/stream-%u.csv", n);
        if (!sd.exists(path)) break;
        ++n;
    }
    return std::string(path);
}

/* ------------------------------------------------------------------ */
int DataSaverBigSD::saveDataPoint(DataPoint dp, uint8_t name)
{
    if (!_ready) return -1;                       // not initialised

    SdFile_t f = sd.open(_filePath.c_str(), O_WRITE | O_APPEND);
    if (!f)      return -2;                       // open error

    f.print(dp.timestamp_ms);   f.print(',');
    f.print(name);     f.print(',');
    f.println(dp.data);

    f.close();
    return 0;                                    // success
}
