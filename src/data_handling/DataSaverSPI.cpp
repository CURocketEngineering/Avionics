#include "data_handling/DataSaverSPI.h"
#include "ArduinoHAL.h"

DataSaverSPI::DataSaverSPI(uint16_t timestampInterval_ms, int mosi, int miso, int sck, int cs) 
: timestampInterval_ms(timestampInterval_ms), mosi(mosi), miso(miso), sck(sck), cs(cs)
{
    pinMode(mosi, OUTPUT);
    pinMode(miso, INPUT);
    pinMode(sck, OUTPUT);
    pinMode(cs, OUTPUT);
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    digitalWrite(cs, HIGH);

    lastTimestamp_ms = 0;
    lastDataPoint = DataPoint(0, 0.0);

}

int DataSaverSPI::saveDataPoint(DataPoint dp, uint8_t name) {
    // Write the timestamp if it is greater than the last timestamp by the timestampInterval_ms
    // If the timestamp is younger, we can't subtract to avoid underflow
    if (dp.timestamp_ms > lastTimestamp_ms && dp.timestamp_ms - lastTimestamp_ms >= timestampInterval_ms) {
        // Write the timestamp
        digitalWrite(cs, LOW);
        SPI.transfer(0x00);
        SPI.transfer(dp.timestamp_ms >> 8);
        SPI.transfer(dp.timestamp_ms & 0xFF);
        digitalWrite(cs, HIGH);

        lastTimestamp_ms = dp.timestamp_ms;
    }
    

    // Write the data
    digitalWrite(cs, LOW);
    SPI.transfer(0x01);

    // Send the name
    SPI.transfer(name);
    
    // Send the 4 bytes of the float
    unsigned char* dataBytes = (unsigned char*)&dp.data;
    for (int i = 0; i < 4; i++) {
        SPI.transfer(dataBytes[i]);
    }

    digitalWrite(cs, HIGH);

    lastDataPoint = dp;

    return 0;
}