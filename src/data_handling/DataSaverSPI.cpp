#include "data_handling/DataSaverSPI.h"
#include "FlashDriver.h"
#include "ArduinoHAL.h"



// --------------------------------------------------------------------------
// Data Format Layout in Flash
// --------------------------------------------------------------------------
/**
 * We’ll use a simple marker-based layout:
 *   [ 1 byte  : flags ]
 *   [ optional 4 bytes of timestamp (if flags & 0x01 != 0) ]
 *   [ 1 byte  : name ]
 *   [ 4 bytes : float value (DataPoint.value) ]
 *
 *  flags bit 0 (0x01) indicates if timestamp is included.
 */

// A helper constant so we can identify the "timestamp included" bit
static const uint8_t FLAG_TIMESTAMP_INCLUDED = 0x01;

// --------------------------------------------------------------------------
// Constructor
// --------------------------------------------------------------------------
DataSaverSPI::DataSaverSPI(uint16_t timestampInterval_ms, int mosi, int miso, int sck, int cs)
    : timestampInterval_ms(timestampInterval_ms),
      lastTimestamp_ms(0),
      lastDataPoint({0, 0.0f}),
      mosiPin(mosi),
      misoPin(miso),
      sckPin(sck),
      csPin(cs),
      flash(mosi, miso, sck, cs),
      nextWriteAddress(0)
{}

bool DataSaverSPI::begin(){
    // pinMode(csPin, OUTPUT);
    bool success = true;
    
    // Initialize the flash
    success = flash.initFlash() == FLASH_SUCCESS;
    if (!success) {
        Serial.println("Failed to initialize flash");
    }

    // Add any other initialization code here with each step checking for success

    return success;
}

// --------------------------------------------------------------------------
// Save a single data point
// --------------------------------------------------------------------------
int DataSaverSPI::saveDataPoint(DataPoint dp, uint8_t name)
{
    // Decide if we need to store the timestamp
    bool shouldWriteTimestamp = false;
    if ((dp.timestamp_ms - lastTimestamp_ms) >= timestampInterval_ms) {
        shouldWriteTimestamp = true;
        lastTimestamp_ms = dp.timestamp_ms;  // Update stored "lastTimestamp_ms"
    }

    // Build up the data to write
    // 1. flags
    uint8_t flags = 0;
    if (shouldWriteTimestamp) {
        flags |= FLAG_TIMESTAMP_INCLUDED;
    }

    // Write out flags
    if (!writeToFlash(&flags, sizeof(flags))) {
        return -1; // error
    }

    // 2. optionally write timestamp
    if (shouldWriteTimestamp) {
        if (!writeToFlash(&dp.timestamp_ms, sizeof(dp.timestamp_ms))) {
            return -2; // error
        }
    }

    // 3. write name
    if (!writeToFlash(&name, sizeof(name))) {
        return -3; // error
    }

    // 4. write the data portion of DataPoint (minus the timestamp, if you want to avoid duplication)
    if (!writeToFlash(&dp.data, sizeof(dp.data))) {
        return -4; // error
    }

    // Save the last data point (for quick retrieval)
    lastDataPoint = dp;

    return 0; // success
}

// --------------------------------------------------------------------------
// Dump data from flash
// --------------------------------------------------------------------------
void DataSaverSPI::dumpData()
{
    Serial.println("==== Dumping SPI Flash Data ====");

    uint32_t readAddress = 0;
    while (true) {
        // 1. Read flags
        uint8_t flags;
        if (!readFromFlash(readAddress, &flags, sizeof(flags))) {
            // Either we reached the end of written data or a read error
            break;
        }

        // If flags byte is 0xFF (erased flash often reads as 0xFF),
        // we can assume there's no more data
        if (flags == 0xFF) {
            break;
        }

        bool hasTimestamp = (flags & FLAG_TIMESTAMP_INCLUDED) != 0;
        uint32_t timestamp = 0;
        if (hasTimestamp) {
            // 2. read timestamp
            if (!readFromFlash(readAddress, &timestamp, sizeof(timestamp))) {
                break;
            }
        }

        // 3. read name
        uint8_t name;
        if (!readFromFlash(readAddress, &name, sizeof(name))) {
            break;
        }

        // 4. read value
        float value;
        if (!readFromFlash(readAddress, &value, sizeof(value))) {
            break;
        }

        // Now do something with the data
        if (hasTimestamp) {
            Serial.print("[T=");
            Serial.print(timestamp);
            Serial.print("] ");
        } else {
            Serial.print("[no TS] ");
        }

        Serial.print("Name: ");
        Serial.print(name);
        Serial.print(", Value: ");
        Serial.println(value);
    }

    Serial.println("==== End of Data Dump ====");
}

// --------------------------------------------------------------------------
// Erase entire flash
// --------------------------------------------------------------------------
void DataSaverSPI::eraseAllData()
{
    flash.eraseFlash();
    nextWriteAddress = 0;   // reset the pointer
    lastTimestamp_ms = 0;   // reset our last timestamp
    // Optionally reset lastDataPoint as well
    lastDataPoint = {0, 0.0f};
    Serial.println("Flash erased. nextWriteAddress reset to 0.");
}

// --------------------------------------------------------------------------
// Helper: Write data to flash at `nextWriteAddress`, then advance
// --------------------------------------------------------------------------
bool DataSaverSPI::writeToFlash(const void* data, size_t length)
{
    if (!data || length == 0) {
        return false;
    }

    FlashStatus status = flash.writeFlash(nextWriteAddress,
                                          static_cast<const uint8_t*>(data),
                                          length);
    if (status == FLASH_SUCCESS) {
        nextWriteAddress += length;
        return true;
    } else {
        return false;
    }
}

// --------------------------------------------------------------------------
// Helper: Read data from flash at `readAddress` into `buffer`
//          Advances readAddress only if successful
// --------------------------------------------------------------------------
bool DataSaverSPI::readFromFlash(uint32_t& readAddress, void* buffer, size_t length)
{
    if (!buffer || length == 0) {
        return false;
    }

    uint8_t* out = static_cast<uint8_t*>(buffer);
    FlashStatus status = flash.readFlash(readAddress, out, length);

    if (status == FLASH_SUCCESS) {
        readAddress += length;
        return true;
    } else {
        return false;
    }
}