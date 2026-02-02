// Originally written by Ethan Anderson -- 2/17/2024
// DataHandler.h

#ifndef SensorDataHandler_H
#define SensorDataHandler_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

/**
 * @brief Buffers sensor samples and forwards them to an IDataSaver at a
 *        controlled rate.
 * @note When to use: wrap any sensor stream that needs to be saved at a 
 *      limited frequency to avoid overwhelming the saver or consuming excess
 *      storage.
 */
class SensorDataHandler {
public:
    /**
     * @brief Construct a handler for a specific sensor.
     * @param name      8-bit identifier for the sensor/channel.
     * @param dataSaver Destination used when saving samples.
     * @note When to use: create one per sensor source and share a saver across
     *       multiple handlers as needed.
     */
    SensorDataHandler(uint8_t name, IDataSaver* dataSaver);
    
    /**
     * @brief Ingest a data point and persist it if the save interval elapsed.
     * @param data Data point to record.
     * @return Status from the underlying saver.
     * @note When to use: on every new reading; rate limiting is handled
     *       internally.
     */
    int addData(DataPoint data);

    /**
     * @brief Set the minimum gap between persisted samples.
     * @param interval_ms Minimum milliseconds between writes.
     * @note When to use: tune logging cadence to balance fidelity vs. storage.
     */
    void restrictSaveSpeed(uint16_t interval_ms);

    uint8_t getName() const {return name;}

    DataPoint getLastDataPointSaved() const {
        return lastDataPointSaved;
    }

 
protected:
    IDataSaver* dataSaver;
private:
    uint8_t name;
    uint16_t saveInterval_ms; // The minimum time between each data point that is saved to the SD card
    uint32_t lastSaveTime_ms; // The last time a data point was saved to the SD card

    DataPoint lastDataPointSaved;
};

#endif // DATAHANDLER_H