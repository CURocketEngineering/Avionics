// #include "flightstatus.h"

// FlightStatus::FlightStatus(SensorDataHandler * xac, SensorDataHandler * yac, SensorDataHandler * zac) {
//     // Why 128 to 0 when passing in?
//     flightStage = ARMED;
//     validatedSDHs = 0;
//     this->xac = xac;
//     this->yac = yac;
//     this->zac = zac;
// }

// void FlightStatus::setupSDHs(){

//     // Check that all of their intervals are the same
//     if(xac->getInterval_ms() != yac->getInterval_ms() || xac->getInterval_ms() != zac->getInterval_ms()){
//         Serial.println("Acl Intervals are not the same");
//         return;
//     }

//     // Check that all of their sizes are the same
//     if(xac->getMaxSize() != yac->getMaxSize() || xac->getMaxSize() != zac->getMaxSize()){
//         Serial.println("FS_ERROR - Acl Sizes are not the same");
//         return;
//     }

//     // Check that the total interval is between 500 and 2000ms
//     int16_t total_interval = xac->getInterval_ms() * xac->getMaxSize();
//     if(total_interval < 500 || total_interval > 2000){
//         Serial.println("FS_ERROR - Acl Size is not between 500 and 2000ms");
//         Serial.print("Interval (");
//         Serial.print(xac->getInterval_ms());
//         Serial.print(") * Size (");
//         Serial.print(xac->getMaxSize());
//         Serial.print(") = ");
//         Serial.println(total_interval);
//         return;
//     }

//     // Check that interval is between 50 and 500ms
//     if(xac->getInterval_ms() < 50 || xac->getInterval_ms() > 500){
//         Serial.println("FS_ERROR - Acl Interval is not between 50 and 500ms");
//         return;
//     }

//     validatedSDHs = 1;

// }

// // Using SensorDataHandlers to check for launch
// bool FlightStatus::checkLaunchSDH(HardwareSerial * SD_serial){
    

//     // If the acl arrays are not full, then skip for now
//     if(!xac->isFull() || !yac->isFull() || !zac->isFull()){
//         Serial.println("Acl arrays are not full, skipping launch check");
//         return false;
//     }
    
    
//     // Check the current actual data rate, between a few values from xac
//     // If the data rate is too slow, then the sensor is not working
    
//     // Get the latest 2 timestamps
//     uint32_t xac_0_time = xac->getHistoricalDataCount(0).timestamp_ms;
//     uint32_t xac_1_time = xac->getHistoricalDataCount(1).timestamp_ms; // Isn't changing

//     // Check that the interval is between 50 and 500ms
//     int32_t xac_interval = xac_0_time - xac_1_time;
//     if(xac_interval < 50 || xac_interval > 500){
//         Serial.print("Real Acl Interval is not between 50 and 500ms, got: ");
//         Serial.println(xac_interval);
//         Serial.print("Latest timestamp (ghdc0): ");
//         Serial.println(xac_0_time);
//         Serial.print("Latest timestamp (ghdc1): ");
//         Serial.println(xac_1_time);
//         // Latest timestamp from .getLatestData()
//         Serial.print("Latest timestamp:         ");
//         Serial.println(xac->getLatestData().timestamp_ms);

//         return false;
//     }

//     // Taking the median data from each component
//     float xac_median = xac->getTemporalArrayMedian().data;
//     float yac_median = yac->getTemporalArrayMedian().data;
//     float zac_median = zac->getTemporalArrayMedian().data;

//     // Calculate the magnitude of the acceleration without sqrt because sqrt is expensive
//     float accel_mag_squared = xac_median * xac_median + yac_median * yac_median + zac_median * zac_median;

//     // Compare to threshold
//     if(accel_mag_squared > 50*50){
//         return true;
//     }

//     return false;    
// }

// void FlightStatus::update(HardwareSerial * SD_serial) {
//     if(validatedSDHs == 0){
//         Serial.println("Sensor Data Handlers not setup/validated successfully (run setupSDHs), flight status not being updated");
//         return;
//     }

//     if(flightStage == ARMED && checkLaunchSDH(SD_serial)) {
//         Serial.println("Launch detected");
//         flightStage = ASCENT;
//     }
// }

// Stage FlightStatus::getStage() {
//     return flightStage;
// }