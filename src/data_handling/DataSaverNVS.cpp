// #include "../../lib/Avionics/include/data_handling/DataSaverNVS.h"

// /**
//  * Writing to nvs instead of associated 8mb flash using arduino preferences
//  * 
//  * Notes for working on the FeatherS3 - ESP32-S3 Development Board by Unexpected Maker:
//  * 
//  * nvs partition memory starts  @ 0x9000
//  * nvs memory has size of 0x5000
//  * nvs page size is 4096 bytes
//  *      ** i don't know if this nvs has partial page writing 
//  * 
//  */

// Preferences preferences;

// int DataSaverNVS::saveDataPoint(DataPoint dp, uint8_t name)
// {
//     uint8_t key = name;
//     float data = dp.data;

//     preferences.putFloat((char*)key, data);
// }

