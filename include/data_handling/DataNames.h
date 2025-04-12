#ifndef DATA_NAMES_H
#define DATA_NAMES_H
// Defines the uint8_t value for each data stream

// Sensor data
#define ACCELEROMETER_X 0
#define ACCELEROMETER_Y 1
#define ACCELEROMETER_Z 2
#define GYROSCOPE_X 3
#define GYROSCOPE_Y 4
#define GYROSCOPE_Z 5
#define TEMPERATURE 6
#define PRESSURE 7
#define ALTITUDE 8
#define MAGNETOMETER_X 9
#define MAGNETOMETER_Y 10
#define MAGNETOMETER_Z 11

// Metrics
#define MEDIAN_ACCELERATION_SQUARED 12
#define AVERAGE_CYCLE_RATE 13

// Metadata
#define TIMESTAMP 14
#define STATE_CHANGE 15
#define FLIGHT_ID 16

// State Estimation
#define EST_APOGEE 17
#define EST_VERTICAL_VELOCITY 18
#define EST_ALTITUDE 19

// Power
#define BATTERY_VOLTAGE 20

// Active Aero
#define FIN_DEPLOYMENT_AMOUNT 21

#endif