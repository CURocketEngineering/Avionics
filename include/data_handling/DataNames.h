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
#define NUM_PACKETS_SENT 24

// Metadata
#define TIMESTAMP 14
#define STATE_CHANGE 15
#define CURRENT_STATE 23
#define FLIGHT_ID 16

// State Estimation
#define EST_APOGEE 17
#define EST_VERTICAL_VELOCITY 18
#define EST_ALTITUDE 19
#define TIME_TO_APOGEE 22
#define ROLL 25
#define PITCH 26
#define YAW 27

// Power
#define BATTERY_VOLTAGE 20

// Active Aero
#define FIN_DEPLOYMENT_AMOUNT 21

// Payload
#define PROBE_TEMPERATURE 28

// JEM
#define BMI_ACCEL_X 29
#define BMI_ACCEL_Y 30
#define BMI_ACCEL_Z 31
#define BMI_GYRO_X 32
#define BMI_GYRO_Y 33
#define BMI_GYRO_Z 34
#define MS5_PRESSURE 35
#define MS5_TEMP 36
#define MS5_ALTITUDE 37
#define KX_ACCEL_X 38
#define KX_ACCEL_Y 39
#define KX_ACCEL_Z 40

#endif
