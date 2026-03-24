"""
Converts a flight CSV to the standardized format required by Avionics tests.
"""

import pandas as pd
import os
import sys 

# Load the first arg as the input file
input_file = sys.argv[1]
input_data = pd.read_csv(input_file)

# Find a standin for each unit
"""

### Units
- `time` is in miliseconds
- `accelx`, `accely`, `accelz` are in m/s^2
- `gyrox`, `gyroy`, `gyroz` are in rad/s
- `magx`, `magy`, `magz` are in uT
- `altitude` is in meters
- `pressure` is in hPa
- `temp` is in degrees Celsius

"""

column_mapping = {
}

def get_first_matching_column(df, column_names):
    """
    Get the first matching column from the dataframe
    """
    for column_name in column_names:
        if column_name in df.columns:
            return column_name
        if column_name.upper() in df.columns:
            return column_name.upper()
        if column_name.lower() in df.columns:
            return column_name.lower()
        if column_name.capitalize() in df.columns:
            return column_name.capitalize()
    return None

# Get time 
column_mapping['time'] = get_first_matching_column(input_data, ['time', 'timestamp', 'Time', 'Time (ms)', 'Time (s)'])

# Get accel
column_mapping['accelx'] = get_first_matching_column(input_data, ['accelx', 'accel_x', 'accel_x (m/s^2)', 'acceleration_x', 'accelerometer_x'])
column_mapping['accely'] = get_first_matching_column(input_data, ['accely', 'accel_y', 'accel_y (m/s^2)', 'acceleration_y', 'accelerometer_y'])
column_mapping['accelz'] = get_first_matching_column(input_data, ['accelz', 'accel_z', 'accel_z (m/s^2)', 'acceleration_z', 'accelerometer_z'])

# Get gyro
column_mapping['gyrox'] =  get_first_matching_column(input_data, ['gyrox', 'gyro_x', 'gyro_x (rad/s)', 'gyroscope_x'])
column_mapping['gyroy'] = get_first_matching_column(input_data, ['gyroy', 'gyro_y', 'gyro_y (rad/s)', 'gyroscope_y'])
column_mapping['gyroz'] = get_first_matching_column(input_data, ['gyroz', 'gyro_z', 'gyro_z (rad/s)', 'gyroscope_z'])

# Get mag
column_mapping['magx'] = get_first_matching_column(input_data, ['magx', 'mag_x', 'mag_x (uT)', 'magnetometer_x'])
column_mapping['magy'] = get_first_matching_column(input_data, ['magy', 'mag_y', 'mag_y (uT)', 'magnetometer_y'])
column_mapping['magz'] = get_first_matching_column(input_data, ['magz', 'mag_z', 'mag_z (uT)', 'magnetometer_z'])

# Get altitude
column_mapping['altitude'] = get_first_matching_column(input_data, ['altitude', 'altitude (m)', 'altitude (meters)', 'elevation', 'height'])

# Get pressure
column_mapping['pressure'] = get_first_matching_column(input_data, ['pressure', 'pressure (hPa)', 'pressure (hPa)', 'barometric_pressure', 'air_pressure'])

# Get temperature
column_mapping['temp'] = get_first_matching_column(input_data, ['temp', 'temperature', 'temperature (C)', 'temperature (degrees Celsius)', 'temp (C)', 'temp (degrees Celsius)'])

print(column_mapping)

# Create a new dataframe with the columns in the order required by Avionics tests
output_data = pd.DataFrame(columns=[
    'time',
    'accelx',
    'accely',
    'accelz',
    'gyrox',
    'gyroy',
    'gyroz',
    'magx',
    'magy',
    'magz',
    'altitude',
    'pressure',
    'temp'
])

# Add the data to the new dataframe
for column_name, new_column_name in column_mapping.items():
    if new_column_name is not None:
        output_data[column_name] = input_data[new_column_name]
    else:
        output_data[column_name] = None


# Interpolate data to fill in missing values
output_data = output_data.interpolate(method='linear', limit_direction='both')

# Sort the data my timestamp
output_data = output_data.sort_values(by='time').reset_index(drop=True)


# Find where peak altitude is an only keep the 30 seconds before and after
peak_altitude_index = output_data['altitude'].idxmax()
peak_altitude_time = output_data['time'][peak_altitude_index]
# Filter the data to only keep the 30 seconds before and after the peak altitude
start_time = peak_altitude_time - 30000  # 30 seconds before
end_time = peak_altitude_time + 30000    # 30 seconds after
output_data = output_data[(output_data['time'] >= start_time) & (output_data['time'] <= end_time)].reset_index(drop=True)

# Plot alt and aclz on the same plot to make sure it all looks good
import matplotlib.pyplot as plt
plt.plot(output_data['time'], output_data['altitude'], label='altitude')
plt.plot(output_data['time'], output_data['accelz'], label='accelz')
plt.xlabel('time (ms)')
plt.ylabel('altitude (m) / accelz (m/s^2)')
plt.title('altitude and accelz')
plt.legend()
plt.show()

# Save the new dataframe to a csv file
output_file = os.path.splitext(input_file)[0] + '_transformed.csv'

output_data.to_csv(output_file, index=False)

