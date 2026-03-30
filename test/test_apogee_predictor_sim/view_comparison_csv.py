import matplotlib.pyplot as plt
import pandas as pd 
import numpy as np

# Open 
# Read the CSV file
df = pd.read_csv('apogee_prediction_all_methods.csv')


# Columns
# timestamp,true_alt,true_vertical_velocity,est_alt,est_vertical_velocity,true_acl,est_acl,cd,est_apogee


# Plot everything except for the vertical velocity stuff 

plt.figure(figsize=(10, 6))
plt.plot(df['timestamp'], df['true_alt'], label='True Altitude', color='blue')
plt.plot(df['timestamp'], df['est_alt'], label='Estimated Altitude', color='orange')

#analytic_apogee,simulate_apogee,quad_apogee,poly_apogee,default_apogee
plt.plot(df['timestamp'], df['analytic_apogee'], label='analytic Apogee', color='green')
plt.plot(df['timestamp'], df['simulate_apogee'], label='simulate Apogee', color='black')
plt.plot(df['timestamp'], df['quad_apogee'], label='quad Apogee', color='red')
plt.plot(df['timestamp'], df['poly_apogee'], label='poly Apogee', color='purple')
plt.plot(df['timestamp'], df['default_apogee'], label='default Apogee', color='orange')

# # Plot a vertical line when vertical velocty first crosses 0
# crossings = np.where(np.diff(np.sign(df['true_vertical_velocity'])))[0]
# for crossing in crossings:
#     plt.axvline(x=df['timestamp'][crossing], color='red', linestyle='--', label='Vertical Velocity Crossing')

# plot acceleration here
plt.plot(df['timestamp'], df['true_acl'], label='True Acceleration', color='purple')
plt.plot(df['timestamp'], df['est_acl'], label='Estimated Acceleration', color='brown')

# Plot cd 
plt.plot(df['timestamp'], df['cd'] * 1e3, label='Drag Coefficient', color='pink')





plt.title('Altitude and Apogee Estimation')
plt.xlabel('Timestamp')
plt.ylabel('Altitude (m)')
plt.legend()
plt.grid()
plt.show()

# Plot the vertical velocity
plt.figure(figsize=(10, 6))
plt.plot(df['timestamp'], df['true_vertical_velocity'], label='True Vertical Velocity', color='blue')
plt.plot(df['timestamp'], df['est_vertical_velocity'], label='Estimated Vertical Velocity', color='orange')
plt.title('Vertical Velocity Estimation')
plt.xlabel('Timestamp')
plt.ylabel('Vertical Velocity (m/s)')
plt.legend()
plt.grid()
plt.show()