import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data
df = pd.read_csv("data/MARTHA_IREC_2025_B2_transformed.csv")  # replace with actual path

# Plot accelx and altitude vs time
plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["accelx"], label="Accel X (m/s²)")
plt.plot(df["time"], df["altitude"], label="Altitude (m)", linestyle='--')

plt.xlabel("Time (ms)")
plt.ylabel("Value")
plt.title("Accel X and Altitude vs Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
