import pandas as pd
import matplotlib.pyplot as plt
import os

# Customize this if needed
RESULT_CSV = "vve_results_MARTHA_IREC_2025_B2_transformed.csv"

# Check file exists
if not os.path.exists(RESULT_CSV):
    raise FileNotFoundError(f"{RESULT_CSV} not found.")

# Load CSV
df = pd.read_csv(RESULT_CSV)

# Convert ms → s for readability
df["t_s"] = df["t_ms"] * 0.001

# Create subplots
fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Vertical Velocity Estimator Results", fontsize=16)

# Altitude comparison
axs[0].plot(df["t_s"], df["raw_alt"], label="Raw Altitude", linestyle="--")
axs[0].plot(df["t_s"], df["est_alt"], label="Estimated Altitude")
axs[0].set_ylabel("Altitude (m)")
axs[0].legend()
axs[0].grid(True)

# Velocity comparison
axs[1].plot(df["t_s"], df["fdiff_vel"], label="Finite-Difference Velocity", linestyle="--")
axs[1].plot(df["t_s"], df["est_vel"], label="Estimated Velocity")
axs[1].set_ylabel("Velocity (m/s)")
axs[1].legend()
axs[1].grid(True)

# Acceleration
axs[2].plot(df["t_s"], df["accZ"], label="Inertial Vertical Accel")
axs[2].set_ylabel("Accel (m/s²)")
axs[2].legend()
axs[2].grid(True)

# Error
axs[3].plot(df["t_s"], df["err_vel"], label="Velocity Error")
axs[3].plot(df["t_s"], df["err_alt"], label="Altitude Error")
axs[3].set_ylabel("Error")
axs[3].set_xlabel("Time (s)")
axs[3].legend()
axs[3].grid(True)

plt.tight_layout()
plt.show()
