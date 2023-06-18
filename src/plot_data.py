# Script for working out Kalman filter implementation

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# GET DATA
df = pd.read_csv('temp.csv')

# CONSTANTS
dt = 0.004 # [s]
freq = 1/dt # [Hz]

def get_time(df, dt, freq):
    return np.arange(0, len(df)/freq, dt)

time = get_time(df, dt, freq)
roll = df['roll'].to_numpy()
pitch = df['pitch'].to_numpy()
roll_acc = df['roll_acc'].to_numpy()
pitch_acc = df['pitch_acc'].to_numpy()

print(f"Roll mean: {np.mean(roll)}\n")
print(f"Pitch mean: {np.mean(pitch)}\n")

# Plot
fig1, ax1 = plt.subplots(2)
ax1[0].plot(time, roll)
ax1[0].plot(time, roll_acc)
ax1[1].plot(time, pitch)
ax1[1].plot(time, pitch_acc)
ax1[0].set_ylabel("Roll [deg]")
ax1[1].set_ylabel("Pitch [deg]")

plt.show()
