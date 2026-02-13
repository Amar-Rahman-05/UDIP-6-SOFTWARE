#UDIP-6 GROUND ANALYSIS

"""REQUIRED GRAPHS
    Acceleration vs Flight Duration
    [x y z] vs [time] 
    High Range Acceleration vs Flight Duration
    [x y z] vs [time]
    Gyroscope vs Flight Duration 
    [x y z] vs time
    Temperature Vs Flight Duration
    [x] vs time
    Magnetic Flux Density (Magnetometer) Vs Flight Duration
    [x y z] vs time
    Photosensors vs Flight Duration 
    [x] vs time

    


    Exemplar IV Curve At Apogee
    
    Electron Density Vs Altitude
    Electron Temperature vs Altitude
    Medium Sweep Summary
"""


import csv
import os
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt

plt.rcParams.update({
    "figure.figsize": (6.5, 4),
    "figure.dpi": 300,
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "axes.grid": True,
    "grid.linestyle": "--",
    "grid.linewidth": 0.5,
    "lines.linewidth": 1.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

sensor_csv = "sensor_packets_2026-02-06_09-39-18.csv" 
sweep_csv = "sweep_packets_2026-02-04_21-36-27.csv"
folder_name_sensor =  "C:\\Users\\Indoo\\UDIP-6-SOFTWARE\\UDIP-6-GROUND\\CSV-DATA\\SENSOR"
folder_name_sweep =  "C:\\Users\\Indoo\\UDIP-6-SOFTWARE\\UDIP-6-GROUND\\CSV-DATA\\SWEEP"
file_path_sensor = os.path.join(folder_name_sensor, sensor_csv)
file_path_sweep = os.path.join(folder_name_sweep, sweep_csv)

time = []
accel_M = {"x": [], "y": [], "z": []}
accel_H = []
gyro    = {"x": [], "y": [], "z": []}
mag     = {"x": [], "y": [], "z": []}
temp    = []

with open(file_path_sensor, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t = float(row["tFinal"]) - float(row["tInitial"])
        time.append(t)

        accel_M["x"].append(float(row["accel_M_x"]))
        accel_M["y"].append(float(row["accel_M_y"]))
        accel_M["z"].append(float(row["accel_M_z"]))

        accel_H.append(float(row["accel_H"]))

        gyro["x"].append(float(row["gyro_M_x"]))
        gyro["y"].append(float(row["gyro_M_y"]))
        gyro["z"].append(float(row["gyro_M_z"]))

        mag["x"].append(float(row["mag_M_x"]))
        mag["y"].append(float(row["mag_M_y"]))
        mag["z"].append(float(row["mag_M_z"]))

        #temp.append(float(row["temp"]))


# Acceleration vs Flight Duration

plt.figure()
plt.plot(time, accel_M["x"], label="X-axis")
plt.plot(time, accel_M["y"], label="Y-axis")
plt.plot(time, accel_M["z"], label="Z-axis")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s²)")
plt.title("UDIP-6 Test Data: Medium-Range Acceleration")
plt.legend(frameon=False)
plt.tight_layout()
plt.show()


# High Range Acceleration

plt.figure()
plt.plot(time, accel_H, label="High-Range Accelerometer")

plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s²)")
plt.title("UDIP-6 Test Data: High-Range Acceleration")
plt.legend(frameon=False)
plt.tight_layout()
plt.show()


# Gyroscope vs Flight Duration

plt.figure()
plt.plot(time, gyro["x"], label="X-axis")
plt.plot(time, gyro["y"], label="Y-axis")
plt.plot(time, gyro["z"], label="Z-axis")

plt.xlabel("Time (s)")
plt.ylabel("Angular Rate (rad/s)")
plt.title("UDIP-6 Test Data: Gyroscope Measurements")
plt.legend(frameon=False)
plt.tight_layout()
plt.show()

# Temperature vs Flight Duration
'''plt.figure()
plt.plot(time, temp)
plt.xlabel("Flight Duration")
plt.ylabel("Temperature")
plt.title("Temperature vs Flight Duration")
plt.grid()
plt.show()'''

# Magnetometer vs Flight Duration

plt.figure()
plt.plot(time, mag["x"], label="X-axis")
plt.plot(time, mag["y"], label="Y-axis")
plt.plot(time, mag["z"], label="Z-axis")
plt.xlabel("Time (s)")
plt.ylabel("Magnetic Flux Density (µT)")
plt.title("UDIP-6 Test Data: Magnetometer Measurements")
plt.legend(frameon=False)
plt.tight_layout()
plt.show()

fig, ax1 = plt.subplots()

# Acceleration
ax1.plot(time, accel_M["x"], label="Accel X")
ax1.plot(time, accel_M["y"], label="Accel Y")
ax1.plot(time, accel_M["z"], label="Accel Z")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Acceleration (m/s²)")

# Gyroscope (offset left)
ax2 = ax1.twinx()
ax2.spines["right"].set_visible(False)
ax2.spines["left"].set_position(("outward", 50))
ax2.yaxis.set_label_position("left")
ax2.yaxis.tick_left()
ax2.plot(time, gyro["x"], linestyle="--", label="Gyro X")
ax2.plot(time, gyro["y"], linestyle="--", label="Gyro Y")
ax2.plot(time, gyro["z"], linestyle="--", label="Gyro Z")
ax2.set_ylabel("Angular Rate (rad/s)")

# Magnetometer (further offset)
ax3 = ax1.twinx()
ax3.spines["right"].set_visible(False)
ax3.spines["left"].set_position(("outward", 100))
ax3.yaxis.set_label_position("left")
ax3.yaxis.tick_left()
ax3.plot(time, mag["x"], linestyle=":", label="Mag X")
ax3.plot(time, mag["y"], linestyle=":", label="Mag Y")
ax3.plot(time, mag["z"], linestyle=":", label="Mag Z")
ax3.set_ylabel("Magnetic Flux Density (µT)")

lines = ax1.get_lines() + ax2.get_lines() + ax3.get_lines()
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, frameon=False, loc="upper right")

plt.title("UDIP-6 Test Data: Combined Sensor Measurements")
plt.tight_layout()
plt.show()

plt.savefig("UDIP6_Accel_Medium.png", dpi=300)
plt.savefig("UDIP6_Accel_Medium.pdf")