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
plt.plot(time, accel_M["x"], label="Accel X")
plt.plot(time, accel_M["y"], label="Accel Y")
plt.plot(time, accel_M["z"], label="Accel Z")
plt.xlabel("Flight Duration")
plt.ylabel("Acceleration (Medium Range)")
plt.title("Acceleration vs Flight Duration")
plt.legend()
plt.grid()


# High Range Acceleration

plt.figure()
plt.plot(time, accel_H, label="High Range Accel")
plt.xlabel("Flight Duration")
plt.ylabel("Acceleration (High Range)")
plt.title("High Range Acceleration vs Flight Duration")
plt.legend()
plt.grid()


# Gyroscope vs Flight Duration

plt.figure()
plt.plot(time, gyro["x"], label="Gyro X")
plt.plot(time, gyro["y"], label="Gyro Y")
plt.plot(time, gyro["z"], label="Gyro Z")
plt.xlabel("Flight Duration")
plt.ylabel("Angular Rate")
plt.title("Gyroscope vs Flight Duration")
plt.legend()
plt.grid()


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
plt.plot(time, mag["x"], label="Mag X")
plt.plot(time, mag["y"], label="Mag Y")
plt.plot(time, mag["z"], label="Mag Z")
plt.xlabel("Flight Duration")
plt.ylabel("Magnetic Flux Density")
plt.title("Magnetometer vs Flight Duration")
plt.legend()
plt.grid()

fig, ax1 = plt.subplots()

# ---- Acceleration (primary left axis) ----
ax1.plot(time, accel_M["x"], label="Accel M X")
ax1.plot(time, accel_M["y"], label="Accel M Y")
ax1.plot(time, accel_M["z"], label="Accel M Z")
ax1.plot(time, accel_H, label="Accel High", linestyle="--")
ax1.set_xlabel("Flight Duration")
ax1.set_ylabel("Acceleration")
ax1.grid()

# ---- Gyroscope (second LEFT axis) ----
ax2 = ax1.twinx()
ax2.spines["right"].set_visible(False)
ax2.spines["left"].set_position(("outward", 60))
ax2.yaxis.set_label_position("left")
ax2.yaxis.tick_left()
ax2.set_ylabel("Angular Rate")

ax2.plot(time, gyro["x"], label="Gyro X", linestyle=":")
ax2.plot(time, gyro["y"], label="Gyro Y", linestyle=":")
ax2.plot(time, gyro["z"], label="Gyro Z", linestyle=":")

# ---- Magnetometer (third LEFT axis) ----
ax3 = ax1.twinx()
ax3.spines["right"].set_visible(False)
ax3.spines["left"].set_position(("outward", 120))
ax3.yaxis.set_label_position("left")
ax3.yaxis.tick_left()
ax3.set_ylabel("Magnetic Flux Density")

ax3.plot(time, mag["x"], label="Mag X", linestyle="-.")
ax3.plot(time, mag["y"], label="Mag Y", linestyle="-.")
ax3.plot(time, mag["z"], label="Mag Z", linestyle="-.")

# ---- Combined legend ----
lines = ax1.get_lines() + ax2.get_lines() + ax3.get_lines()
labels = [line.get_label() for line in lines]
ax1.legend(lines, labels, loc="upper right")

plt.title("Flight Sensor Data (All Sensors, Left Y-Axes)")
plt.show()