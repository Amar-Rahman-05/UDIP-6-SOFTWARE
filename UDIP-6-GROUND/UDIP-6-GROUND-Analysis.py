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
import matplotlib.pyplot as plt

sensor_csv = "sensor_packets_YYYY-MM-DD_HH-MM-SS.csv"  

time = []
accel_M = {"x": [], "y": [], "z": []}
accel_H = []
gyro    = {"x": [], "y": [], "z": []}
mag     = {"x": [], "y": [], "z": []}
temp    = []

with open(sensor_csv, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t = float(row["tFinal"]) - float(row["tInitial"])
        time.append(t)

        accel_M["x"].append(float(row["accel_M_x"]))
        accel_M["y"].append(float(row["accel_M_y"]))
        accel_M["z"].append(float(row["accel_M_z"]))

        accel_H["x"].append(float(row["accel_H_x"]))
        accel_H["y"].append(float(row["accel_H_y"]))
        accel_H["z"].append(float(row["accel_H_z"]))

        gyro["x"].append(float(row["gyro_M_x"]))
        gyro["y"].append(float(row["gyro_M_y"]))
        gyro["z"].append(float(row["gyro_M_z"]))

        mag["x"].append(float(row["mag_M_x"]))
        mag["y"].append(float(row["mag_M_y"]))
        mag["z"].append(float(row["mag_M_z"]))

        temp.append(float(row["temp"]))
        photo.append(float(row["photo"]))


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
plt.show()


# High Range Acceleration

plt.figure()
plt.plot(time, accel_H["x"], label="High Accel X")
plt.plot(time, accel_H["y"], label="High Accel Y")
plt.plot(time, accel_H["z"], label="High Accel Z")
plt.xlabel("Flight Duration")
plt.ylabel("Acceleration (High Range)")
plt.title("High Range Acceleration vs Flight Duration")
plt.legend()
plt.grid()
plt.show()


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
plt.show()


# Temperature vs Flight Duration

plt.figure()
plt.plot(time, temp)
plt.xlabel("Flight Duration")
plt.ylabel("Temperature")
plt.title("Temperature vs Flight Duration")
plt.grid()
plt.show()


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
plt.show()

