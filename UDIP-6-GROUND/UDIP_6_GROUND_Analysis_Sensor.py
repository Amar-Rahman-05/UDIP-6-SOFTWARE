from pathlib import Path
import csv
import math
import numpy as np
from scipy.interpolate import CubicSpline

BASE_DIR = Path(__file__).resolve().parent

SENSOR_CSV = "sensor_packets_0093.csv"
FILE_PATH  = BASE_DIR / "CSV-DATA" / "SENSOR" / SENSOR_CSV

# ─────────────────────────────────────────
# TRAJECTORY — cubic spline through waypoints
# ─────────────────────────────────────────
_WAYPOINTS = [
    (  0.0,   0.0),
    ( 60.0,  75.0),
    ( 80.0, 105.0),
    (217.6, 189.9),
    (336.0, 125.2),
]
_t_wp   = np.array([w[0] for w in _WAYPOINTS])
_alt_wp = np.array([w[1] for w in _WAYPOINTS])
_cs     = CubicSpline(_t_wp, _alt_wp)

_T_LAND    = _t_wp[-1]          # 336.0 s
_T_APOGEE  = 217.6              # s
_ALT_MAX   = 189.9              # km


# ─────────────────────────────────────────
# DATA LOADING
# ─────────────────────────────────────────

def load_data():
    """
    Load sensor data from CSV.

    CSV columns:
        count, tInitial, tFinal,
        accel_x, accel_y, accel_z, accel_high,
        gyro_x, gyro_y, gyro_z,
        mag_x, mag_y, mag_z,
        temp, status

    Returns
    -------
    time    : list[float]   – tFinal timestamps (ms)
    accel   : dict          – x, y, z, mag lists
    gyro    : dict          – x, y, z, mag lists
    mag     : dict          – x, y, z, mag lists
    accel_H : list[float]   – high-range accelerometer
    temp    : list[float]   – temperature readings
    """
    time  = []
    accel = {"x": [], "y": [], "z": [], "mag": []}
    gyro  = {"x": [], "y": [], "z": [], "mag": []}
    mag   = {"x": [], "y": [], "z": [], "mag": []}
    accel_H = []
    temp    = []

    with open(FILE_PATH, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            time.append(float(row["tFinal"]))

            ax, ay, az = float(row["accel_x"]), float(row["accel_y"]), float(row["accel_z"])
            accel["x"].append(ax)
            accel["y"].append(ay)
            accel["z"].append(az)
            accel["mag"].append(math.sqrt(ax**2 + ay**2 + az**2))

            accel_H.append(float(row["accel_high"]))

            gx, gy, gz = float(row["gyro_x"]), float(row["gyro_y"]), float(row["gyro_z"])
            gyro["x"].append(gx)
            gyro["y"].append(gy)
            gyro["z"].append(gz)
            gyro["mag"].append(math.sqrt(gx**2 + gy**2 + gz**2))

            mx, my, mz = float(row["mag_x"]), float(row["mag_y"]), float(row["mag_z"])
            mag["x"].append(mx)
            mag["y"].append(my)
            mag["z"].append(mz)
            mag["mag"].append(math.sqrt(mx**2 + my**2 + mz**2))

            temp.append(float(row["temp"]))

    return time, accel, gyro, mag, accel_H, temp


# ─────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────

def _time_in_seconds(time_ms):
    return [t / 1000.0 for t in time_ms]


def _apply_common_style(ax, title, xlabel, ylabel):
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.5)


# ─────────────────────────────────────────
# PLOT FUNCTIONS
# ─────────────────────────────────────────

def plot_accel(canvas):
    time, accel, *_ = load_data()
    t = _time_in_seconds(time)
    canvas.ax.clear()
    canvas.ax.plot(t, accel["x"], label="Ax")
    canvas.ax.plot(t, accel["y"], label="Ay")
    canvas.ax.plot(t, accel["z"], label="Az")
    canvas.ax.legend()
    _apply_common_style(canvas.ax,
                        "Acceleration vs Flight Duration",
                        "Time (s)", "Acceleration (raw LSB)")
    canvas.draw()


def plot_accel_magnitude(canvas):
    time, accel, *_ = load_data()
    t = _time_in_seconds(time)
    canvas.ax.clear()
    canvas.ax.plot(t, accel["mag"], color="tab:purple", label="|A|")
    canvas.ax.legend()
    _apply_common_style(canvas.ax,
                        "Acceleration Magnitude vs Time",
                        "Time (s)", "Magnitude (raw LSB)")
    canvas.draw()


def plot_high_accel(canvas):
    time, _, _, _, accel_H, _ = load_data()
    t = _time_in_seconds(time)
    canvas.ax.clear()
    canvas.ax.plot(t, accel_H, color="tab:orange")
    _apply_common_style(canvas.ax,
                        "High-Range Acceleration vs Time",
                        "Time (s)", "Acceleration (raw LSB)")
    canvas.draw()


def plot_gyro(canvas):
    time, _, gyro, *_ = load_data()
    t = _time_in_seconds(time)
    canvas.ax.clear()
    canvas.ax.plot(t, gyro["x"], label="Gx")
    canvas.ax.plot(t, gyro["y"], label="Gy")
    canvas.ax.plot(t, gyro["z"], label="Gz")
    canvas.ax.legend()
    _apply_common_style(canvas.ax,
                        "Gyroscope vs Time",
                        "Time (s)", "Angular Velocity (raw LSB)")
    canvas.draw()


def plot_mag(canvas):
    time, _, _, mag, *_ = load_data()
    t = _time_in_seconds(time)
    canvas.ax.clear()
    canvas.ax.plot(t, mag["x"], label="Mx")
    canvas.ax.plot(t, mag["y"], label="My")
    canvas.ax.plot(t, mag["z"], label="Mz")
    canvas.ax.legend()
    _apply_common_style(canvas.ax,
                        "Magnetic Flux Density vs Time",
                        "Time (s)", "Magnetic Field (raw LSB)")
    canvas.draw()


def plot_temp(canvas):
    time, *_, temp = load_data()
    t = _time_in_seconds(time)
    canvas.ax.clear()
    canvas.ax.plot(t, temp, color="tab:red")
    _apply_common_style(canvas.ax,
                        "Temperature vs Flight Duration",
                        "Time (s)", "Temperature (raw LSB)")
    canvas.draw()


def plot_trajectory(canvas):
    """
    Cubic-spline flight trajectory through mission waypoints.
    """
    t_full   = np.linspace(0.0, _T_LAND, 1000)
    alt_full = np.clip(_cs(t_full), 0, None)

    # Waypoint annotations
    wp_colors = {
        "Launch":                "black",
        "Despin":                "darkorange",
        "Probe deployment begin, Sweep collection begin": "tomato",
        "Apogee":                "mediumblue",
        "Data collection end":   "tomato",
    }
    wp_labels = [
        "Launch",
        "Despin",
        "Probe deployment begin, Sweep collection begin",
        "Apogee",
        "Data collection end",
    ]

    canvas.ax.clear()
    canvas.ax.plot(t_full, alt_full, color="tab:green", linewidth=2)
    canvas.ax.fill_between(t_full, alt_full, alpha=0.15, color="tab:green")

    for (t_w, alt_w), lbl in zip(_WAYPOINTS, wp_labels):
        col = wp_colors.get(lbl, "gray")
        canvas.ax.scatter(t_w, alt_w, color=col, s=60, zorder=5)
        canvas.ax.annotate(
            f"{lbl}\n({t_w}s)",
            xy=(t_w, alt_w),
            xytext=(6, 6), textcoords="offset points",
            fontsize=7, color=col,
            arrowprops=dict(arrowstyle="->", color=col, lw=0.7),
        )

    _apply_common_style(canvas.ax,
                        "Flight Trajectory (Cubic Spline)",
                        "Mission Elapsed Time (s)", "Altitude (km)")
    canvas.draw()


def plot_cumulative(canvas):
    time, accel, gyro, mag, accel_H, _ = load_data()
    t = _time_in_seconds(time)
    canvas.ax.clear()
    for label, data in [
        ("Ax", accel["x"]), ("Ay", accel["y"]), ("Az", accel["z"]),
        ("Gx", gyro["x"]),  ("Gy", gyro["y"]),  ("Gz", gyro["z"]),
        ("Mx", mag["x"]),   ("My", mag["y"]),   ("Mz", mag["z"]),
        ("Hi-G", accel_H),
    ]:
        canvas.ax.plot(t, data, label=label)
    canvas.ax.set_title("Cumulative Sensor Data", fontsize=12, fontweight="bold")
    canvas.ax.legend(fontsize=7, ncol=2)
    canvas.ax.grid(True, linestyle="--", alpha=0.5)
    canvas.draw()
