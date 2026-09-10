import pandas as pd
import numpy as np
from pathlib import Path

# =============================================================================
# SENSOR CONFIGURATION
# These values must match the ranges configured in the flight firmware.
# =============================================================================

# LSM9DS1 accelerometer, ±2 g
ACCEL_SENS_G_PER_LSB = 0.061e-3

# LSM9DS1 gyroscope, ±245 degrees/second
GYRO_SENS_DPS_PER_LSB = 8.75e-3

# LIS3MDL magnetometer, ±4 gauss
MAG_SENS_GAUSS_PER_LSB = 0.14e-3

# H3LIS331DL configured full-scale range.
# Change to 200 or 400 if that range was used in the flight firmware.
HIGH_G_RANGE_G = 100

# The binary converter preserves the left-justified signed 16-bit output.
HIGH_G_SENS_G_PER_LSB = HIGH_G_RANGE_G / 32768.0

# LSM9DS1 internal temperature sensor
TEMP_OFFSET_C = 25.0
TEMP_SENS_LSB_PER_C = 16.0

# Standard gravitational acceleration
G0 = 9.80665

# Directory containing this Python file
BASE_DIR = Path(__file__).resolve().parent


def _apply_common_style(
    ax,
    title,
    xlabel,
    ylabel,
    show_legend=True
):
    ax.set_title(
        title,
        fontsize=12,
        fontweight="bold"
    )

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    ax.grid(
        True,
        linestyle="--",
        alpha=0.4
    )

    ax.tick_params(
        axis="both",
        labelsize=10
    )

    if show_legend:
        handles, labels = ax.get_legend_handles_labels()

        if handles:
            ax.legend(loc="best")


def load_data(dat_number):
    csv_name = (
        f"sensor_packets_{int(dat_number):04d}.csv"
    )

    file_path = (
        BASE_DIR
        / "CSV-DATA"
        / "SENSOR"
        / csv_name
    )

    if not file_path.exists():
        raise FileNotFoundError(
            f"Sensor CSV file not found: {file_path}"
        )

    df = pd.read_csv(file_path)

    required_columns = {
        "tFinal",
        "accel_x",
        "accel_y",
        "accel_z",
        "accel_high",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "mag_x",
        "mag_y",
        "mag_z",
        "temp",
    }

    missing = required_columns - set(df.columns)

    if missing:
        raise ValueError(
            f"Sensor CSV is missing columns: {sorted(missing)}"
        )

    # Packet timestamps are stored in milliseconds.
    df["time_s"] = df["tFinal"] / 1000.0

    # -------------------------------------------------------------------------
    # LSM9DS1 accelerometer
    #
    # raw LSB × g/LSB × m/s² per g
    # -------------------------------------------------------------------------
    for axis in ("x", "y", "z"):
        df[f"accel_{axis}_si"] = (
            df[f"accel_{axis}"]
            * ACCEL_SENS_G_PER_LSB
            * G0
        )

    # -------------------------------------------------------------------------
    # H3LIS331DL high-G accelerometer
    #
    # This device must use its own sensitivity.
    # -------------------------------------------------------------------------
    df["accel_high_g"] = (
        df["accel_high"]
        * HIGH_G_SENS_G_PER_LSB
    )

    df["accel_high_si"] = (
        df["accel_high_g"] * G0
    )

    # -------------------------------------------------------------------------
    # LSM9DS1 gyroscope
    #
    # raw LSB × degrees/second per LSB → radians/second
    # -------------------------------------------------------------------------
    for axis in ("x", "y", "z"):
        gyro_dps = (
            df[f"gyro_{axis}"]
            * GYRO_SENS_DPS_PER_LSB
        )

        # Primary display unit: degrees per second (DPS).
        df[f"gyro_{axis}_dps"] = gyro_dps

        # Retain the SI conversion for any other code that uses it.
        df[f"gyro_{axis}_si"] = np.deg2rad(
            gyro_dps
        )

    # -------------------------------------------------------------------------
    # LIS3MDL magnetometer
    #
    # raw LSB × gauss/LSB × 10^-4 tesla/gauss
    # -------------------------------------------------------------------------
    for axis in ("x", "y", "z"):
        # Primary display unit: gauss.
        df[f"mag_{axis}_G"] = (
            df[f"mag_{axis}"]
            * MAG_SENS_GAUSS_PER_LSB
        )

        # Retain the SI conversion in tesla for compatibility.
        df[f"mag_{axis}_si"] = (
            df[f"mag_{axis}_G"] * 1e-4
        )

    # -------------------------------------------------------------------------
    # LSM9DS1 internal temperature
    #
    # T_C = 25 + raw/16
    # T_K = T_C + 273.15
    # -------------------------------------------------------------------------
    df["temp_C"] = (
        TEMP_OFFSET_C
        + df["temp"] / TEMP_SENS_LSB_PER_C
    )

    df["temp_K"] = (
        df["temp_C"] + 273.15
    )

    return df


def plot_accel(canvas, dat_number):
    df = load_data(dat_number)

    canvas.ax.clear()

    canvas.ax.plot(
        df["time_s"],
        df["accel_x_si"],
        label="X"
    )

    canvas.ax.plot(
        df["time_s"],
        df["accel_y_si"],
        label="Y"
    )

    canvas.ax.plot(
        df["time_s"],
        df["accel_z_si"],
        label="Z"
    )

    _apply_common_style(
        canvas.ax,
        "UDIP-6 Accelerometer",
        "Time (s)",
        "Acceleration (m/s²)"
    )

    canvas.draw()


def plot_high_accel(canvas, dat_number):
    df = load_data(dat_number)

    canvas.ax.clear()

    canvas.ax.plot(
        df["time_s"],
        df["accel_high_si"],
        label="High-G"
    )

    _apply_common_style(
        canvas.ax,
        "UDIP-6 High-G Accelerometer",
        "Time (s)",
        "Acceleration (m/s²)"
    )

    canvas.draw()


def plot_gyro(canvas, dat_number):
    df = load_data(dat_number)

    canvas.ax.clear()

    canvas.ax.plot(
        df["time_s"],
        df["gyro_x_dps"],
        label="X"
    )

    canvas.ax.plot(
        df["time_s"],
        df["gyro_y_dps"],
        label="Y"
    )

    canvas.ax.plot(
        df["time_s"],
        df["gyro_z_dps"],
        label="Z"
    )

    _apply_common_style(
        canvas.ax,
        "UDIP-6 Gyroscope",
        "Time (s)",
        "Angular Frequency (DPS)"
    )

    canvas.draw()


def plot_mag(canvas, dat_number):
    df = load_data(dat_number)

    canvas.ax.clear()

    canvas.ax.plot(
        df["time_s"],
        df["mag_x_G"],
        label="X"
    )

    canvas.ax.plot(
        df["time_s"],
        df["mag_y_G"],
        label="Y"
    )

    canvas.ax.plot(
        df["time_s"],
        df["mag_z_G"],
        label="Z"
    )

    _apply_common_style(
        canvas.ax,
        "UDIP-6 Magnetometer",
        "Time (s)",
        "Magnetic Flux Density (G)"
    )

    canvas.draw()


def plot_temp(canvas, dat_number):
    df = load_data(dat_number)

    canvas.ax.clear()

    canvas.ax.plot(
        df["time_s"],
        df["temp_C"],
        label="Temperature"
    )

    _apply_common_style(
        canvas.ax,
        "UDIP-6 Temperature",
        "Time (s)",
        "Temperature (°C)"
    )

    canvas.draw()
