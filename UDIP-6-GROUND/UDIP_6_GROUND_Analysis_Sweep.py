# =============================================================================
# UDIP-6 sweep analysis module
# =============================================================================


from pathlib import Path
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import numpy as np
from calibration import CAL
from scipy.signal import savgol_filter


# =============================================================================
# Sweep and physics constants
# =============================================================================

# Number of voltage/current samples in one complete sweep packet.
#   0-88    : 0 -> +V
#   89-177  : +V -> 0
#   178-266 : 0 -> -V
#   267-355 : -V -> 0
# Total: 89 * 4 = 356 points.
N_SWP_STEP = 356
# 12-bit ADC maximum count.
# The Arduino Due uses analogReadResolution(12), so ADC counts span 0-4095.
# ADC voltage equation used later:
#   V_adc = (ADC_count / ADC_RES) * CAL.adc_vref
ADC_RES = 4095.0
# Physical constants used in the double-probe equations.
E_CHARGE = 1.602176634e-19       # elementary charge, C
K_B = 1.380649e-23              # Boltzmann constant, J/K
EV_TO_K = 11604.51812           # kelvin per electronvolt
BOHM_COEFFICIENT = 0.61         # ion-saturation Bohm-current coefficient

# Backward-compatible aliases used by existing comments/equations.
e = E_CHARGE
k = K_B


# =============================================================================
# Flight trajectory lookup table
# =============================================================================

# Flight trajectory time values in seconds. convert experiment time to flight time using:
#   flight_time_s = experiment_time_s - time_offset_s
# time_offset_s defaults to 180 seconds because experiment starts 3 minutes before launch.
TRAJ_TIME_S = np.array([
    0, 0.1, 0.1, 0.5, 1,
    6.2, 15, 26, 60, 64,
    65, 66, 68, 71, 79,
    80, 82, 85, 87, 110,
    160, 194, 217.6, 302, 305,
    306, 326, 336, 363.5, 520.4,
    856.5
], dtype=np.float64)


# Flight altitude values in kilometers corresponding element-by-element to
# TRAJ_TIME_S. The altitude for each sweep is found by linear interpolation:
#   altitude_km = np.interp(flight_time_s, TRAJ_TIME_S, TRAJ_ALT_KM)
TRAJ_ALT_KM = np.array([
    0, 0, 0, 0, 0,
    2.3, 7.7, 20.7, 75, 80.8,
    82.2, 83.6, 86.4, 91.5, 101.1,
    102.5, 105.4, 109.3, 112.1, 137.4,
    175.3, 187.6, 189.9, 157.1, 154.6,
    153.7, 135.4, 125.2, 91.4, 4.6,
    0
], dtype=np.float64)


# Approximate ionosphere/atmospheric altitude regions used for shaded plot bands.
# Each tuple is:
#   (region_label, lower_altitude_km, upper_altitude_km, display_color)
# These regions are visual guides only; actual layer boundaries vary with time,
# solar activity, latitude, and geomagnetic conditions.
IONOSPHERE_LAYERS = [
    ("Lower atmosphere", 0,   60,  "#d9d9d9"),  # light gray
    ("D layer",          60,  90,  "#f4d35e"),  # yellow
    ("E layer",          90,  150, "#8fd694"),  # green
    ("F1 layer",         150, 220, "#7fb3ff"),  # blue
    ("F2 layer",         220, 1000,"#c9a0ff"),  # purple
]


# =============================================================================
# Fixed graph-axis limits
# =============================================================================
# These limits are applied after every canvas clear so changing the selected
# sweep or reopening a graph cannot cause Matplotlib to autoscale the axes.
# Adjust these mission-wide constants if a later dataset requires more range.
IV_VOLTAGE_LIMITS_V = (-9.0, 9.0)
IV_CURRENT_LIMITS_UA = (-6.0, 6.0)

PLASMA_TIME_LIMITS_S = (225.0, 550.0)
ELECTRON_TEMPERATURE_LIMITS_K = (0.0, 1.7e7)
ELECTRON_DENSITY_LIMITS_M3 = (0.0, 1.1e16)
ALTITUDE_LIMITS_KM = (0.0, 200.0)


# Base folder containing this Python file.
# CSV paths are built relative to BASE_DIR so the GUI can find:
#   CSV-DATA/SWEEP/sweep_packets_XXXX.csv
BASE_DIR = Path(__file__).resolve().parent


# -----------------------------------------------------------------------------
# _get_sweep_csv(dat_number)
# -----------------------------------------------------------------------------
# Purpose:
#   Build the path to the converted sweep CSV for a selected DAT number.
#
# Input:
#   dat_number: string such as "0001" or "0096".
#
# Output:
#   Path object pointing to:
#     BASE_DIR / CSV-DATA / SWEEP / sweep_packets_<dat_number>.csv
#
# Equation/path pattern:
#   csv_path = BASE_DIR / "CSV-DATA" / "SWEEP" / f"sweep_packets_{dat_number}.csv"
def _get_sweep_csv(dat_number: str) -> Path:
    return BASE_DIR / "CSV-DATA" / "SWEEP" / f"sweep_packets_{dat_number}.csv"


# -----------------------------------------------------------------------------
# dac_to_voltage(vrefA, vrefB)
# -----------------------------------------------------------------------------
# Purpose:
#   Convert sweep monitor ADC counts into probe voltage in volts.
#
# Inputs:
#   vrefA, vrefB: ADC-count arrays from the sweep voltage monitor channels.
#
# Equations:
#   vx1 = (vrefA / ADC_RES) * CAL.adc_vref
#   vx2 = (vrefB / ADC_RES) * CAL.adc_vref
#   V_probe = (vx2 - vx1) * CAL.voltage_gain
#
# Output:
#   Probe voltage array in volts.
def dac_to_voltage(vrefA, vrefB):

    vrefA = np.asarray(vrefA, dtype=np.float64)
    vrefB = np.asarray(vrefB, dtype=np.float64)

    vx1 = (vrefA / ADC_RES) * CAL.adc_vref
    vx2 = (vrefB / ADC_RES) * CAL.adc_vref

    return (vx2 - vx1) * CAL.voltage_gain



# -----------------------------------------------------------------------------
# adc_to_current(adcA, adcB)
# -----------------------------------------------------------------------------
# Purpose:
#   Convert probe ADC channel counts into probe current in nanoamps.
#
# Inputs:
#   adcA, adcB: ADC-count arrays from the two current measurement channels.
#
# Equations:
#   vA = (adcA / ADC_RES) * CAL.adc_vref
#   vB = (adcB / ADC_RES) * CAL.adc_vref
#   v_diff = (vA - vB) * CAL.current_gain
#   I_A = v_diff / CAL.shunt_resistance
#   I_nA = I_A * 1e9
#
# Output:
#   Probe current array in nanoamps.
def adc_to_current(adcA, adcB):

    adcA = np.asarray(adcA, dtype=np.float64)
    adcB = np.asarray(adcB, dtype=np.float64)

    vA = (adcA / ADC_RES) * CAL.adc_vref
    vB = (adcB / ADC_RES) * CAL.adc_vref

    v_diff = (vA - vB) * CAL.current_gain

    current = v_diff / CAL.shunt_resistance

    return current * 1e9


# -----------------------------------------------------------------------------
# correct_hysteresis(V, I)
# -----------------------------------------------------------------------------
# Purpose:
#   Collapse the four raw sweep branches into one corrected dual-probe I-V curve
#   by averaging matching forward and reverse voltage passes.
#
# Sweep layout:
#   0-88    : 0 -> +V
#   89-177  : +V -> 0
#   178-266 : 0 -> -V
#   267-355 : -V -> 0
#
# Branch averaging equations:
#   V_pos = 0.5 * (V_pos_forward + V_pos_reverse)
#   I_pos = 0.5 * (I_pos_forward + I_pos_reverse)
#   V_neg = 0.5 * (V_neg_forward + V_neg_reverse)
#   I_neg = 0.5 * (I_neg_forward + I_neg_reverse)
#
# Current-centering equation near V = 0:
#   I_corrected = Icorr - median(Icorr where |Vcorr| < 1 V)
#
# Output:
#   Vcorr, Icorr: corrected voltage/current arrays sorted by voltage.
def _prepare_hysteresis_branch(V_branch, I_branch):
    """Clean and sort one sweep branch by its measured voltage."""
    V_branch = np.asarray(V_branch, dtype=np.float64)
    I_branch = np.asarray(I_branch, dtype=np.float64)

    mask = np.isfinite(V_branch) & np.isfinite(I_branch)
    V_branch = V_branch[mask]
    I_branch = I_branch[mask]

    if len(V_branch) == 0:
        return V_branch, I_branch

    order = np.argsort(V_branch)
    V_branch = V_branch[order]
    I_branch = I_branch[order]

    # Average any repeated voltage coordinates before interpolation.
    unique_V, inverse = np.unique(V_branch, return_inverse=True)
    I_sums = np.bincount(inverse, weights=I_branch)
    I_counts = np.bincount(inverse)

    return unique_V, I_sums / I_counts


def _average_hysteresis_pair(V_forward, I_forward, V_reverse, I_reverse):
    """
    Average forward and reverse currents at identical voltage coordinates.

    This accounts for hysteresis without filtering or deleting measurements.
    """
    Vf, If = _prepare_hysteresis_branch(V_forward, I_forward)
    Vr, Ir = _prepare_hysteresis_branch(V_reverse, I_reverse)

    if min(len(Vf), len(Vr)) < 2:
        return np.array([]), np.array([])

    overlap_min = max(Vf.min(), Vr.min())
    overlap_max = min(Vf.max(), Vr.max())

    if overlap_max <= overlap_min:
        return np.array([]), np.array([])

    V_common = np.linspace(overlap_min, overlap_max, 89)
    I_forward_common = np.interp(V_common, Vf, If)
    I_reverse_common = np.interp(V_common, Vr, Ir)

    # Hysteresis correction: use the midpoint of the two sweep directions.
    I_average = 0.5 * (I_forward_common + I_reverse_common)

    return V_common, I_average


def correct_hysteresis(V, I):
    """
    Account for forward/reverse hysteresis using voltage-aligned averaging.

    No temperature, density, current, or fit-quality filter is applied here.
    Every complete sweep continues to the existing tanh fitting calculation.
    """
    V = np.asarray(V, dtype=np.float64)
    I = np.asarray(I, dtype=np.float64)

    if len(V) != N_SWP_STEP or len(I) != N_SWP_STEP:
        return V, I

    N = N_SWP_STEP // 4

    V_first, I_first = _average_hysteresis_pair(
        V[0:N], I[0:N],
        V[N:2*N], I[N:2*N]
    )

    V_second, I_second = _average_hysteresis_pair(
        V[2*N:3*N], I[2*N:3*N],
        V[3*N:4*N], I[3*N:4*N]
    )

    if len(V_first) == 0 or len(V_second) == 0:
        return V, I

    Vcorr = np.concatenate((V_first, V_second))
    Icorr = np.concatenate((I_first, I_second))

    order = np.argsort(Vcorr)
    Vcorr = Vcorr[order]
    Icorr = Icorr[order]

    # Retain the existing zero-current centering without rejecting any points.
    zero_mask = np.abs(Vcorr) < 1.0
    if np.count_nonzero(zero_mask) >= 3:
        Icorr = Icorr - np.median(Icorr[zero_mask])

    return Vcorr, Icorr


# -----------------------------------------------------------------------------
# make_display_iv_curve(V, I)
# -----------------------------------------------------------------------------
# 
# Build a visually cleaner I-V curve for display in the GUI.
#
# Processing steps:
#   1. Apply hysteresis correction.
#   2. Remove non-finite values.
#   3. Sort by voltage.
#   4. Interpolate onto a symmetric voltage grid.
#   5. Enforce approximate dual-probe odd symmetry.
#   6. Smooth the display curve using Savitzky-Golay filtering.
#
# Dual-probe symmetry equation:
#   I(-V) ≈ -I(V)
#
# Symmetrization equation used here:
#   I_sym(V) = 0.5 * [I_interp(V) - I_interp(-V)]
#
# Output:
#   V_grid, I_sym, I_smooth for plotting only.
def make_display_iv_curve(V, I):

    V, I = correct_hysteresis(V, I)

    mask = np.isfinite(V) & np.isfinite(I)
    V = V[mask]
    I = I[mask]

    if len(V) < 20:
        return V, I, I

    order = np.argsort(V)
    V = V[order]
    I = I[order]

    vmax = min(abs(np.nanmin(V)), abs(np.nanmax(V)))
    V_grid = np.linspace(-vmax, vmax, 160)

    I_interp = np.interp(V_grid, V, I)
    I_mirror = np.interp(-V_grid, V, I)

    # Enforce approximate dual-probe symmetry: I(-V) = -I(V)
    I_sym = 0.5 * (I_interp - I_mirror)

    window = 17

    if len(I_sym) <= window:
        window = len(I_sym) - 1

    if window % 2 == 0:
        window -= 1

    if window >= 5:
        I_smooth = savgol_filter(I_sym, window_length=window, polyorder=3)
    else:
        I_smooth = I_sym

    return V_grid, I_sym, I_smooth


# -----------------------------------------------------------------------------
# load_data(dat_number)
# -----------------------------------------------------------------------------
# Purpose:
#   Load the converted sweep CSV, validate required columns, group rows by sweep
#   packet, convert raw ADC values into physical V/I arrays, and return one
#   dictionary per sweep.
#
# CSV grouping:
#   Each packet_num corresponds to one sweep packet.
#   Each valid sweep should contain N_SWP_STEP = 356 rows.
#
# Conversion equations used inside this function:
#   V = dac_to_voltage(v_refA, v_refB)
#   I = adc_to_current(adc_A, adc_B)
#
# Output list element fields:
#   packet_num, count, num_points, min_step, max_step, tInitial, tFinal, V, I
def load_data(dat_number: str = "0001") -> list[dict]:

    csv_path = _get_sweep_csv(dat_number)

    if not csv_path.exists():
        print(f"[ERROR] CSV not found: {csv_path}")
        return []

    df = pd.read_csv(csv_path)

    required_cols = {
        "packet_num",
        "count",
        "step",
        "tInitial",
        "tFinal",
        "v_refA",
        "v_refB",
        "adc_A",
        "adc_B",
    }

    missing = required_cols - set(df.columns)

    if missing:
        print(f"[ERROR] Missing columns: {missing}")
        return []

    print(f"Loaded {len(df)} rows from {csv_path.name}")

    sweeps = []

    for packet_num, group in df.groupby("packet_num", sort=True):

        group = (
            group
            .sort_values("step")
            .drop_duplicates("step")
        )

        point_count = len(group)

        min_step = int(group["step"].min())
        max_step = int(group["step"].max())

        if point_count != N_SWP_STEP:
            print(
                f"[WARNING] Packet {packet_num}: "
                f"{point_count} points "
                f"(steps {min_step}-{max_step}, "
                f"expected {N_SWP_STEP})"
            )

        v_refA = group["v_refA"].to_numpy(dtype=np.float64)
        v_refB = group["v_refB"].to_numpy(dtype=np.float64)

        adc_A = group["adc_A"].to_numpy(dtype=np.float64)
        adc_B = group["adc_B"].to_numpy(dtype=np.float64)

        # time
        t0 = group["tInitial"].iloc[0]
        t1 = group["tFinal"].iloc[0]

        V = dac_to_voltage(v_refA, v_refB)
        I = adc_to_current(adc_A, adc_B)

        print(
            f"Packet {packet_num}: "
            f"v_refA={len(v_refA)}, "
            f"v_refB={len(v_refB)}, "
            f"adc_A={len(adc_A)}, "
            f"adc_B={len(adc_B)}"
        )

        sweeps.append({
            "packet_num": int(packet_num),
            "count": int(group["count"].iloc[0]),
            "num_points": point_count,
            "min_step": min_step,
            "max_step": max_step,
            "tInitial": float(t0),
            "tFinal": float(t1),
            "V": V,
            "I": I,
        })

    print(f"Loaded {len(sweeps)} sweeps")

    return sweeps




# -----------------------------------------------------------------------------
# print_sweep_summary(dat_number)
# -----------------------------------------------------------------------------
# Purpose:
#   Print a table showing how many step rows each sweep packet contains.
#
# Validation rule:
#   A complete sweep should have:
#     points = N_SWP_STEP = 356
#
# This function is for debugging CSV conversion completeness.
def print_sweep_summary(dat_number: str = "0001"):

    csv_path = _get_sweep_csv(dat_number)
    df = pd.read_csv(csv_path)

    summary = (
        df.groupby("packet_num")
        .agg(
            points=("step", "count"),
            min_step=("step", "min"),
            max_step=("step", "max"),
        )
        .reset_index()
    )

    print(summary)

    bad = summary[summary["points"] != N_SWP_STEP]

    if len(bad):
        print("\nIncomplete sweeps:")
        print(bad)
    else:
        print("\nAll sweeps contain 356 points.")


# -----------------------------------------------------------------------------
# plot_raw_iv_curve(canvas, sweeps, sweep_idx)
# -----------------------------------------------------------------------------
# Purpose:
#   Plot the original raw four-branch I-V sweep for one selected packet without
#   hysteresis correction or symmetry smoothing.
#
# Display behavior:
#   - Separates the four sweep branches when a full 356-point sweep is present.
#   - Displays current consistently in µA.
#   - Uses fixed voltage and current limits for every selected sweep.
#
# Unit conversion for display only:
#   I_plot = I / scale
# where scale = 1 for nA and scale = 1000 for µA.
def plot_raw_iv_curve(canvas, sweeps: list[dict], sweep_idx: int = 0):
    """
    Plot the raw I-V curve without hysteresis correction.

    This shows the original four sweep branches:
        0-88    : 0 -> +V
        89-177  : +V -> 0
        178-266 : 0 -> -V
        267-355 : -V -> 0
    """

    if not sweeps or sweep_idx >= len(sweeps):
        return

    s = sweeps[sweep_idx]

    V = np.asarray(s["V"], dtype=np.float64)
    I = np.asarray(s["I"], dtype=np.float64)

    mask = np.isfinite(V) & np.isfinite(I)
    V = V[mask]
    I = I[mask]

    ax = canvas.ax
    ax.clear()

    print(f"RAW V range: {np.nanmin(V):.3f} -> {np.nanmax(V):.3f}")
    print(f"RAW I range: {np.nanmin(I):.3f} -> {np.nanmax(I):.3f}")

    # Keep one display unit for every sweep so the fixed y-axis is meaningful.
    I_plot = I / 1000.0
    ylabel = "Current (µA)"

    N = 89

    if len(V) >= N_SWP_STEP:
        ax.plot(V[0:N], I_plot[0:N], marker='.', linewidth=1, label="0 → +V")
        ax.plot(V[N:2*N], I_plot[N:2*N], marker='.', linewidth=1, label="+V → 0")
        ax.plot(V[2*N:3*N], I_plot[2*N:3*N], marker='.', linewidth=1, label="0 → -V")
        ax.plot(V[3*N:4*N], I_plot[3*N:4*N], marker='.', linewidth=1, label="-V → 0")
    else:
        ax.plot(V, I_plot, marker='.', linewidth=1, label="Raw I-V")

    ax.legend()

    # Fixed axes: selecting another sweep does not change the graph scale.
    ax.set_xlim(*IV_VOLTAGE_LIMITS_V)
    ax.set_ylim(*IV_CURRENT_LIMITS_UA)

    ax.axhline(0, color='black', linewidth=0.6)
    ax.axvline(0, color='black', linewidth=0.6)

    # time (ms → s)
    t0 = s["tInitial"] / 1000
    t1 = s["tFinal"] / 1000

    ax.set_title(
        f"Raw I–V Curve (ID: {s['packet_num']} | "
        f"Time: {t0:.3f}s – {t1:.3f}s | "
        f"Rshunt: {CAL.shunt_resistance}Ω)"
    )

    ax.set_xlabel("Voltage (V)")
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.5)

    canvas.draw()
    

# -----------------------------------------------------------------------------
# plot_iv_curve(canvas, sweeps, sweep_idx)
# -----------------------------------------------------------------------------
# Purpose:
#   Plot the corrected dual-probe I-V curve for one selected sweep.
#
# Processing chain:
#   raw V/I -> make_display_iv_curve() -> corrected data + smoothed display curve
#
# Important display equations inherited from make_display_iv_curve():
#   I_sym(V) = 0.5 * [I_interp(V) - I_interp(-V)]
#   I_smooth = Savitzky-Golay-filtered I_sym
#
# Display behavior:
#   - Displays current consistently in µA.
#   - Uses fixed voltage and current limits for every selected sweep.
#   - Draws zero-voltage and zero-current reference lines.
def plot_iv_curve(canvas, sweeps: list[dict], sweep_idx: int = 0):

    if not sweeps or sweep_idx >= len(sweeps):
        return

    s = sweeps[sweep_idx]

    V = np.array(s["V"], dtype=np.float64)
    I = np.array(s["I"], dtype=np.float64)

    V, I, I_smooth = make_display_iv_curve(V, I)

    ax = canvas.ax
    ax.clear()

    print(
    f"V range: {np.nanmin(V):.3f} -> {np.nanmax(V):.3f}"
)
    print(
    f"I range: {np.nanmin(I):.3f} -> {np.nanmax(I):.3f}"
)
    # Keep one display unit for every sweep so the fixed y-axis is meaningful.
    I_plot = I / 1000.0
    I_smooth_plot = I_smooth / 1000.0
    ylabel = "Current (µA)"

    ax.scatter(V, I_plot, s=10, c='blue', alpha=0.7, label="Corrected data")
    ax.plot(V, I_smooth_plot, linewidth=2, color='black', label="Smoothed curve")
    ax.legend()

    # Fixed axes: selecting another sweep does not change the graph scale.
    ax.set_xlim(*IV_VOLTAGE_LIMITS_V)
    ax.set_ylim(*IV_CURRENT_LIMITS_UA)

    ax.set_ylabel(ylabel)

    ax.axhline(0, color='black', linewidth=0.6)
    ax.axvline(0, color='black', linewidth=0.6)

    # time (ms → s)
    t0 = s["tInitial"] / 1000
    t1 = s["tFinal"] / 1000

    ax.set_title(
        f"I–V Curve (ID: {s['packet_num']} | "
        f"Time: {t0:.3f}s – {t1:.3f}s | "
        f"Rshunt: {CAL.shunt_resistance}Ω)"
    )

    ax.set_xlabel("Voltage (V)")
    ax.grid(True, linestyle="--", alpha=0.5)

    canvas.draw()



# -----------------------------------------------------------------------------
# _tanh_model(V, I0, a, V0, Ioff)
# -----------------------------------------------------------------------------
# Purpose:
#   Define the dual Langmuir probe tanh model used by curve_fit.
#
# Model equation:
#   I(V) = Ioff + I0 * tanh(a * (V - V0))
#
# Parameter meanings:
#   I0   : saturation-current scale, nA
#   a    : tanh steepness parameter, in 1/V
#   V0   : horizontal voltage offset, in V
#   Ioff : vertical current offset, in nA
def _tanh_model(V, I0, a, V0, Ioff):
    return Ioff + I0 * np.tanh(a * (V - V0))


def fit_double_probe_curve(V, I_nA):
    """
    Fit one hysteresis-corrected I-V curve to the symmetric double-probe model.

    No R-squared, temperature, density, percentile, or outlier filter is used.
    The 5th/95th current percentiles provide only robust initial guesses for the
    nonlinear fit; the returned saturation current comes from the fitted I0.
    """
    V = np.asarray(V, dtype=np.float64)
    I_nA = np.asarray(I_nA, dtype=np.float64)

    mask = np.isfinite(V) & np.isfinite(I_nA)
    V = V[mask]
    I_nA = I_nA[mask]

    if len(V) < 20:
        raise ValueError("At least 20 finite I-V points are required")

    order = np.argsort(V)
    V = V[order]
    I_nA = I_nA[order]

    I_low = np.percentile(I_nA, 5)
    I_high = np.percentile(I_nA, 95)
    I_sat_guess = max(0.5 * (I_high - I_low), np.finfo(float).eps)
    I_offset_guess = 0.5 * (I_high + I_low)

    popt, pcov = curve_fit(
        _tanh_model,
        V,
        I_nA,
        p0=[I_sat_guess, 0.5, 0.0, I_offset_guess],
        maxfev=20000
    )

    I_sat_nA, a, V0, I_offset_nA = popt
    if not np.isfinite(a) or a == 0:
        raise ValueError("The fitted tanh steepness is zero or non-finite")

    # T_e(eV) = 1/(2|a|), equivalent to e/(2*k_B*|a|) in kelvin.
    Te_eV = 1.0 / (2.0 * abs(a))
    Te_K = Te_eV * EV_TO_K

    I_fitted_nA = _tanh_model(V, *popt)
    residuals = I_nA - I_fitted_nA
    ss_res = float(np.sum(residuals ** 2))
    ss_tot = float(np.sum((I_nA - np.mean(I_nA)) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan

    return {
        "I_sat_nA": abs(float(I_sat_nA)),
        "a_per_V": abs(float(a)),
        "V0_V": float(V0),
        "I_offset_nA": float(I_offset_nA),
        "Te_eV": float(Te_eV),
        "Te_K": float(Te_K),
        "r_squared": float(r_squared),
        "V": V,
        "I_fitted_nA": I_fitted_nA,
        "covariance": pcov,
    }


def calculate_electron_density(
    I_sat_nA,
    Te_K,
    probe_area_m2=1.94e-3,
    ion_mass_kg=2.656e-26
):
    """Calculate n_e from fitted ion saturation current using Bohm collection."""
    if probe_area_m2 <= 0 or ion_mass_kg <= 0 or Te_K <= 0:
        raise ValueError("Probe area, ion mass, and electron temperature must be positive")

    I_sat_A = abs(float(I_sat_nA)) * 1e-9
    return (
        I_sat_A
        / (BOHM_COEFFICIENT * E_CHARGE * probe_area_m2)
        * np.sqrt(ion_mass_kg / (K_B * Te_K))
    )


# -----------------------------------------------------------------------------
# _extract_te_ne(sweeps, probe_area, ion_mass)
# -----------------------------------------------------------------------------
# Purpose:
#   Estimate electron temperature and electron density for every sweep by fitting
#   each corrected I-V curve to the dual-probe tanh model.
#
# Tanh model:
#   I(V) = Ioff + I0 * tanh(a * (V - V0))
#
# Electron temperature equation from tanh steepness:
#   Te = e / (2 * k * |a|)
#
# Electron density equation used here:
#   ne = |I0| * 1e-9 / (0.61 * e * A_probe) * sqrt(m_i / (k * Te))
# where:
#   |I0| * 1e-9 converts nA to A,
#   A_probe = probe_area,
#   m_i = ion_mass.
#
# Sweep midpoint time equation:
#   t = (tInitial + tFinal) / 2000
# because tInitial/tFinal are in milliseconds.
#
# Output:
#   arrays of time, Te, and ne for all successfully fitted sweeps.
def _extract_te_ne(sweeps, probe_area, ion_mass):

    times = []
    Te_list = []
    ne_list = []
    fit_rows = []

    for s in sweeps:

        V = np.asarray(s["V"], dtype=np.float64)
        I = np.asarray(s["I"], dtype=np.float64)

        V, I = correct_hysteresis(V, I)

        try:
            fit = fit_double_probe_curve(V, I)
            Te = fit["Te_K"]
            ne = calculate_electron_density(
                fit["I_sat_nA"],
                Te,
                probe_area_m2=probe_area,
                ion_mass_kg=ion_mass
            )

            t = (s["tInitial"] + s["tFinal"]) / 2000.0

            times.append(t)
            Te_list.append(Te)
            ne_list.append(ne)

            fit_rows.append({
                "packet_num": s["packet_num"],
                "time_s": t,
                "I_sat_nA": fit["I_sat_nA"],
                "a_per_V": fit["a_per_V"],
                "V0_V": fit["V0_V"],
                "I_offset_nA": fit["I_offset_nA"],
                "Te_eV": fit["Te_eV"],
                "Te_K": Te,
                "ne_m3": ne,
                "r_squared": fit["r_squared"],
                "fit_status": "success",
            })

        except Exception as ex:
            print(f"[FIT FAIL] sweep {s['packet_num']}: {ex}")

    global _LAST_PLASMA_FITS
    _LAST_PLASMA_FITS = pd.DataFrame(fit_rows)

    return np.array(times), np.array(Te_list), np.array(ne_list)

# Cache dictionary for expensive Te/ne extraction results.
# The nonlinear tanh fit can be slow because it runs once per sweep packet, so
# the GUI stores the most recent computed result and reuses it when possible.
_TE_NE_CACHE = {}
_LAST_PLASMA_FITS = pd.DataFrame()


def get_plasma_fit_results():
    """Return fitted I_sat, T_e, n_e, offsets, and R-squared for each sweep."""
    return _LAST_PLASMA_FITS.copy()



# -----------------------------------------------------------------------------
# _te_ne_cache_key(sweeps, probe_area, ion_mass)
# -----------------------------------------------------------------------------
# Purpose:
#   Build a tuple that uniquely identifies the current sweep data and calibration
#   settings. If any important input changes, the tuple changes and the cache is
#   recomputed.
#
# Key includes:
#   - identity and length of the sweep list
#   - first and last packet numbers
#   - probe area and ion mass
#   - calibration values from CAL
def _te_ne_cache_key(sweeps, probe_area, ion_mass):
    """
    Create a cache key for the current sweep data and calibration settings.

    If calibration changes or sweep data is reloaded, the key changes,
    so Te/ne will be recalculated.
    """

    if not sweeps:
        return None

    return (
        id(sweeps),
        len(sweeps),
        sweeps[0]["packet_num"],
        sweeps[-1]["packet_num"],
        float(probe_area),
        float(ion_mass),
        float(CAL.shunt_resistance),
        float(CAL.current_gain),
        float(CAL.voltage_gain),
        float(CAL.adc_vref),
    )



# -----------------------------------------------------------------------------
# _get_te_ne_cached(sweeps, probe_area, ion_mass, force_recompute)
# -----------------------------------------------------------------------------
# Purpose:
#   Return cached electron temperature/density arrays if available; otherwise run
#   _extract_te_ne() and store the result.
#
# Speed reason:
#   Without caching, every Te/ne plot button would refit every sweep.
#   With caching, the first Te/ne graph may be slow, but later Te/ne/time/altitude
#   graphs reuse the same arrays.
#
# Output:
#   t, Te, ne arrays.
def _get_te_ne_cached(
    sweeps,
    probe_area=1.94e-3,
    ion_mass=2.656e-26,
    force_recompute=False
):
    """
    Get electron temperature and density results.

    First call:
        runs the expensive tanh fits for all sweeps

    Later calls:
        reuses the saved result, so plots generate much faster
    """

    key = _te_ne_cache_key(sweeps, probe_area, ion_mass)

    if key is None:
        return np.array([]), np.array([]), np.array([])

    if not force_recompute and key in _TE_NE_CACHE:
        print("[CACHE] Using cached Te/ne results")
        return _TE_NE_CACHE[key]

    print("[CACHE] Computing Te/ne results...")

    result = _extract_te_ne(
        sweeps,
        probe_area=probe_area,
        ion_mass=ion_mass
    )

    # Keep only the most recent result to avoid memory buildup
    _TE_NE_CACHE.clear()
    _TE_NE_CACHE[key] = result

    return result


# -----------------------------------------------------------------------------
# plot_te_time(canvas, sweeps)
# -----------------------------------------------------------------------------
# Purpose:
#   Plot electron temperature versus experiment time using cached Te results.
#
# Data source:
#   t, Te, ne = _get_te_ne_cached(...)
#
# Electron temperature equation used upstream:
#   Te = e / (2 * k * |a|)
#
# Axes:
#   x-axis: time in seconds
#   y-axis: electron temperature in Kelvin
def plot_te_time(canvas, sweeps):

    if not sweeps:
        return

    t, Te, _ = _get_te_ne_cached(
        sweeps,
        probe_area=1.94e-3,
        ion_mass=2.656e-26
)

    ax = canvas.ax
    ax.clear()

    # Scatter only: do not connect unrelated consecutive measurements.
    ax.scatter(t, Te, s=14)

    # Fixed mission-wide axes.
    ax.set_xlim(*PLASMA_TIME_LIMITS_S)
    ax.set_ylim(*ELECTRON_TEMPERATURE_LIMITS_K)

    ax.set_title("Electron Temperature vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Te (K)")
    ax.grid(True, linestyle="--", alpha=0.5)

    canvas.draw()


# -----------------------------------------------------------------------------
# plot_ne_time(canvas, sweeps)
# -----------------------------------------------------------------------------
# Purpose:
#   Plot electron density versus experiment time using cached ne results.
#
# Electron density equation used upstream:
#   ne = |I0| * 1e-9 / (0.61 * e * A_probe) * sqrt(m_i / (k * Te))
#
# Axes:
#   x-axis: time in seconds
#   y-axis: electron density in m^-3
def plot_ne_time(canvas, sweeps):

    if not sweeps:
        return

    t, _, ne = _get_te_ne_cached(
        sweeps,
        probe_area=1.94e-3,
        ion_mass=2.656e-26
)

    ax = canvas.ax
    ax.clear()

    # Scatter only: do not connect unrelated consecutive measurements.
    ax.scatter(t, ne, s=14, color='red')

    # Fixed mission-wide axes.
    ax.set_xlim(*PLASMA_TIME_LIMITS_S)
    ax.set_ylim(*ELECTRON_DENSITY_LIMITS_M3)

    ax.set_title("Electron Density vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("ne (m^-3)")
    ax.grid(True, linestyle="--", alpha=0.5)

    canvas.draw()   


# -----------------------------------------------------------------------------
# _clean_trajectory(time_s, altitude_km)
# -----------------------------------------------------------------------------
# Purpose:
#   Prepare the flight trajectory table for interpolation.
#
# Steps:
#   1. Put time and altitude arrays into a DataFrame.
#   2. Remove NaN/inf rows.
#   3. Average duplicate time values.
#   4. Sort by time.
#
# Reason:
#   np.interp requires the x-values, here time_s, to be ordered. Duplicate time
#   entries are reduced to one altitude by averaging.
#
# Output:
#   clean trajectory time array and clean altitude array.
def _clean_trajectory(time_s, altitude_km):
    """
    Clean the flight trajectory before interpolation.

    The trajectory has repeated time values, such as 0.1 seconds.
    np.interp expects time values to be sorted, so this function:
        - removes invalid values
        - averages duplicate time values
        - sorts the trajectory by time
    """

    traj = pd.DataFrame({
        "time_s": time_s,
        "altitude_km": altitude_km
    })

    traj = traj.replace([np.inf, -np.inf], np.nan).dropna()

    traj = (
        traj
        .groupby("time_s", as_index=False)
        .agg(altitude_km=("altitude_km", "mean"))
        .sort_values("time_s")
    )

    return (
        traj["time_s"].to_numpy(dtype=np.float64),
        traj["altitude_km"].to_numpy(dtype=np.float64)
    )


# -----------------------------------------------------------------------------
# _add_ionosphere_layers(ax)
# -----------------------------------------------------------------------------
# Purpose:
#   Draw colored vertical background bands for ionosphere/atmosphere regions on
#   altitude-based graphs.
#
# Assumption:
#   Altitude is on the x-axis.
#
# Plot operation:
#   ax.axvspan(left, right, color=color, alpha=0.18, zorder=0)
#
# Axis protection:
#   The function saves and restores the original x/y limits so adding shaded
#   regions does not change the data scaling.
def _add_ionosphere_layers(ax):
    """
    Add colored ionosphere regions to altitude-based plots.

    Assumes altitude is on the x-axis.
    """

    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()

    y_text = ymin + 0.95 * (ymax - ymin)

    for label, low, high, color in IONOSPHERE_LAYERS:
        left = max(low, xmin)
        right = min(high, xmax)

        if right <= left:
            continue

        ax.axvspan(left, right, color=color, alpha=0.18, zorder=0)

        ax.text(
            (left + right) / 2,
            y_text,
            label,
            ha="center",
            va="top",
            fontsize=8,
            color="black"
        )

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)


# -----------------------------------------------------------------------------
# _altitude_at_sweep_times(t, time_offset_s)
# -----------------------------------------------------------------------------
# Purpose:
#   Convert sweep midpoint experiment times into flight altitudes.
#
# Timer relationship:
#   The experiment timer starts 3 minutes before flight, so:
#     flight_time_s = experiment_time_s - 180
#
# General equation used by the function:
#   flight_time_s = t - time_offset_s
#
# Interpolation equation:
#   altitude_at_sweeps = np.interp(flight_time_s, TRAJ_TIME_S, TRAJ_ALT_KM)
#
# Output:
#   altitude_at_sweeps: altitude values in km for valid sweeps
#   valid: boolean mask for sweeps inside the trajectory time range
#   flight_time_s: converted flight-time array
def _altitude_at_sweep_times(t, time_offset_s=180.0):
    """
    Convert sweep times into flight altitudes.

    Your experiment timer starts 3 minutes before flight, so:

        flight_time = experiment_time - 180

    Parameters
    ----------
    t:
        Sweep midpoint times in seconds from the experiment timer.

    time_offset_s:
        Time difference between experiment timer and flight timer.
        Default is 180 seconds because the experiment starts 3 minutes early.

    Returns
    -------
    altitude_at_sweeps:
        Interpolated altitude in km for each valid sweep.

    valid:
        Boolean mask showing which sweeps fall inside the known trajectory range.

    flight_time_s:
        Sweep times converted from experiment time to flight time.
    """

    traj_time_s, traj_alt_km = _clean_trajectory(
        TRAJ_TIME_S,
        TRAJ_ALT_KM
    )

    t = np.asarray(t, dtype=np.float64)

    # Convert experiment timer time to flight timer time
    flight_time_s = t - time_offset_s

    valid = (
        np.isfinite(flight_time_s)
        & (flight_time_s >= traj_time_s[0])
        & (flight_time_s <= traj_time_s[-1])
    )

    if np.count_nonzero(valid) == 0:
        return np.array([]), valid, flight_time_s

    altitude_at_sweeps = np.interp(
        flight_time_s[valid],
        traj_time_s,
        traj_alt_km
    )

    return altitude_at_sweeps, valid, flight_time_s


def calculate_plasma_parameters(
    sweeps,
    probe_area_m2=1.94e-3,
    ion_mass_kg=2.656e-26,
    time_offset_s=180.0,
    force_recompute=False,
):
    """Calculate the physical plasma result associated with every fitted sweep.

    Processing chain for each complete sweep:

        raw ADC values (already converted by load_data)
        -> voltage-aligned forward/reverse hysteresis averaging
        -> symmetric double-probe tanh fit
        -> T_e(eV) = 1 / (2|a|)
        -> T_e(K) = T_e(eV) * 11604.51812
        -> n_e = I_sat / [0.61 e A sqrt(k_B T_e / m_i)]
        -> sweep midpoint time
        -> interpolated rocket altitude

    This function does not replace I_sat with a current percentile and does not
    apply temperature, density, percentile, or fit-quality rejection filters.
    Sweeps for which the nonlinear fit mathematically fails are reported by
    _extract_te_ne and cannot produce T_e or n_e values.

    Returns
    -------
    pandas.DataFrame
        One row per successfully fitted sweep. Altitude is NaN when the sweep
        time lies outside the supplied trajectory time range.
    """
    if not sweeps:
        return pd.DataFrame(columns=[
            "packet_num", "time_s", "flight_time_s", "altitude_km",
            "I_sat_nA", "a_per_V", "V0_V", "I_offset_nA",
            "Te_eV", "Te_K", "ne_m3", "r_squared", "fit_status",
        ])

    t, _, _ = _get_te_ne_cached(
        sweeps,
        probe_area=probe_area_m2,
        ion_mass=ion_mass_kg,
        force_recompute=force_recompute,
    )

    results = get_plasma_fit_results()
    if results.empty or len(t) == 0:
        return results

    altitude_km, valid, flight_time_s = _altitude_at_sweep_times(
        t,
        time_offset_s=time_offset_s,
    )

    results = results.copy().reset_index(drop=True)
    results["flight_time_s"] = flight_time_s
    results["altitude_km"] = np.nan
    results.loc[valid, "altitude_km"] = altitude_km

    ordered_columns = [
        "packet_num", "time_s", "flight_time_s", "altitude_km",
        "I_sat_nA", "a_per_V", "V0_V", "I_offset_nA",
        "Te_eV", "Te_K", "ne_m3", "r_squared", "fit_status",
    ]
    return results[ordered_columns]



# -----------------------------------------------------------------------------
# plot_te_altitude(canvas, sweeps, time_offset_s)
# -----------------------------------------------------------------------------
# Purpose:
#   Plot electron temperature versus interpolated rocket altitude.
#
# Data flow:
#   sweep time -> flight time -> trajectory interpolation -> altitude -> plot Te
#
# Equations used upstream:
#   flight_time_s = experiment_time_s - time_offset_s
#   altitude_km = interp(flight_time_s, TRAJ_TIME_S, TRAJ_ALT_KM)
#   Te = e / (2 * k * |a|)
#
# Display features:
#   - x-axis starts at 0 km
#   - colored ionosphere/atmosphere region bands are drawn behind the data
def plot_te_altitude(canvas, sweeps, time_offset_s=180.0):
    """
    Plot electron temperature versus altitude.

    This uses:
        sweep time -> flight time -> interpolated altitude -> electron temperature

    Because the experiment timer starts 3 minutes before flight,
    the default time offset is 180 seconds.
    """

    if not sweeps:
        return

    plasma = calculate_plasma_parameters(
        sweeps,
        probe_area_m2=1.94e-3,
        ion_mass_kg=2.656e-26,
        time_offset_s=time_offset_s,
    )

    if plasma.empty:
        return

    valid = np.isfinite(plasma["altitude_km"]) & np.isfinite(plasma["Te_K"])
    altitude_km = plasma.loc[valid, "altitude_km"].to_numpy()
    Te_valid = plasma.loc[valid, "Te_K"].to_numpy()

    if len(altitude_km) == 0:
        print("[WARNING] No sweep times fall inside the trajectory time range.")
        return

    ax = canvas.ax
    ax.clear()

    # Scatter only: ascent/descent points at the same altitude are not joined.
    ax.scatter(altitude_km, Te_valid, s=14, zorder=2)

    # Fixed mission-wide axes.
    ax.set_xlim(*ALTITUDE_LIMITS_KM)
    ax.set_ylim(*ELECTRON_TEMPERATURE_LIMITS_K)

    _add_ionosphere_layers(ax)

    ax.set_title("Electron Temperature vs Altitude")
    ax.set_xlabel("Altitude (km)")
    ax.set_ylabel("Te (K)")
    ax.grid(True, linestyle="--", alpha=0.5)

    canvas.draw()



# -----------------------------------------------------------------------------
# plot_ne_altitude(canvas, sweeps, time_offset_s)
# -----------------------------------------------------------------------------
# Purpose:
#   Plot electron density versus interpolated rocket altitude.
#
# Data flow:
#   sweep time -> flight time -> trajectory interpolation -> altitude -> plot ne
#
# Equations used upstream:
#   flight_time_s = experiment_time_s - time_offset_s
#   altitude_km = interp(flight_time_s, TRAJ_TIME_S, TRAJ_ALT_KM)
#   ne = |I0| * 1e-9 / (0.61 * e * A_probe) * sqrt(m_i / (k * Te))
#
# Display features:
#   - x-axis starts at 0 km
#   - colored ionosphere/atmosphere region bands are drawn behind the data
#   - y-axis uses scientific notation for density
def plot_ne_altitude(canvas, sweeps, time_offset_s=180.0):
    """
    Plot electron density versus altitude.

    This uses:
        sweep time -> flight time -> interpolated altitude -> electron density

    Because the experiment timer starts 3 minutes before flight,
    the default time offset is 180 seconds.
    """

    if not sweeps:
        return

    plasma = calculate_plasma_parameters(
        sweeps,
        probe_area_m2=1.94e-3,
        ion_mass_kg=2.656e-26,
        time_offset_s=time_offset_s,
    )

    if plasma.empty:
        return

    valid = np.isfinite(plasma["altitude_km"]) & np.isfinite(plasma["ne_m3"])
    altitude_km = plasma.loc[valid, "altitude_km"].to_numpy()
    ne_valid = plasma.loc[valid, "ne_m3"].to_numpy()

    if len(altitude_km) == 0:
        print("[WARNING] No sweep times fall inside the trajectory time range.")
        return

    ax = canvas.ax
    ax.clear()

    # Scatter only: ascent/descent points at the same altitude are not joined.
    ax.scatter(altitude_km, ne_valid, s=14, color='red', zorder=2)

    # Fixed mission-wide axes.
    ax.set_xlim(*ALTITUDE_LIMITS_KM)
    ax.set_ylim(*ELECTRON_DENSITY_LIMITS_M3)

    _add_ionosphere_layers(ax)

    ax.set_title("Electron Density vs Altitude")
    ax.set_xlabel("Altitude (km)")
    ax.set_ylabel("ne (m^-3)")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))

    canvas.draw()
