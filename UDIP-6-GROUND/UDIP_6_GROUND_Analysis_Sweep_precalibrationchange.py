from pathlib import Path
import pandas as pd
import numpy as np
from calibration import CAL

N_SWP_STEP = 356

ADC_RES = 4095.0
ADC_VREF = 3.3

BASE_DIR = Path(__file__).resolve().parent


def _get_sweep_csv(dat_number: str) -> Path:
    return BASE_DIR / "CSV-DATA" / "SWEEP" / f"sweep_packets_{dat_number}.csv"


def dac_to_voltage(vrefA, vrefB):
    vrefA = np.asarray(vrefA, dtype=np.float64)
    vrefB = np.asarray(vrefB, dtype=np.float64)

    vx1 = (vrefA / ADC_RES) * ADC_VREF
    vx2 = (vrefB / ADC_RES) * ADC_VREF

    return (vx2 - vx1) * 16


def adc_to_current(adcA, adcB):
    adcA = np.asarray(adcA, dtype=np.float64)
    adcB = np.asarray(adcB, dtype=np.float64)

    vA = (adcA / ADC_RES) * ADC_VREF
    vB = (adcB / ADC_RES) * ADC_VREF

    v_diff = (vA - vB) * 4

    return (v_diff / 100000000.0) * 1e9


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


def plot_iv_curve(canvas, sweeps: list[dict], sweep_idx: int = 0):
    if not sweeps or sweep_idx >= len(sweeps):
        return

    s = sweeps[sweep_idx]

    V = np.array(s["V"], dtype=np.float64)
    I = np.array(s["I"], dtype=np.float64)

    mask = np.isfinite(V) & np.isfinite(I)
    V = V[mask]
    I = I[mask]

    ax = canvas.ax
    ax.clear()

    ax.plot(V, I, linestyle=":", linewidth=1, color='gray', alpha=0.7)
    ax.scatter(V, I, s=10, c='blue')

    ax.set_xlim(-25, 25)
    ax.set_ylim(-250, 250)

    ax.axhline(0, color='black', linewidth=0.6)
    ax.axvline(0, color='black', linewidth=0.6)

    # Title now reflects both the Global Packet ID and Hardware Count
    t0 = s["tInitial"] / 1000
    t1 = s["tFinal"] / 1000

    ax.set_title(
    f"I–V Curve (ID: {s['packet_num']} | "
    f"Time: {t0:.3f}–{t1:.3f})"
)
    ax.set_xlabel("Voltage (V)")
    ax.set_ylabel("Current (nA)")
    ax.grid(True, linestyle="--", alpha=0.5)

    canvas.draw()
