from pathlib import Path
import csv
import numpy as np

# ─────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────
N_SWP_STEP = 356

ZERO_VOLT_DAC = 1737
DAC_MAX       = 2693
V_MAX         = 8.0

ADC_RES  = 4095.0
ADC_VREF = 5.0

SHUNT_OHM = 100000000  # 100 MΩ resistor


# ─────────────────────────────────────────
# PATH
# ─────────────────────────────────────────
BASE_DIR = Path(__file__).resolve().parent

def _get_sweep_csv(dat_number: str) -> Path:
    return BASE_DIR / "CSV-DATA" / "SWEEP" / f"sweep_packets_{dat_number}.csv"


# ─────────────────────────────────────────
# LOAD DATA
# ─────────────────────────────────────────
def load_data(dat_number: str = "0001") -> list[dict]:
    csv_path = _get_sweep_csv(dat_number)
    sweeps_raw = {}

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            cnt = int(row["count"])

            if cnt not in sweeps_raw:
                sweeps_raw[cnt] = {
                    "count": cnt,
                    "step": [],
                    "v_ref": [],
                    "adc_A": [],
                    "adc_B": [],
                }

            sweeps_raw[cnt]["step"].append(int(row["step"]))
            sweeps_raw[cnt]["v_ref"].append(int(row["v_ref"]))
            sweeps_raw[cnt]["adc_A"].append(int(row["adc_A"]))
            sweeps_raw[cnt]["adc_B"].append(int(row["adc_B"]))

    sweeps = []

    for cnt in sorted(sweeps_raw):
        s = sweeps_raw[cnt]

        v_ref = np.array(s["v_ref"], dtype=np.float64)
        adc_A = np.array(s["adc_A"], dtype=np.float64)
        adc_B = np.array(s["adc_B"], dtype=np.float64)

        v_ref = v_ref[:N_SWP_STEP]
        adc_A = adc_A[:N_SWP_STEP]
        adc_B = adc_B[:N_SWP_STEP]

        # ─────────────────────────────────────────
        # 1. REAL DAC VOLTAGE (LINEAR RESTORATION)
        # ─────────────────────────────────────────
        V = (v_ref - np.mean(v_ref)) * (V_MAX / (np.max(v_ref) - np.min(v_ref)))
        # ─────────────────────────────────────────
        # 2. REAL CURRENT FROM 100 MΩ SHUNT
        # ─────────────────────────────────────────
        V_adc_A = (adc_A / ADC_RES) * ADC_VREF
        V_adc_B = (adc_B / ADC_RES) * ADC_VREF

        I_A = (V_adc_A / SHUNT_OHM) * 1e9  # nA
        I_B = (V_adc_B / SHUNT_OHM) * 1e9  # nA

        # differential mode (if used)
        CURRENT_GAIN = 4.5
        I = -(I_A - I_B) * CURRENT_GAIN

        # IMPORTANT: DO NOT NORMALIZE

        sweeps.append({
            "count": cnt,
            "V": V,
            "I": I
        })

    return sweeps
# ─────────────────────────────────────────
# IV PLOT (FORCED DIAGONAL + DOTTED STYLE)
# ─────────────────────────────────────────
def plot_iv_curve(canvas, sweeps: list[dict], sweep_idx: int = 0):
    if not sweeps:
        return

    s = sweeps[sweep_idx]

    V = np.array(s["V"], dtype=np.float64)
    I = np.array(s["I"], dtype=np.float64)

    mask = np.isfinite(V) & np.isfinite(I)
    V = V[mask]
    I = I[mask]

    ax = canvas.ax
    ax.clear()

    ax.plot(V, I, linestyle=":", linewidth=1)
    ax.scatter(V, I, s=10)

    ax.set_xlim(-16, 16)
    ax.set_ylim(-160, 160)

    ax.axhline(0, linewidth=0.6)
    ax.axvline(0, linewidth=0.6)

    ax.set_title(f"I–V Curve (Sweep {s['count']}) [100MΩ resistor]")
    ax.set_xlabel("Voltage (V)")
    ax.set_ylabel("Current (nA)")
    ax.grid(True, linestyle="--", alpha=0.5)

    canvas.draw()
