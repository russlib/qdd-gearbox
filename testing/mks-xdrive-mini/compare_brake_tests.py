"""Side-by-side overlay of RPM-vs-time curves from all brake-disable tests.

Goal: visually compare every test we've run against the H2 to see whether any
mode produced a meaningfully different acceleration curve. Generates two
figures:

  1. all_brake_tests_rpm.png   — |RPM| vs time, every test as one line.
  2. all_brake_tests_grid.png  — small-multiples grid: one panel per test,
                                  RPM + Iq + FET on each panel.

Reads thermal_data/*.csv and groups by configuration label.
"""
from __future__ import annotations
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DATA = Path(__file__).parent / "thermal_data"
OUT  = DATA

# Map filename prefix -> short label + group + color
TESTS = [
    ("brake_disable_none_20260507-175721",       "T0 baseline (no BLE, 30A)",     "30A no-BLE",      "#888888"),
    ("brake_disable_warmup_20260507-180209",     "T1 WARM_UP (30A)",              "30A BLE-mode",    "#1f77b4"),
    ("brake_disable_headless_20260507-180308",   "T3 HEADLESS (30A)",             "30A BLE-mode",    "#ff7f0e"),
    ("brake_disable_power-range-0_20260507-180350","T4 POWER_RANGE 0,0 (30A)",    "30A BLE-mode",    "#2ca02c"),
    ("brake_disable_rolldown_mr_20260507-180432","T2 ROLL_DOWN multi-ramp (30A)", "30A BLE-mode",    "#d62728"),
    ("brake_disable_warmup_disc_20260507-180607","T5 WARM_UP+disc (30A)",         "30A BLE-mode",    "#9467bd"),
    ("brake_disable_sim-flat_20260507-180747",   "SIM_FLAT (30A)",                "30A BLE-mode",    "#8c564b"),
    ("rapid_brake_cycle_20260507-183055",        "rapid cycle (45A)",             "45A cycle",       "#e377c2"),
    ("brake_disable_none_20260507-185535",       "post-rolldown none (45A)",      "45A no-BLE",      "#17becf"),
    ("rolldown_then_spin_20260507-185818",       "rolldown_then_spin v1 (45A)",   "45A rolldown",    "#bcbd22"),
    ("rolldown_then_spin_20260507-190017",       "rolldown_then_spin v2-HEADLESS confirmed (45A)",
                                                                                  "45A rolldown",    "#7f7f7f"),
]


def load_csv(path: Path):
    """Return arrays (t, vel_t_s, iq, fet, phase_labels)."""
    t, vel, iq, fet = [], [], [], []
    phases = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t.append(float(row["t_s"]))
                vel.append(float(row["vel_t_s"]))
                iq.append(float(row["iq_a"]))
                fet.append(float(row["fet_c"]))
                phases.append(row.get("phase", ""))
            except (ValueError, KeyError):
                continue
    return (np.array(t), np.array(vel), np.array(iq), np.array(fet), phases)


def overlay_rpm():
    fig, ax = plt.subplots(figsize=(14, 8))
    for prefix, label, group, color in TESTS:
        path = DATA / f"{prefix}.csv"
        if not path.exists():
            print(f"  missing: {path.name}")
            continue
        t, vel, iq, fet, _ = load_csv(path)
        if len(t) == 0:
            continue
        rpm = np.abs(vel) * 60.0
        # Linestyle by group (same group shares style)
        ls = "-"
        if "no-BLE" in group: ls = "--"
        if "rolldown" in group: ls = "-."
        ax.plot(t, rpm, color=color, label=label, linestyle=ls, linewidth=1.6, alpha=0.85)

    ax.set_xlabel("time [s]")
    ax.set_ylabel("|motor RPM|")
    ax.set_title("Saris H2 brake-disable tests — motor RPM vs time, 2026-05-06/07")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    ax.axhline(50, color="red", linestyle=":", linewidth=0.8, alpha=0.6)
    ax.text(0.5, 52, "braked ceiling ~50 RPM", color="red", fontsize=8, alpha=0.8)
    fig.tight_layout()
    out = OUT / "all_brake_tests_rpm.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


def small_multiples():
    n = len(TESTS)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(15, 3.4 * rows), sharex=False)
    axes = axes.flatten()

    for i, (prefix, label, group, color) in enumerate(TESTS):
        ax = axes[i]
        path = DATA / f"{prefix}.csv"
        if not path.exists():
            ax.set_title(f"{label}\n[missing]", fontsize=9)
            ax.axis("off")
            continue
        t, vel, iq, fet, phases = load_csv(path)
        if len(t) == 0:
            ax.axis("off"); continue
        rpm = np.abs(vel) * 60.0

        ax2 = ax.twinx()
        ax3 = ax.twinx()
        ax3.spines["right"].set_position(("outward", 38))

        ax.plot(t, rpm, color=color, label="RPM", linewidth=1.6)
        ax2.plot(t, iq, color="#888888", linestyle=":", linewidth=1.0, label="Iq")
        ax3.plot(t, fet, color="red", linestyle="--", linewidth=0.8, alpha=0.6, label="FET")

        # Phase shading
        if len(phases):
            cur_phase = None; cur_start = None
            phase_colors = {"ramp": "#fffacd", "hold": "#ffe4b5",
                            "settle": "#e0f0ff", "observe": "#e8e8e8"}
            for k, ph in enumerate(phases + [""]):
                if ph != cur_phase:
                    if cur_phase is not None and cur_phase in phase_colors:
                        end_t = t[k-1] if k <= len(t) else t[-1]
                        ax.axvspan(cur_start, end_t, color=phase_colors[cur_phase], alpha=0.4)
                    cur_phase = ph
                    cur_start = t[k] if k < len(t) else t[-1]

        peak = rpm.max() if len(rpm) else 0
        ax.set_title(f"{label}\npeak={peak:.0f} RPM", fontsize=9)
        ax.set_xlabel("t [s]", fontsize=8)
        ax.set_ylabel("|RPM|", color=color, fontsize=8)
        ax2.set_ylabel("Iq [A]", color="#888888", fontsize=8)
        ax3.set_ylabel("FET [°C]", color="red", fontsize=8)
        ax.tick_params(axis="both", labelsize=7)
        ax2.tick_params(axis="y", labelsize=7)
        ax3.tick_params(axis="y", labelsize=7)
        ax.grid(True, alpha=0.3)
        ax.axhline(50, color="red", linestyle=":", linewidth=0.6, alpha=0.4)

    # Hide any unused subplots
    for j in range(n, len(axes)):
        axes[j].axis("off")

    fig.suptitle("Saris H2 brake-disable test suite — small multiples (RPM, Iq, FET)", fontsize=12)
    fig.tight_layout()
    out = OUT / "all_brake_tests_grid.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


def main():
    print("Building comparison plots...")
    p1 = overlay_rpm()
    print(f"  -> {p1}")
    p2 = small_multiples()
    print(f"  -> {p2}")
    print("Done.")


if __name__ == "__main__":
    main()
