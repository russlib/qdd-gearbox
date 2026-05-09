"""Overlay all four brake-drag sweeps on a single hysteresis plot.

Reads step results from each sweep's `_meta.json` and produces:
  thermal_data/brake_hysteresis_overlay.png

Sweeps included (auto-detected by filename):
  - brake_drag_steady_state_*  : original ascending 10..45 A (5 A steps)
  - brake_drag_reverse_sweep_* : descending 45..10 A (5 A steps)
  - brake_drag_knee_ascending_*: ascending 12..22 A (1 A steps)
  - brake_drag_knee_descending_*: descending 30..12 A (mixed steps)
"""
from __future__ import annotations
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = Path(__file__).parent / "thermal_data"
OUT_PNG = DATA_DIR / "brake_hysteresis_overlay.png"

# (filename glob, label, color, marker, direction-tag)
SWEEPS = [
    ("brake_drag_steady_state_20260508-202617_meta.json",
     "Ascending 10-45 A (5 A steps)", "#1f77b4", "o", "asc"),
    ("brake_drag_reverse_sweep_*_meta.json",
     "Descending 45-10 A (5 A steps)", "#d62728", "s", "desc"),
    ("brake_drag_knee_ascending_*_meta.json",
     "Knee ascending 12-22 A (1 A)", "#2ca02c", "^", "asc"),
    ("brake_drag_knee_descending_*_meta.json",
     "Knee descending 30-12 A (1 A)", "#ff7f0e", "v", "desc"),
]


def load_steps(glob_pattern):
    matches = sorted(DATA_DIR.glob(glob_pattern))
    if not matches:
        return None
    meta_path = matches[-1]  # most recent
    with open(meta_path) as f:
        meta = json.load(f)
    steps = meta.get("result", {}).get("steps", [])
    return steps, meta_path.name


def main():
    fig, axs = plt.subplots(1, 2, figsize=(13, 6))

    for glob_pat, label, color, marker, dir_tag in SWEEPS:
        out = load_steps(glob_pat)
        if not out:
            print(f"  skip (no match): {glob_pat}")
            continue
        steps, src = out
        if not steps:
            print(f"  skip (no steps): {src}")
            continue
        iq = [s["iq_a"] for s in steps]
        rpm = [s["rpm_eq"] for s in steps]
        torque = [s["torque_nm"] for s in steps]
        # Order points by sweep order (already in step order from JSON)
        axs[0].plot(iq, rpm, marker=marker, ms=7, lw=1.5, color=color, label=label, alpha=0.85)
        axs[1].plot(rpm, torque, marker=marker, ms=7, lw=1.5, color=color, label=label, alpha=0.85)
        print(f"  {label}: {len(steps)} steps from {src}")

    # IQ vs RPM
    axs[0].set_xlabel("Motor IQ command (A)")
    axs[0].set_ylabel("Equilibrium RPM (motor shaft)")
    axs[0].set_title("H2 brake — IQ vs equilibrium RPM\n(hysteresis between asc/desc paths)")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc="upper left", fontsize=9)
    axs[0].axhline(31, color="gray", lw=0.5, linestyle=":", alpha=0.5)
    axs[0].axvline(16.5, color="gray", lw=0.5, linestyle=":", alpha=0.5)
    axs[0].axvline(20, color="gray", lw=0.5, linestyle=":", alpha=0.5)

    # RPM vs T_drag
    axs[1].set_xlabel("Equilibrium RPM (motor shaft)")
    axs[1].set_ylabel("Motor torque @ equilibrium = T_drag (Nm)")
    axs[1].set_title("Operating points (RPM, T_drag)\nplateau at ~31 RPM spans 0.8-1.8 Nm")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="upper left", fontsize=9)

    fig.suptitle("Saris H2 brake — full hysteresis characterization (D6374 + MKS XDrive Mini, dir -1)",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=130)
    plt.close(fig)
    print(f"\n  PNG: {OUT_PNG}")


if __name__ == "__main__":
    main()
