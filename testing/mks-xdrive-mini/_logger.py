"""Shared test logger for the MKS Mini bench scripts.

Writes a row-per-sample CSV and a standard 4-panel plot:
- FET temp (C)
- Iq (A)
- velocity (motor RPM)
- position (motor turns)

Output: thermal_data/<test_name>_<YYYYmmdd-HHMMSS>.{csv,png}
"""
from __future__ import annotations
import csv
import math
import time
from pathlib import Path

OUT_DIR = Path(__file__).parent / "thermal_data"
OUT_DIR.mkdir(exist_ok=True)


class TestLogger:
    """Drop-in CSV+plot helper.

    Usage:
        logger = TestLogger("45a_loaded")
        logger.row(t=t, phase="hold", cmd_nm=..., iq=..., vel_t_s=..., pos_turns=..., fet_c=...)
        ...
        logger.close()
        logger.plot(title="...", subtitle="...")
        logger.fingerprint  # dict of paths
    """

    DEFAULT_FIELDS = [
        "t_s", "phase", "cmd_nm", "iq_a", "vel_t_s", "pos_turns",
        "fet_c", "axis_err", "motor_err", "encoder_err", "extra"
    ]

    def __init__(self, test_name: str, ts: str | None = None, extra_fields: list[str] | None = None):
        self.test_name = test_name
        self.ts = ts or time.strftime("%Y%m%d-%H%M%S")
        self.csv_path = OUT_DIR / f"{test_name}_{self.ts}.csv"
        self.png_path = OUT_DIR / f"{test_name}_{self.ts}.png"
        self.fields = list(self.DEFAULT_FIELDS)
        if extra_fields:
            self.fields.extend(extra_fields)
        self._file = open(self.csv_path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fields)
        self._writer.writeheader()
        self._closed = False
        self._row_count = 0

    def row(self, **kwargs):
        """Write a row. Missing fields = empty string."""
        out = {k: kwargs.get(k, "") for k in self.fields}
        # numeric formatting: keep 4 sig figs for floats
        for k, v in list(out.items()):
            if isinstance(v, float):
                if math.isnan(v):
                    out[k] = ""
                else:
                    out[k] = f"{v:.4f}"
            elif v is None:
                out[k] = ""
        self._writer.writerow(out)
        self._row_count += 1

    def close(self):
        if not self._closed:
            self._file.close()
            self._closed = True
        return self.csv_path

    def __enter__(self): return self
    def __exit__(self, *a): self.close()

    @property
    def fingerprint(self):
        return {"csv": str(self.csv_path), "png": str(self.png_path), "rows": self._row_count}

    def plot(self, title: str = "", subtitle: str = "", show_phases: bool = True):
        """Standard 4-panel plot from the saved CSV."""
        if not self._closed:
            self.close()
        try:
            import numpy as np
            import matplotlib.pyplot as plt
        except Exception as e:
            print(f"  plot skipped — matplotlib import failed: {e}")
            return None

        rows = []
        with open(self.csv_path, newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                rows.append(row)
        if not rows:
            print("  plot skipped — no rows")
            return None

        def col(name):
            out = []
            for r in rows:
                v = r.get(name, "")
                try:
                    out.append(float(v))
                except (TypeError, ValueError):
                    out.append(math.nan)
            return np.array(out)

        t = col("t_s")
        fet = col("fet_c")
        iq = col("iq_a")
        vel = col("vel_t_s")
        pos = col("pos_turns")
        phases = [r.get("phase", "") for r in rows]

        rpm = vel * 60.0  # turns/sec -> RPM

        fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
        full_title = title if title else self.test_name
        if subtitle:
            full_title = f"{full_title}\n{subtitle}"
        fig.suptitle(full_title, fontsize=12)

        phase_colors = {"ramp": "#fffacd", "hold": "#ffe4b5", "cool": "#e0f0ff",
                        "idle": "#f0f0f0", "burst": "#ffd6e0"}

        def shade_phases(ax):
            if not show_phases or len(t) == 0:
                return
            current_phase = None
            start_t = None
            for i, p in enumerate(phases):
                if p != current_phase:
                    if current_phase is not None and start_t is not None:
                        c = phase_colors.get(current_phase)
                        if c:
                            ax.axvspan(start_t, t[i], color=c, alpha=0.5)
                    current_phase = p
                    start_t = t[i]
            if current_phase is not None and start_t is not None:
                c = phase_colors.get(current_phase)
                if c:
                    ax.axvspan(start_t, t[-1], color=c, alpha=0.5)

        ax = axs[0]; shade_phases(ax)
        ax.plot(t, fet, color="#c44", lw=1.5)
        ax.set_ylabel("FET temp (°C)")
        ax.grid(True, alpha=0.3)

        ax = axs[1]; shade_phases(ax)
        ax.plot(t, iq, color="#26a", lw=1.0)
        ax.set_ylabel("Iq (A)")
        ax.grid(True, alpha=0.3)

        ax = axs[2]; shade_phases(ax)
        ax.plot(t, rpm, color="#2a8", lw=1.0)
        ax.set_ylabel("Velocity (motor RPM)")
        ax.grid(True, alpha=0.3)

        ax = axs[3]; shade_phases(ax)
        ax.plot(t, pos, color="#84a", lw=1.0)
        ax.set_ylabel("Position (motor turns)")
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)

        # phase legend
        unique_phases = []
        for p in phases:
            if p and p not in unique_phases:
                unique_phases.append(p)
        if unique_phases and show_phases:
            from matplotlib.patches import Patch
            handles = [Patch(facecolor=phase_colors.get(p, "#ccc"), alpha=0.5, label=p)
                       for p in unique_phases if p in phase_colors]
            if handles:
                axs[0].legend(handles=handles, loc="upper right", fontsize=8)

        plt.tight_layout()
        plt.subplots_adjust(top=0.93)
        plt.savefig(self.png_path, dpi=130)
        plt.close(fig)
        return self.png_path
