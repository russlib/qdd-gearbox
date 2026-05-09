"""Shared test logger for the MKS Mini bench scripts.

Writes a row-per-sample CSV, a standard 4-panel plot, and a JSON metadata
sidecar capturing test conditions for future reanalysis.

- FET temp (C)
- Iq (A)
- velocity (motor RPM)
- position (motor turns)

Output: thermal_data/<test_name>_<YYYYmmdd-HHMMSS>.{csv,png,_meta.json}
"""
from __future__ import annotations
import csv
import json
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
        self.meta_path = OUT_DIR / f"{test_name}_{self.ts}_meta.json"
        self.fields = list(self.DEFAULT_FIELDS)
        if extra_fields:
            self.fields.extend(extra_fields)
        self._file = open(self.csv_path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fields)
        self._writer.writeheader()
        self._closed = False
        self._row_count = 0
        self._meta: dict = {
            "test_name": test_name,
            "timestamp": self.ts,
            "csv": self.csv_path.name,
            "png": self.png_path.name,
        }

    def set_meta(self, **kwargs):
        """Attach test conditions / hardware / result info to the metadata sidecar.

        Recommended keys:
          purpose:        one-line description
          hardware:       dict (motor, controller, encoder, trainer, ...)
          conditions:     dict (esc_fan, motor_fan, trainer_ac, direction, bus_v, ...)
          parameters:     dict (target_iq, hold_s, ramp_s, ...)
          result:         dict (peak_rpm, peak_fet, abort, verdict, ...)
        """
        self._meta.update(kwargs)

    def write_meta(self):
        """Persist metadata sidecar JSON. Idempotent — call any time after set_meta."""
        try:
            with open(self.meta_path, "w") as fh:
                json.dump(self._meta, fh, indent=2)
        except Exception as e:
            print(f"  meta write skipped: {e}")

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
            # Write meta sidecar on close so it always exists, even if no plot
            self._meta.setdefault("rows", self._row_count)
            self.write_meta()
        return self.csv_path

    def __enter__(self): return self
    def __exit__(self, *a): self.close()

    @property
    def fingerprint(self):
        return {"csv": str(self.csv_path), "png": str(self.png_path), "rows": self._row_count}

    @staticmethod
    def _conditions_subtitle(meta: dict) -> str:
        """Build a one-line condition-summary from meta['conditions']."""
        c = meta.get("conditions", {}) if isinstance(meta, dict) else {}
        parts = []
        if "esc_fan" in c: parts.append(f"ESC fan: {'on' if c['esc_fan'] else 'off'}")
        if "motor_fan" in c: parts.append(f"motor fan: {'on' if c['motor_fan'] else 'off'}")
        if "trainer_ac" in c: parts.append(f"trainer AC: {c['trainer_ac']}")
        if "trainer_ble_state" in c: parts.append(f"BLE: {c['trainer_ble_state']}")
        if "direction" in c: parts.append(f"dir {c['direction']:+d}")
        if "bus_v" in c: parts.append(f"bus {c['bus_v']:.2f}V")
        return " | ".join(parts)

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
        # Auto-add condition line from meta if subtitle not explicit
        cond_line = self._conditions_subtitle(self._meta)
        sub_lines = []
        if subtitle: sub_lines.append(subtitle)
        if cond_line: sub_lines.append(cond_line)
        if sub_lines:
            full_title = full_title + "\n" + "\n".join(sub_lines)
        fig.suptitle(full_title, fontsize=11)

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
        plt.subplots_adjust(top=0.90 if cond_line else 0.93)
        plt.savefig(self.png_path, dpi=130)
        plt.close(fig)
        # Update meta on plot
        self._meta["png"] = self.png_path.name
        self.write_meta()
        return self.png_path
