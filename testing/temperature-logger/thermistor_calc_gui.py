"""Tkinter GUI for the QDD NTC thermistor (R <-> T, beta model).

Defaults match qdd_thermistor_logger.ino: 10k NTC, beta=3435 K, T0=25 C,
10k fixed resistor, Vref=4.35 V, thermistor-to-GND divider, 10-bit ADC.
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

DEFAULTS = {
    "r0": 10000.0,
    "beta": 3435.0,
    "t0_c": 25.0,
    "r_fixed": 10000.0,
    "vref": 4.35,
    "adc_max": 1023.0,
}


def beta_t_from_r(r: float, r0: float, beta: float, t0_c: float) -> float:
    t0_k = t0_c + 273.15
    inv_t = 1.0 / t0_k + (1.0 / beta) * math.log(r / r0)
    return 1.0 / inv_t - 273.15


def beta_r_from_t(t_c: float, r0: float, beta: float, t0_c: float) -> float:
    t0_k = t0_c + 273.15
    t_k = t_c + 273.15
    return r0 * math.exp(beta * (1.0 / t_k - 1.0 / t0_k))


def divider_voltage(r_therm: float, vref: float, r_fixed: float) -> float:
    # Thermistor-to-GND: Vadc = Vref * R_therm / (R_fixed + R_therm)
    return vref * r_therm / (r_fixed + r_therm)


def adc_counts(v: float, vref: float, adc_max: float) -> float:
    return v / vref * adc_max


class ThermistorApp:
    def __init__(self, root: tk.Tk) -> None:
        root.title("QDD Thermistor Calc")
        root.resizable(False, False)

        self.r0 = tk.StringVar(value=str(DEFAULTS["r0"]))
        self.beta = tk.StringVar(value=str(DEFAULTS["beta"]))
        self.t0 = tk.StringVar(value=str(DEFAULTS["t0_c"]))
        self.r_fixed = tk.StringVar(value=str(DEFAULTS["r_fixed"]))
        self.vref = tk.StringVar(value=str(DEFAULTS["vref"]))

        self.r_input = tk.StringVar(value="1700")
        self.t_input = tk.StringVar(value="25")

        self.r_to_t_out = tk.StringVar(value="-")
        self.t_to_r_out = tk.StringVar(value="-")

        pad = {"padx": 6, "pady": 3}

        params = ttk.LabelFrame(root, text="Thermistor parameters (beta model)")
        params.grid(row=0, column=0, sticky="ew", padx=10, pady=8)
        for i, (label, var) in enumerate([
            ("R0 [ohm]", self.r0),
            ("Beta [K]", self.beta),
            ("T0 [C]", self.t0),
            ("R_fixed [ohm]", self.r_fixed),
            ("Vref [V]", self.vref),
        ]):
            ttk.Label(params, text=label).grid(row=i, column=0, sticky="e", **pad)
            entry = ttk.Entry(params, textvariable=var, width=12)
            entry.grid(row=i, column=1, sticky="w", **pad)
            var.trace_add("write", lambda *_: self.recompute())

        rt = ttk.LabelFrame(root, text="Resistance -> Temperature")
        rt.grid(row=1, column=0, sticky="ew", padx=10, pady=4)
        ttk.Label(rt, text="R [ohm]").grid(row=0, column=0, sticky="e", **pad)
        ttk.Entry(rt, textvariable=self.r_input, width=12).grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(rt, text="->").grid(row=0, column=2, **pad)
        ttk.Label(rt, textvariable=self.r_to_t_out, width=42, anchor="w").grid(
            row=0, column=3, sticky="w", **pad
        )
        self.r_input.trace_add("write", lambda *_: self.recompute())

        tr = ttk.LabelFrame(root, text="Temperature -> Resistance")
        tr.grid(row=2, column=0, sticky="ew", padx=10, pady=4)
        ttk.Label(tr, text="T [C]").grid(row=0, column=0, sticky="e", **pad)
        ttk.Entry(tr, textvariable=self.t_input, width=12).grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(tr, text="->").grid(row=0, column=2, **pad)
        ttk.Label(tr, textvariable=self.t_to_r_out, width=42, anchor="w").grid(
            row=0, column=3, sticky="w", **pad
        )
        self.t_input.trace_add("write", lambda *_: self.recompute())

        plot_frame = ttk.LabelFrame(root, text="R vs T (beta model)")
        plot_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=4)

        self.fig = Figure(figsize=(6.0, 3.4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.log_y = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            plot_frame, text="log R axis", variable=self.log_y,
            command=self.recompute,
        ).pack(anchor="w", padx=4)

        ttk.Button(root, text="Reset to .ino defaults", command=self.reset).grid(
            row=4, column=0, sticky="w", padx=10, pady=(2, 10)
        )

        self.recompute()

    def _redraw_plot(self, params: dict[str, float], r_pt: float | None, t_pt: float | None) -> None:
        t_sweep = np.linspace(-20.0, 150.0, 400)
        r_sweep = np.array([
            beta_r_from_t(t, params["r0"], params["beta"], params["t0_c"])
            for t in t_sweep
        ])

        self.ax.clear()
        self.ax.plot(t_sweep, r_sweep, color="#1f77b4", linewidth=1.6)
        if self.log_y.get():
            self.ax.set_yscale("log")

        self.ax.axhline(params["r0"], color="gray", linestyle=":", linewidth=0.8)
        self.ax.axvline(params["t0_c"], color="gray", linestyle=":", linewidth=0.8)
        self.ax.plot(
            [params["t0_c"]], [params["r0"]],
            marker="o", color="gray", markersize=5, label=f"R0 ({params['t0_c']:.0f} C, {params['r0']:.0f} Ω)",
        )

        if r_pt is not None and t_pt is not None and r_pt > 0:
            self.ax.plot(
                [t_pt], [r_pt],
                marker="X", color="#d62728", markersize=10,
                label=f"current ({t_pt:.1f} C, {r_pt:.0f} Ω)",
            )

        self.ax.set_xlabel("Temperature [°C]")
        self.ax.set_ylabel("Resistance [Ω]")
        self.ax.grid(True, which="both", alpha=0.3)
        self.ax.legend(loc="best", fontsize=8)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def reset(self) -> None:
        self.r0.set(str(DEFAULTS["r0"]))
        self.beta.set(str(DEFAULTS["beta"]))
        self.t0.set(str(DEFAULTS["t0_c"]))
        self.r_fixed.set(str(DEFAULTS["r_fixed"]))
        self.vref.set(str(DEFAULTS["vref"]))

    def _floats(self) -> dict[str, float] | None:
        try:
            return {
                "r0": float(self.r0.get()),
                "beta": float(self.beta.get()),
                "t0_c": float(self.t0.get()),
                "r_fixed": float(self.r_fixed.get()),
                "vref": float(self.vref.get()),
            }
        except ValueError:
            return None

    def recompute(self) -> None:
        params = self._floats()
        if params is None:
            self.r_to_t_out.set("invalid parameters")
            self.t_to_r_out.set("invalid parameters")
            return

        r_pt: float | None = None
        t_pt: float | None = None

        try:
            r = float(self.r_input.get())
            if r <= 0:
                raise ValueError
            t_c = beta_t_from_r(r, params["r0"], params["beta"], params["t0_c"])
            v = divider_voltage(r, params["vref"], params["r_fixed"])
            counts = adc_counts(v, params["vref"], DEFAULTS["adc_max"])
            self.r_to_t_out.set(
                f"{t_c:7.2f} C  /  {t_c * 9 / 5 + 32:7.2f} F   |  Vadc {v:5.3f} V  ADC {counts:6.1f}"
            )
            r_pt, t_pt = r, t_c
        except ValueError:
            self.r_to_t_out.set("enter R > 0")

        try:
            t_c = float(self.t_input.get())
            r = beta_r_from_t(t_c, params["r0"], params["beta"], params["t0_c"])
            v = divider_voltage(r, params["vref"], params["r_fixed"])
            counts = adc_counts(v, params["vref"], DEFAULTS["adc_max"])
            self.t_to_r_out.set(
                f"{r:9.1f} ohm   |  Vadc {v:5.3f} V  ADC {counts:6.1f}"
            )
        except ValueError:
            self.t_to_r_out.set("enter T in C")

        self._redraw_plot(params, r_pt, t_pt)


def main() -> None:
    root = tk.Tk()
    ThermistorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
