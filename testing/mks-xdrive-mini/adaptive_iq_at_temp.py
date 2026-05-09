"""Adaptive IQ controller targeting a fixed FET temperature.

Goal: find the maximum continuous IQ where FET temperature settles at the
target. The asymptotic IQ value is the motor's continuous rating with the
current cooling setup.

Algorithm:
  - Start IQ at IQ_START (default 30 A, known safe)
  - If FET below TEMP_LOW: raise IQ slowly (+UP_RATE A/s)
  - If FET in [TEMP_LOW, TEMP_HIGH]: hold IQ
  - If FET above TEMP_HIGH: lower IQ fast (-DN_RATE A/s)
  - HARD ABORT if FET >= TEMP_HARD_ABORT
  - Run for DURATION_S; final IQ = continuous rating

The asymmetric gains (slow up, fast down) handle the thermal lag from
τ ≈ 47 s. Going up faster than this causes overshoot; coming down should
be aggressive to stay safe.

Direction +1 (freewheel-engaged direction). Brake will resist; motor will
sit at very low RPM but draw the commanded current. That's the worst case
for thermal — at speed, the motor self-cools somewhat. So this is a
conservative measurement of continuous rating.
"""
from __future__ import annotations
import argparse
import csv
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

KT_NMA              = 0.04
DIRECTION           = +1

# Search bounds
IQ_START            = 30.0
IQ_MIN              = 20.0
IQ_MAX              = 60.0

# Temp targets
TARGET_TEMP_C       = 78.0    # aim here
TEMP_LOW            = 76.0    # deadband bottom
TEMP_HIGH           = 80.0    # deadband top  (user requested 80 C max)
TEMP_HARD_ABORT     = 85.0    # immediate disarm

# Gain rates (A/s of IQ adjustment)
UP_RATE_A_S         = 0.3
DN_RATE_A_S         = 2.0

# Loop / safety
LOOP_DT_S           = 0.1
DURATION_S          = 480.0   # 8 minutes default
RAMP_INITIAL_S      = 4.0     # initial soft ramp from 0 to IQ_START
VEL_ABORT_T_S       = 20.0
CURRENT_LIM_A       = 65.0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--target", type=float, default=TARGET_TEMP_C)
    p.add_argument("--temp-low", type=float, default=TEMP_LOW)
    p.add_argument("--temp-high", type=float, default=TEMP_HIGH)
    p.add_argument("--temp-abort", type=float, default=TEMP_HARD_ABORT)
    p.add_argument("--iq-start", type=float, default=IQ_START)
    p.add_argument("--iq-max", type=float, default=IQ_MAX)
    p.add_argument("--duration", type=float, default=DURATION_S)
    p.add_argument("--up-rate", type=float, default=UP_RATE_A_S)
    p.add_argument("--dn-rate", type=float, default=DN_RATE_A_S)
    args = p.parse_args()

    print(f"========== ADAPTIVE IQ-AT-TEMP ==========")
    print(f"  target {args.target}°C  band [{args.temp_low}, {args.temp_high}]  "
          f"abort {args.temp_abort}°C")
    print(f"  iq start {args.iq_start} A  max {args.iq_max} A")
    print(f"  rates: up {args.up_rate}/s  dn {args.dn_rate}/s")
    print(f"  duration {args.duration}s")

    import odrive
    odrv = odrive.find_any(timeout=15)
    if odrv is None: raise SystemExit("USB enum failed")
    ax = odrv.axis0
    mc = ax.motor.config; cc = ax.controller.config

    print(f"  pre: bus={odrv.vbus_voltage:.2f}V state={ax.current_state} FET={ax.fet_thermistor.temperature:.1f}C")
    if ax.fet_thermistor.temperature > 50.0:
        print("  !! FET >50°C — let it cool first")
        return

    ax.config.enable_watchdog = False
    for tgt in (ax, ax.motor, ax.encoder, ax.controller):
        try: tgt.error = 0
        except: pass
    mc.current_lim = CURRENT_LIM_A
    try: mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.3)
    except: pass
    try: odrv.config.dc_max_positive_current = CURRENT_LIM_A
    except: pass
    cc.control_mode = 1; cc.input_mode = 1
    ax.controller.input_torque = 0
    ax.requested_state = 8
    deadline = time.time() + 3.0
    while time.time() < deadline and ax.current_state != 8: time.sleep(0.05)
    if ax.current_state != 8:
        raise SystemExit(f"arm failed: {ax.current_state} {ax.error}")

    # Open log
    ts = time.strftime("%Y%m%d-%H%M%S")
    out_dir = Path(__file__).parent / "thermal_data"
    out_dir.mkdir(exist_ok=True)
    csv_path = out_dir / f"adaptive_iq_at_temp_{ts}.csv"
    png_path = out_dir / f"adaptive_iq_at_temp_{ts}.png"
    log = open(csv_path, "w", newline="")
    writer = csv.writer(log)
    writer.writerow(["t_s", "iq_target_a", "iq_meas_a", "vel_t_s", "fet_c",
                     "phase", "axis_err"])

    # State vars
    iq_target = 0.0
    abort = None
    t0 = time.time()
    last_print = -10.0
    samples = []

    def now(): return time.time() - t0

    def step(phase: str):
        nonlocal abort
        t = now()
        vel = ax.encoder.vel_estimate
        iq_meas = ax.motor.current_control.Iq_measured
        temp = ax.fet_thermistor.temperature
        errs = ax.error | ax.motor.error | ax.encoder.error
        target_T = DIRECTION * iq_target * KT_NMA
        ax.controller.input_torque = target_T
        writer.writerow([f"{t:.2f}", f"{iq_target:.2f}", f"{iq_meas:.2f}",
                         f"{vel:.4f}", f"{temp:.2f}", phase, ax.error])
        samples.append((t, iq_target, iq_meas, abs(vel)*60, temp, phase))

        nonlocal last_print
        if t - last_print > 5.0:
            print(f"  t={t:6.1f}s  iq_tgt={iq_target:5.2f}A  iq={iq_meas:+5.1f}A  "
                  f"vel={vel*60:+6.1f} RPM  FET={temp:.1f}C  {phase}")
            last_print = t

        if errs:
            abort = f"err: ax={ax.error}"; return None
        if abs(vel) > VEL_ABORT_T_S:
            abort = f"vel limit ({vel*60:.0f} RPM)"; return None
        if temp >= args.temp_abort:
            abort = f"HARD FET abort {temp:.1f}C"; return None
        return temp

    try:
        # --- Initial ramp 0 -> IQ_START ---
        rs = time.time()
        while time.time() - rs < RAMP_INITIAL_S:
            iq_target = args.iq_start * (time.time() - rs) / RAMP_INITIAL_S
            t = step("ramp")
            if t is None: break
            time.sleep(LOOP_DT_S)

        # --- Adaptive loop ---
        if not abort:
            iq_target = args.iq_start
            print(f"\n  --- adaptive search: target {args.target}°C, band "
                  f"[{args.temp_low}, {args.temp_high}], duration {args.duration}s ---")
            search_start = time.time()
            last_t = time.time()
            while time.time() - search_start < args.duration:
                temp = step("adaptive")
                if temp is None: break

                dt = time.time() - last_t
                last_t = time.time()
                if temp < args.temp_low:
                    iq_target = min(args.iq_max, iq_target + args.up_rate * dt)
                elif temp > args.temp_high:
                    iq_target = max(IQ_MIN, iq_target - args.dn_rate * dt)
                # else: hold

                time.sleep(LOOP_DT_S)
    finally:
        try:
            ax.controller.input_torque = 0
            time.sleep(0.3)
            ax.requested_state = 1
        except Exception: pass
        log.close()

    # --- Analysis ---
    print(f"\n========== RESULT ==========")
    print(f"  abort: {abort or 'completed'}")
    print(f"  total samples: {len(samples)}")
    if samples:
        adaptive_samples = [s for s in samples if s[5] == "adaptive"]
        if adaptive_samples and len(adaptive_samples) > 50:
            # Average IQ over the last 60s = best estimate of continuous rating
            t_end = adaptive_samples[-1][0]
            tail = [s for s in adaptive_samples if s[0] >= t_end - 60]
            if tail:
                avg_iq_tail = sum(s[1] for s in tail) / len(tail)
                avg_temp_tail = sum(s[4] for s in tail) / len(tail)
                std_iq_tail = (sum((s[1] - avg_iq_tail)**2 for s in tail) / len(tail)) ** 0.5
                print(f"  last-60s avg iq_target : {avg_iq_tail:.2f} A  (±{std_iq_tail:.2f})")
                print(f"  last-60s avg FET       : {avg_temp_tail:.2f} C")
                print(f"  CONTINUOUS RATING ESTIMATE: {avg_iq_tail:.1f} A "
                      f"= {avg_iq_tail * KT_NMA:.2f} Nm at {avg_temp_tail:.1f}°C FET")

    # Plot
    if samples:
        ts_arr = [s[0] for s in samples]
        iq_tgt = [s[1] for s in samples]
        iq_m   = [s[2] for s in samples]
        rpms   = [s[3] for s in samples]
        temps  = [s[4] for s in samples]
        fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        axs[0].plot(ts_arr, iq_tgt, "b-", label="IQ target", linewidth=1.5)
        axs[0].plot(ts_arr, iq_m,   "g:", label="IQ meas",   linewidth=1.0, alpha=0.7)
        axs[0].set_ylabel("IQ [A]")
        axs[0].grid(True, alpha=0.3); axs[0].legend()
        axs[1].plot(ts_arr, temps, "r-", linewidth=1.5)
        axs[1].axhline(args.target, color="green", linestyle="--", alpha=0.5, label="target")
        axs[1].axhline(args.temp_low, color="orange", linestyle=":", alpha=0.4, label="band")
        axs[1].axhline(args.temp_high, color="orange", linestyle=":", alpha=0.4)
        axs[1].axhline(args.temp_abort, color="red", linestyle="--", alpha=0.5, label="hard abort")
        axs[1].set_ylabel("FET [°C]")
        axs[1].grid(True, alpha=0.3); axs[1].legend(loc="lower right")
        axs[2].plot(ts_arr, rpms, "purple", linewidth=1.0)
        axs[2].set_ylabel("|RPM|")
        axs[2].set_xlabel("t [s]")
        axs[2].grid(True, alpha=0.3)
        fig.suptitle(f"Adaptive IQ-at-temp: target {args.target}°C, dir +1\n"
                     f"abort: {abort or 'completed'}")
        fig.tight_layout()
        fig.savefig(png_path, dpi=120)
        plt.close(fig)
        print(f"\n  CSV: {csv_path}")
        print(f"  PNG: {png_path}")


if __name__ == "__main__":
    main()
