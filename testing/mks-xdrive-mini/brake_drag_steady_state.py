"""Multi-current steady-state brake drag characterization (H2 trainer).

Goal: fit T_drag(omega) for the Saris H2 eddy brake by stepping through a
ladder of motor IQ commands and recording the equilibrium RPM at each.

At equilibrium with constant IQ in +1 direction (freewheel engaged):

    T_motor = T_drag(omega_eq)
    Kt * IQ = T_drag(omega_eq)

So a sweep of IQ values gives a (omega_eq, T_drag) table.

Stability criterion: rolling-window RPM std-dev below threshold for STABLE_WINDOW_S.

Safety:
- FET temp abort (default 80 C)
- Velocity abort (run-away)
- Error abort
- Auto-cool phase between steps if FET > FET_COOL_TRIGGER

Direction +1 (freewheel-engaged direction). The H2 brake CANNOT be disabled
on this firmware (see H2_BENCH_FINDINGS.md), so this whole sweep happens
under baseline brake.
"""
from __future__ import annotations
import argparse
import statistics
import time
from collections import deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from _logger import TestLogger

KT_NMA            = 0.04
DIRECTION_DEFAULT = +1

DEFAULT_IQ_STEPS  = [10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0]

RAMP_S            = 1.5     # seconds to ramp into each IQ target
MIN_HOLD_S        = 8.0     # minimum hold per step (lets thermal drift settle a bit)
MAX_HOLD_S        = 35.0    # cap per step
STABLE_WINDOW_S   = 5.0     # need this much rolling-std-low time
STABLE_RPM_STD    = 0.6     # RPM std-dev threshold for "stable"

FET_COOL_TRIGGER  = 70.0    # if FET hits this before next step, cool first
FET_COOL_RESUME   = 60.0    # resume next step when FET cools to this
COOL_MAX_S        = 90.0    # cap for cool-down

LOOP_DT_S         = 0.10
RPM_VEL_ABORT     = 5.0     # turns/sec — anything above this is unexpected (brake-loaded)
FET_HARD_ABORT    = 80.0
FET_PRECHECK_MAX  = 50.0    # refuse to start if FET already this hot
CURRENT_LIM_A     = 55.0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=str, default=None,
                   help="Comma-separated IQ steps (A). Default 10..45 by 5.")
    p.add_argument("--min-hold", type=float, default=MIN_HOLD_S)
    p.add_argument("--max-hold", type=float, default=MAX_HOLD_S)
    p.add_argument("--stable-window", type=float, default=STABLE_WINDOW_S)
    p.add_argument("--stable-std", type=float, default=STABLE_RPM_STD)
    p.add_argument("--fet-abort", type=float, default=FET_HARD_ABORT)
    p.add_argument("--fet-cool-trigger", type=float, default=FET_COOL_TRIGGER)
    p.add_argument("--fet-cool-resume", type=float, default=FET_COOL_RESUME)
    p.add_argument("--fet-precheck", type=float, default=FET_PRECHECK_MAX)
    p.add_argument("--esc-fan", choices=["on", "off"], default="on")
    p.add_argument("--motor-fan", choices=["on", "off"], default="on")
    p.add_argument("--name", type=str, default="brake_drag_steady_state")
    p.add_argument("--direction", type=int, default=DIRECTION_DEFAULT, choices=[-1, +1],
                   help="Motor torque sign. The freewheel-engaged direction depends on "
                        "current encoder polarity (may flip after a recal).")
    args = p.parse_args()
    DIRECTION = args.direction

    if args.steps:
        iq_steps = [float(x) for x in args.steps.split(",")]
    else:
        iq_steps = list(DEFAULT_IQ_STEPS)

    print(f"========== BRAKE DRAG STEADY-STATE SWEEP ==========")
    print(f"  IQ steps    : {iq_steps} A")
    print(f"  hold        : min {args.min_hold}s, max {args.max_hold}s")
    print(f"  stability   : std<{args.stable_std} RPM over {args.stable_window}s")
    print(f"  FET safety  : abort {args.fet_abort}C, cool>{args.fet_cool_trigger}C, "
          f"resume<{args.fet_cool_resume}C")
    print(f"  cooling     : ESC fan {args.esc_fan}, motor fan {args.motor_fan}")

    import odrive
    odrv = odrive.find_any(timeout=15)
    if odrv is None:
        raise SystemExit("USB enum failed")
    ax = odrv.axis0
    mc = ax.motor.config
    cc = ax.controller.config

    pre_fet = ax.fet_thermistor.temperature
    print(f"\n  pre: bus={odrv.vbus_voltage:.2f}V state={ax.current_state} FET={pre_fet:.1f}C")
    if pre_fet > args.fet_precheck:
        print(f"  !! FET >{args.fet_precheck}C - let it cool first")
        return

    ax.config.enable_watchdog = False
    for tgt in (ax, ax.motor, ax.encoder, ax.controller):
        try: tgt.error = 0
        except Exception: pass
    mc.current_lim = CURRENT_LIM_A
    try: mc.requested_current_range = max(getattr(mc, "requested_current_range", 0), CURRENT_LIM_A * 1.3)
    except Exception: pass
    try: odrv.config.dc_max_positive_current = CURRENT_LIM_A
    except Exception: pass
    cc.control_mode = 1   # torque mode
    cc.input_mode = 1     # passthrough
    ax.controller.input_torque = 0
    ax.requested_state = 8
    deadline = time.time() + 3.0
    while time.time() < deadline and ax.current_state != 8:
        time.sleep(0.05)
    if ax.current_state != 8:
        raise SystemExit(f"arm failed: state={ax.current_state} err={ax.error}")

    logger = TestLogger(args.name)
    logger.set_meta(
        purpose="Multi-current steady-state sweep to fit T_drag(omega) for H2 trainer brake",
        hardware={
            "motor": "D6374-150KV",
            "controller": "MKS XDrive Mini (ODrive 0.5.1)",
            "encoder": "AMT-102 ABI 8192 CPR",
            "trainer": "Saris Hammer H2 (eddy brake, baseline-clamped)",
        },
        conditions={
            "esc_fan": args.esc_fan == "on",
            "motor_fan": args.motor_fan == "on",
            "trainer_ac": "on",
            "trainer_ble_state": "default (no BLE control sent)",
            "direction": DIRECTION,
            "bus_v": float(odrv.vbus_voltage),
            "current_lim_a": CURRENT_LIM_A,
        },
        parameters={
            "iq_steps_a": iq_steps,
            "min_hold_s": args.min_hold,
            "max_hold_s": args.max_hold,
            "stable_window_s": args.stable_window,
            "stable_rpm_std": args.stable_std,
            "kt_nm_per_a": KT_NMA,
        },
    )

    abort = None
    step_results: list[dict] = []
    t0 = time.time()
    last_print = -10.0

    def now(): return time.time() - t0

    def sample_and_log(phase: str, iq_target: float) -> tuple[float, float, float] | None:
        """Read state, write row. Returns (rpm, fet, iq_meas) or None if abort triggered."""
        nonlocal abort, last_print
        vel_t_s = ax.encoder.vel_estimate
        iq_meas = ax.motor.current_control.Iq_measured
        fet = ax.fet_thermistor.temperature
        pos = ax.encoder.pos_estimate
        cmd_nm = DIRECTION * iq_target * KT_NMA
        ax.controller.input_torque = cmd_nm
        rpm = abs(vel_t_s) * 60.0

        logger.row(t_s=now(), phase=phase, cmd_nm=cmd_nm, iq_a=iq_meas,
                   vel_t_s=vel_t_s, pos_turns=pos, fet_c=fet,
                   axis_err=ax.error, motor_err=ax.motor.error,
                   encoder_err=ax.encoder.error,
                   extra=f"iq_tgt={iq_target:.2f}")

        if now() - last_print > 4.0:
            print(f"  t={now():6.1f}s  phase={phase:<6}  iq_tgt={iq_target:5.1f}A  "
                  f"iq={iq_meas:+5.1f}A  RPM={rpm:6.1f}  FET={fet:5.1f}C")
            last_print = now()

        if ax.error or ax.motor.error or ax.encoder.error:
            abort = f"err: ax={ax.error} mot={ax.motor.error} enc={ax.encoder.error}"
            return None
        if abs(vel_t_s) > RPM_VEL_ABORT:
            abort = f"velocity abort ({rpm:.0f} RPM)"
            return None
        if fet >= args.fet_abort:
            abort = f"HARD FET abort {fet:.1f}C"
            return None
        return rpm, fet, iq_meas

    try:
        for step_idx, iq_target in enumerate(iq_steps):
            if abort:
                break
            print(f"\n  --- STEP {step_idx+1}/{len(iq_steps)}: IQ = {iq_target:.1f} A ---")

            # Cool-down if FET too hot before this step
            cur_fet = ax.fet_thermistor.temperature
            if cur_fet > args.fet_cool_trigger:
                print(f"  FET {cur_fet:.1f}C > {args.fet_cool_trigger}C - cooling first")
                cool_start = time.time()
                while time.time() - cool_start < COOL_MAX_S:
                    out = sample_and_log("cool", 0.0)
                    if out is None: break
                    if out[1] <= args.fet_cool_resume:
                        print(f"  cooled to {out[1]:.1f}C, resuming")
                        break
                    time.sleep(LOOP_DT_S)
                if abort: break

            # Ramp from 0 to iq_target
            ramp_start = time.time()
            while time.time() - ramp_start < RAMP_S:
                frac = (time.time() - ramp_start) / RAMP_S
                out = sample_and_log("ramp", iq_target * frac)
                if out is None: break
                time.sleep(LOOP_DT_S)
            if abort: break

            # Hold and watch for stability
            rpm_buf: deque[float] = deque()  # (t, rpm) pairs trimmed to stable window
            t_buf: deque[float] = deque()
            hold_start = time.time()
            stable_at = None
            while True:
                out = sample_and_log("hold", iq_target)
                if out is None: break
                rpm = out[0]
                t_buf.append(now()); rpm_buf.append(rpm)
                # trim to stability window
                while t_buf and (now() - t_buf[0]) > args.stable_window:
                    t_buf.popleft(); rpm_buf.popleft()

                elapsed = time.time() - hold_start
                if elapsed >= args.min_hold and len(rpm_buf) >= 10:
                    rpm_std = statistics.pstdev(rpm_buf)
                    if rpm_std < args.stable_std:
                        stable_at = elapsed
                        break
                if elapsed >= args.max_hold:
                    break
                time.sleep(LOOP_DT_S)
            if abort: break

            # Equilibrium = mean of last stable_window seconds
            if rpm_buf:
                eq_rpm = sum(rpm_buf) / len(rpm_buf)
                eq_std = statistics.pstdev(rpm_buf) if len(rpm_buf) > 1 else 0.0
                eq_torque = iq_target * KT_NMA
                fet_now = ax.fet_thermistor.temperature
                step_results.append({
                    "iq_a": iq_target,
                    "torque_nm": eq_torque,
                    "rpm_eq": eq_rpm,
                    "rpm_std": eq_std,
                    "fet_c": fet_now,
                    "stable_at_s": stable_at,
                    "samples": len(rpm_buf),
                })
                print(f"    EQ: {eq_rpm:.2f} RPM (std {eq_std:.2f}, n={len(rpm_buf)}) "
                      f"-> T_drag = {eq_torque:.3f} Nm at FET={fet_now:.1f}C "
                      f"{'(stable)' if stable_at else '(timeout)'}")

    finally:
        try:
            ax.controller.input_torque = 0
            time.sleep(0.3)
            ax.requested_state = 1
        except Exception:
            pass

    # Save & report
    print(f"\n========== RESULT ==========")
    print(f"  abort: {abort or 'completed'}")
    print(f"  steps completed: {len(step_results)}/{len(iq_steps)}")
    for r in step_results:
        print(f"    {r['iq_a']:5.1f} A  -> {r['rpm_eq']:6.2f} RPM  "
              f"(T_drag = {r['torque_nm']:.3f} Nm)")

    logger.set_meta(result={
        "abort": abort,
        "steps_completed": len(step_results),
        "steps": step_results,
    })

    logger.plot(
        title=f"Brake drag steady-state sweep (H2)",
        subtitle=f"{len(step_results)}/{len(iq_steps)} steps; {abort or 'completed'}",
    )

    # Extra plot: equilibrium points
    if step_results:
        rpms = [r["rpm_eq"] for r in step_results]
        ts = [r["torque_nm"] for r in step_results]
        fig, ax2 = plt.subplots(figsize=(8, 6))
        ax2.scatter(rpms, ts, color="#c44", s=60, zorder=5)
        for r in step_results:
            ax2.annotate(f"{r['iq_a']:.0f}A",
                         (r["rpm_eq"], r["torque_nm"]),
                         textcoords="offset points", xytext=(6, -3), fontsize=9)
        # Linear fit
        if len(rpms) >= 2:
            n = len(rpms)
            sx = sum(rpms); sy = sum(ts); sxy = sum(x*y for x, y in zip(rpms, ts))
            sxx = sum(x*x for x in rpms)
            denom = n*sxx - sx*sx
            if denom != 0:
                slope = (n*sxy - sx*sy)/denom
                intercept = (sy - slope*sx)/n
                xs = [min(rpms)*0.9, max(rpms)*1.05]
                ys = [slope*x + intercept for x in xs]
                ax2.plot(xs, ys, "k--", alpha=0.4,
                         label=f"linear: T = {slope:.4f}*RPM + {intercept:.3f}")
                ax2.legend()
                logger.set_meta(linear_fit={
                    "slope_nm_per_rpm": slope,
                    "intercept_nm": intercept,
                    "samples": len(rpms),
                })
                logger.write_meta()
        ax2.set_xlabel("Equilibrium RPM")
        ax2.set_ylabel("Motor torque @ equilibrium (Nm)  =  T_drag(ω)")
        ax2.set_title("H2 brake drag curve (motor-shaft frame)")
        ax2.grid(True, alpha=0.3)
        from pathlib import Path
        eq_png = logger.png_path.with_name(logger.png_path.stem + "_eq_curve.png")
        fig.tight_layout()
        fig.savefig(eq_png, dpi=130)
        plt.close(fig)
        print(f"\n  EQ curve PNG: {eq_png}")

    print(f"\n  CSV : {logger.csv_path}")
    print(f"  PNG : {logger.png_path}")
    print(f"  META: {logger.meta_path}")


if __name__ == "__main__":
    main()
