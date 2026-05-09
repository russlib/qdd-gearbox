"""Coast-down characterization of the Saris H2 brake (passive decay).

Goal: extract T_drag(omega) over the full speed range by spinning the motor
up against the brake, cutting torque, and logging RPM decay.

Two trial modes:
  --mode settle : ramp up to IQ_SPIN, hold until RPM stabilizes, then disarm.
                  Coast starts at the brake's regulated plateau (~30 RPM).
  --mode kick   : ramp to a high IQ pulse for KICK_S to try to overshoot
                  the plateau, then disarm. Coast may start above plateau
                  if the brake regulator has lag.

Decay gives RPM(t). Numerical derivative gives dRPM/dt. With J (effective
inertia at motor shaft) we could solve T_drag = -J*alpha. We don't know J
yet, so this script reports the SHAPE of T_drag(omega) and a self-consistent
J estimate using the known anchor:

    At the moment torque is cut, T_drag(omega_0) = T_motor_last
    so J = T_motor_last / |alpha(t=0+)|

Direction defaults to -1 (current freewheel-engaged side after the
2026-05-08 recal). Pass --direction +1 if needed.

Safety:
- FET hard abort (default 80 C)
- Velocity abort (default 5.0 t/s = 300 RPM ceiling)
- Aborts re-disarm in finally
"""
from __future__ import annotations
import argparse
import statistics
import time
from collections import deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from _logger import TestLogger

KT_NMA            = 0.04
DIRECTION_DEFAULT = -1

# Spin-up phase
IQ_SPIN_DEFAULT   = 30.0
RAMP_S            = 1.0
SPIN_HOLD_S       = 8.0       # used in settle mode
KICK_HOLD_S       = 1.5       # used in kick mode

# Coast phase
COAST_S           = 30.0
RPM_STOP          = 0.5       # below this, end early

# Loop / safety
LOOP_DT_S         = 0.05
RPM_VEL_ABORT     = 5.0
FET_HARD_ABORT    = 80.0
FET_PRECHECK_MAX  = 50.0
CURRENT_LIM_A     = 60.0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["settle", "kick"], default="settle")
    p.add_argument("--iq-spin", type=float, default=IQ_SPIN_DEFAULT)
    p.add_argument("--spin-hold", type=float, default=None,
                   help="Spin hold seconds; defaults to 8s settle / 1.5s kick.")
    p.add_argument("--coast-s", type=float, default=COAST_S)
    p.add_argument("--direction", type=int, default=DIRECTION_DEFAULT, choices=[-1, +1])
    p.add_argument("--esc-fan", choices=["on", "off"], default="on")
    p.add_argument("--motor-fan", choices=["on", "off"], default="on")
    p.add_argument("--name", type=str, default=None,
                   help="Test name override; default coast_down_<mode>_<iq>A")
    args = p.parse_args()

    DIRECTION = args.direction
    spin_hold = args.spin_hold
    if spin_hold is None:
        spin_hold = SPIN_HOLD_S if args.mode == "settle" else KICK_HOLD_S
    test_name = args.name or f"coast_down_{args.mode}_{int(args.iq_spin)}A"

    print(f"========== COAST-DOWN ==========")
    print(f"  mode        : {args.mode}")
    print(f"  iq spin     : {args.iq_spin} A   ramp {RAMP_S}s   hold {spin_hold}s")
    print(f"  coast       : {args.coast_s}s (or until RPM<{RPM_STOP})")
    print(f"  direction   : {DIRECTION:+d}")
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
    if pre_fet > FET_PRECHECK_MAX:
        print(f"  !! FET >{FET_PRECHECK_MAX}C - cool first")
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

    logger = TestLogger(test_name)
    logger.set_meta(
        purpose=f"Coast-down ({args.mode}) characterization of H2 brake; "
                f"spin to {args.iq_spin}A then cut torque to log RPM decay",
        hardware={
            "motor": "D6374-150KV",
            "controller": "MKS XDrive Mini (ODrive 0.5.1)",
            "encoder": "AMT-102 ABI 8192 CPR",
            "trainer": "Saris Hammer H2",
        },
        conditions={
            "esc_fan": args.esc_fan == "on",
            "motor_fan": args.motor_fan == "on",
            "trainer_ac": "on",
            "trainer_ble_state": "default (no BLE control)",
            "direction": DIRECTION,
            "bus_v": float(odrv.vbus_voltage),
            "current_lim_a": CURRENT_LIM_A,
        },
        parameters={
            "mode": args.mode,
            "iq_spin_a": args.iq_spin,
            "ramp_s": RAMP_S,
            "spin_hold_s": spin_hold,
            "coast_s": args.coast_s,
            "kt_nm_per_a": KT_NMA,
        },
    )

    abort = None
    t0 = time.time()
    last_print = -10.0
    samples = []  # (t, phase, iq_meas, vel_t_s, fet)

    def now(): return time.time() - t0

    def sample_and_log(phase: str, iq_target: float):
        nonlocal abort, last_print
        vel_t_s = ax.encoder.vel_estimate
        iq_meas = ax.motor.current_control.Iq_measured
        fet = ax.fet_thermistor.temperature
        pos = ax.encoder.pos_estimate
        cmd_nm = DIRECTION * iq_target * KT_NMA
        ax.controller.input_torque = cmd_nm
        rpm = abs(vel_t_s) * 60.0
        samples.append((now(), phase, iq_meas, vel_t_s, fet))
        logger.row(t_s=now(), phase=phase, cmd_nm=cmd_nm, iq_a=iq_meas,
                   vel_t_s=vel_t_s, pos_turns=pos, fet_c=fet,
                   axis_err=ax.error, motor_err=ax.motor.error,
                   encoder_err=ax.encoder.error,
                   extra=f"iq_tgt={iq_target:.2f}")

        if now() - last_print > 2.0:
            print(f"  t={now():6.2f}s  phase={phase:<5}  iq_tgt={iq_target:5.1f}A  "
                  f"iq={iq_meas:+5.1f}A  RPM={rpm:6.1f}  FET={fet:5.1f}C")
            last_print = now()

        if ax.error or ax.motor.error or ax.encoder.error:
            abort = f"err: ax={ax.error} mot={ax.motor.error} enc={ax.encoder.error}"
            return False
        if abs(vel_t_s) > RPM_VEL_ABORT:
            abort = f"velocity abort ({rpm:.0f} RPM)"
            return False
        if fet >= FET_HARD_ABORT:
            abort = f"HARD FET abort {fet:.1f}C"
            return False
        return True

    last_iq_pre_coast = None

    try:
        # --- Ramp phase ---
        ramp_start = time.time()
        while time.time() - ramp_start < RAMP_S:
            frac = (time.time() - ramp_start) / RAMP_S
            if not sample_and_log("ramp", args.iq_spin * frac): break
            time.sleep(LOOP_DT_S)
        if abort: raise RuntimeError("aborted in ramp")

        # --- Spin-hold phase ---
        hold_start = time.time()
        while time.time() - hold_start < spin_hold:
            if not sample_and_log("spin", args.iq_spin): break
            time.sleep(LOOP_DT_S)
        if abort: raise RuntimeError("aborted in spin")

        # Capture last commanded torque for J calibration
        last_iq_pre_coast = args.iq_spin

        # --- Coast phase: cut torque to 0 ---
        ax.controller.input_torque = 0
        coast_start = time.time()
        while time.time() - coast_start < args.coast_s:
            vel_t_s = ax.encoder.vel_estimate
            if abs(vel_t_s) * 60.0 < RPM_STOP and (time.time() - coast_start) > 0.5:
                print(f"  coast end: RPM<{RPM_STOP}")
                # Still log a couple final rows
                for _ in range(3):
                    if not sample_and_log("coast", 0.0): break
                    time.sleep(LOOP_DT_S)
                break
            if not sample_and_log("coast", 0.0): break
            time.sleep(LOOP_DT_S)

    except RuntimeError as e:
        print(f"  loop exit: {e}")
    finally:
        try:
            ax.controller.input_torque = 0
            time.sleep(0.3)
            ax.requested_state = 1
        except Exception:
            pass

    # --- Analysis ---
    print(f"\n========== RESULT ==========")
    print(f"  abort: {abort or 'completed'}")
    print(f"  total samples: {len(samples)}")

    coast = [s for s in samples if s[1] == "coast"]
    j_estimate = None
    if len(coast) >= 8 and last_iq_pre_coast is not None:
        # Build numerical derivative dRPM/dt from coast samples
        t_arr = np.array([s[0] for s in coast])
        rpm_arr = np.abs(np.array([s[3] for s in coast]) * 60.0)
        # smooth a touch with rolling mean (window 3)
        if len(rpm_arr) >= 5:
            rpm_smooth = np.convolve(rpm_arr, np.ones(3)/3.0, mode="same")
        else:
            rpm_smooth = rpm_arr
        dt = np.diff(t_arr)
        dr = np.diff(rpm_smooth)
        valid = dt > 1e-4
        alpha_rpm_s = dr[valid] / dt[valid]                  # RPM/s
        alpha_rad_s2 = alpha_rpm_s * 2*np.pi / 60.0          # rad/s^2
        omega_rad_s = (rpm_smooth[:-1][valid]) * 2*np.pi / 60.0
        # J calibration: at the very start of coast, T_drag = T_motor_last
        # (continuity assumption, may be loose if regulator reacts fast)
        T_motor_last = abs(last_iq_pre_coast) * KT_NMA
        # Use first 5 valid points for alpha at coast start
        n0 = min(5, len(alpha_rad_s2))
        if n0 > 0:
            alpha0 = float(np.mean(alpha_rad_s2[:n0]))
            if alpha0 < -1e-3:
                j_estimate = T_motor_last / abs(alpha0)
                print(f"  T_motor_last  : {T_motor_last:.3f} Nm")
                print(f"  alpha (start) : {alpha0:.3f} rad/s^2")
                print(f"  J estimate    : {j_estimate*1000:.2f} g*m^2 (= {j_estimate:.6f} kg*m^2)")
        # Save table
        if j_estimate:
            t_drag_arr = -j_estimate * alpha_rad_s2
            print(f"\n  T_drag samples (omega [rad/s], T_drag [Nm]):")
            n_print = min(8, len(omega_rad_s))
            stride = max(1, len(omega_rad_s) // n_print)
            for i in range(0, len(omega_rad_s), stride):
                print(f"    omega={omega_rad_s[i]:6.2f}  T_drag={t_drag_arr[i]:.3f}")
            logger.set_meta(
                analysis={
                    "j_kg_m2": j_estimate,
                    "T_motor_last_nm": T_motor_last,
                    "alpha_initial_rad_s2": alpha0,
                    "n_coast_samples": int(len(coast)),
                }
            )

    logger.plot(
        title=f"Coast-down ({args.mode}) — IQ_spin {args.iq_spin}A",
        subtitle=f"{abort or 'completed'}; {len(coast)} coast samples",
    )

    # Extra plot: T_drag vs omega (if J known)
    if j_estimate and len(omega_rad_s) > 5:
        fig, axs = plt.subplots(2, 1, figsize=(9, 9))

        # RPM(t) coast trace
        t_coast = t_arr - t_arr[0]
        axs[0].plot(t_coast, rpm_arr, "b-", lw=1.5, label="raw RPM")
        axs[0].plot(t_coast, rpm_smooth, "r--", lw=1.0, alpha=0.7, label="smoothed")
        axs[0].set_xlabel("Coast time [s]"); axs[0].set_ylabel("RPM")
        axs[0].set_title(f"Coast-down RPM decay (start IQ {args.iq_spin}A)")
        axs[0].grid(True, alpha=0.3); axs[0].legend()

        # T_drag(omega)
        axs[1].plot(omega_rad_s, -j_estimate * alpha_rad_s2, "k-", lw=1.0)
        axs[1].set_xlabel("omega [rad/s]"); axs[1].set_ylabel("T_drag [Nm]")
        axs[1].set_title(f"H2 brake T_drag(ω) — J={j_estimate*1000:.1f} g*m^2 "
                         f"(anchored on T_motor_last={T_motor_last:.2f} Nm)")
        axs[1].grid(True, alpha=0.3)
        axs[1].axhline(0, color="gray", lw=0.5)
        axs[1].axvline(0, color="gray", lw=0.5)

        fig.tight_layout()
        eq_png = logger.png_path.with_name(logger.png_path.stem + "_drag_curve.png")
        fig.savefig(eq_png, dpi=130)
        plt.close(fig)
        print(f"\n  drag curve PNG: {eq_png}")

    print(f"\n  CSV : {logger.csv_path}")
    print(f"  PNG : {logger.png_path}")
    print(f"  META: {logger.meta_path}")


if __name__ == "__main__":
    main()
