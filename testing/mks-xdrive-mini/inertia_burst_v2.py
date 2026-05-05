"""Inertia identification — v2.

Differences from v1:
- Longer holds (10s) so position accumulates meaningfully
- Position-based alpha extraction (parabolic fit p(t) = p0 + v0*t + 0.5*alpha*t^2),
  ignoring the noisy filtered velocity signal entirely
- Skip the first 2s of each hold to avoid breakaway transients
- Uses the shared TestLogger for consistent CSV+PNG output
"""
from __future__ import annotations
import math
import time
from pathlib import Path
import numpy as np
import odrive
from _logger import TestLogger

# ---- knobs ----
KT_NMA            = 0.04
DIRECTION         = -1
BURST_CURRENTS_A  = [25, 35, 45, 55]
RAMP_S            = 1.5
HOLD_S            = 10.0
ALPHA_FIT_SKIP_S  = 2.0     # skip the first 2s of hold (breakaway transient)
IDLE_S            = 18.0    # coastdown + thermal recovery
LOOP_DT_S         = 0.04
VEL_ABORT_T_S     = 5.0
TEMP_WARN_C       = 70.0
TEMP_ABORT_C      = 80.0
CURRENT_LIM_A     = 70.0
DC_MAX_POS_CURRENT = 70.0

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

print("== Pre-test ==")
print(f"  bus={odrv.vbus_voltage:.2f} V  state={ax.current_state}  encoder.offset={ax.encoder.config.offset}")
print(f"  initial FET = {ax.fet_thermistor.temperature:.1f} C")

ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

mc.current_lim = CURRENT_LIM_A
try: mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.3)
except: pass
try: odrv.config.dc_max_positive_current = DC_MAX_POS_CURRENT
except: pass

cc.control_mode = 1
cc.input_mode = 1

logger = TestLogger("inertia_burst_v2", extra_fields=["burst_iq", "alpha_rad_s2"])

# Arm
ax.controller.input_torque = 0
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"!! arm failed: state={ax.current_state} err={ax.error}")
print("  armed in TORQUE mode")

t0 = time.time()
def now(): return time.time() - t0

abort_reason = None
last_print = -1.0

def maybe_print(t, phase, target_iq, cmd, iq, vel, pos, temp):
    global last_print
    if t - last_print > 0.25:
        print(f"   {t:7.2f} {phase:>10} {target_iq:>4} {cmd:+7.3f} {iq:+7.2f} {vel:+7.3f} {pos:+9.2f} {temp:5.1f}")
        last_print = t

burst_results = []  # dicts with target_iq, T_motor, alpha_rad_s2, T_friction estimate per burst

print(f"\n   {'t(s)':>7} {'phase':>10} {'tgt':>4} {'cmd':>7} {'Iq':>7} {'vel':>7} {'pos':>9} {'FET':>5}")

try:
    for burst_idx, target_iq in enumerate(BURST_CURRENTS_A):
        target_T = DIRECTION * target_iq * KT_NMA
        print(f"\n--- BURST {burst_idx+1}/{len(BURST_CURRENTS_A)}: target Iq = {target_iq} A  ({target_T:+.3f} Nm) ---")

        # --- RAMP ---
        ramp_start = time.time()
        while time.time() - ramp_start < RAMP_S:
            r = (time.time() - ramp_start) / RAMP_S
            cmd = target_T * r
            ax.controller.input_torque = cmd
            t = now()
            vel = ax.encoder.vel_estimate
            pos = ax.encoder.pos_estimate
            iq  = ax.motor.current_control.Iq_measured
            temp = ax.fet_thermistor.temperature
            errs = ax.error | ax.motor.error | ax.encoder.error
            logger.row(t_s=t, phase="ramp", cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
                       pos_turns=pos, fet_c=temp, axis_err=ax.error,
                       motor_err=ax.motor.error, encoder_err=ax.encoder.error,
                       burst_iq=target_iq)
            maybe_print(t, "ramp", target_iq, cmd, iq, vel, pos, temp)
            if errs:
                abort_reason = f"err during burst {burst_idx+1} ramp"; break
            if abs(vel) > VEL_ABORT_T_S:
                abort_reason = f"vel limit during burst {burst_idx+1} ramp"; break
            if temp >= TEMP_ABORT_C:
                abort_reason = f"FET {temp:.1f}C during burst {burst_idx+1} ramp"; break
            time.sleep(LOOP_DT_S)
        if abort_reason: break

        # --- HOLD --- collect (t_rel, pos) samples for parabolic fit
        hold_start_wall = time.time()
        hold_t0 = now()
        hold_samples = []  # (t_relative_to_hold_start, pos)
        while time.time() - hold_start_wall < HOLD_S:
            ax.controller.input_torque = target_T
            t = now()
            vel = ax.encoder.vel_estimate
            pos = ax.encoder.pos_estimate
            iq  = ax.motor.current_control.Iq_measured
            temp = ax.fet_thermistor.temperature
            errs = ax.error | ax.motor.error | ax.encoder.error
            logger.row(t_s=t, phase="hold", cmd_nm=target_T, iq_a=iq, vel_t_s=vel,
                       pos_turns=pos, fet_c=temp, axis_err=ax.error,
                       motor_err=ax.motor.error, encoder_err=ax.encoder.error,
                       burst_iq=target_iq)
            hold_samples.append((t - hold_t0, pos))
            maybe_print(t, "hold", target_iq, target_T, iq, vel, pos, temp)
            if errs:
                abort_reason = f"err during burst {burst_idx+1} hold"; break
            if abs(vel) > VEL_ABORT_T_S:
                abort_reason = f"vel limit during burst {burst_idx+1} hold"; break
            if temp >= TEMP_ABORT_C:
                abort_reason = f"FET {temp:.1f}C during burst {burst_idx+1} hold"; break
            time.sleep(LOOP_DT_S)
        if abort_reason: break

        # ---- POSITION-BASED ALPHA EXTRACTION ----
        # Skip first ALPHA_FIT_SKIP_S, then fit p(t) = p0 + v0*t + 0.5*alpha*t^2
        samples_for_fit = [(tr, p) for (tr, p) in hold_samples if tr >= ALPHA_FIT_SKIP_S]
        if len(samples_for_fit) >= 5:
            tr = np.array([s[0] for s in samples_for_fit])
            ps = np.array([s[1] for s in samples_for_fit])
            # solve [t^2/2, t, 1] * [alpha, v0, p0] = pos via lstsq
            A = np.column_stack([0.5*tr*tr, tr, np.ones_like(tr)])
            (alpha_t_s2, v0_t_s, p0), *_ = np.linalg.lstsq(A, ps, rcond=None)
            alpha_rad_s2 = alpha_t_s2 * 2 * math.pi
            print(f"  -> fit on {len(samples_for_fit)} samples (skipped first {ALPHA_FIT_SKIP_S}s):")
            print(f"     alpha = {alpha_t_s2:+.5f} t/s^2 = {alpha_rad_s2:+.4f} rad/s^2")
            print(f"     v0 (at hold start)  = {v0_t_s:+.4f} t/s")
            burst_results.append({
                "target_iq": target_iq,
                "target_T_nm": target_T,
                "alpha_t_s2": alpha_t_s2,
                "alpha_rad_s2": alpha_rad_s2,
                "v0_t_s": v0_t_s,
                "n_fit_pts": len(samples_for_fit),
            })
        else:
            print(f"  -> not enough samples for fit ({len(samples_for_fit)})")

        # --- IDLE between bursts ---
        ax.controller.input_torque = 0
        ax.requested_state = 1
        time.sleep(0.3)
        idle_start = time.time()
        while time.time() - idle_start < IDLE_S:
            t = now()
            vel = ax.encoder.vel_estimate
            pos = ax.encoder.pos_estimate
            iq  = ax.motor.current_control.Iq_measured
            temp = ax.fet_thermistor.temperature
            logger.row(t_s=t, phase="idle", cmd_nm=0.0, iq_a=iq, vel_t_s=vel,
                       pos_turns=pos, fet_c=temp, axis_err=ax.error,
                       motor_err=ax.motor.error, encoder_err=ax.encoder.error,
                       burst_iq=target_iq)
            if t - last_print > 1.5:
                print(f"   {t:7.2f} {'idle':>10} {target_iq:>4} {0.0:+7.3f} {iq:+7.2f} {vel:+7.3f} {pos:+9.2f} {temp:5.1f}")
                last_print = t
            time.sleep(0.5)

        # Re-arm for next burst
        if burst_idx < len(BURST_CURRENTS_A) - 1:
            for tgt in (ax, ax.motor, ax.encoder, ax.controller):
                try: tgt.error = 0
                except: pass
            ax.controller.input_torque = 0
            ax.requested_state = 8
            deadline = time.time() + 3.0
            while time.time() < deadline and ax.current_state != 8:
                time.sleep(0.05)
            if ax.current_state != 8:
                abort_reason = f"re-arm failed before burst {burst_idx+2}"; break

finally:
    ax.controller.input_torque = 0
    ax.requested_state = 1
    logger.close()

# ---- ANALYSIS ----
print(f"\n=== BURST RESULTS (position-based alpha) ===")
print(f"  {'iq':>4} {'T_motor':>10} {'alpha':>14} {'v0':>10} {'n_pts':>6}")
for r in burst_results:
    print(f"  {r['target_iq']:>4} {r['target_T_nm']:+10.3f} {r['alpha_rad_s2']:+14.4f} {r['v0_t_s']:+10.4f} {r['n_fit_pts']:>6}")

J_fit = T_friction_fit = None
if len(burst_results) >= 2:
    T_arr = np.array([abs(r["target_T_nm"]) for r in burst_results])
    a_arr = np.array([abs(r["alpha_rad_s2"]) for r in burst_results])
    A = np.vstack([a_arr, np.ones_like(a_arr)]).T
    J_fit, T_friction_fit = np.linalg.lstsq(A, T_arr, rcond=None)[0]
    print(f"\n=== INERTIA / FRICTION FIT ===")
    print(f"  T_motor (Nm) = J * alpha (rad/s^2) + T_friction (Nm)")
    print(f"  J          = {J_fit:.4f} kg·m^2")
    print(f"  T_friction = {T_friction_fit:.3f} Nm")
    geom_J = 0.1125
    print(f"  geometric J of 9kg/30cm/10cm annulus = {geom_J} kg·m^2")
    print(f"  fitted J / geometric J = {J_fit/geom_J:.2f}x")

# ---- PLOTS ----
# Panel 1: standard time series
subtitle = "loaded direction, multi-current bursts; alpha extracted via parabolic fit on position"
if J_fit is not None:
    subtitle += f"\nJ_fit = {J_fit:.3f} kg·m^2,  T_friction = {T_friction_fit:.3f} Nm"
logger.plot(title="Inertia burst v2", subtitle=subtitle)
print(f"\n  CSV: {logger.csv_path}")
print(f"  PNG (timeseries): {logger.png_path}")

# Panel 2: alpha-vs-T scatter with fit (separate file)
if len(burst_results) >= 2:
    try:
        import matplotlib.pyplot as plt
        from pathlib import Path as _P
        fit_png = _P(str(logger.png_path).replace(".png", "_fit.png"))
        fig, ax_p = plt.subplots(1, 1, figsize=(8, 6))
        ax_p.scatter(a_arr, T_arr, s=80, color="#26a", label="bursts (measured)")
        a_line = np.linspace(0, max(a_arr)*1.1, 50)
        ax_p.plot(a_line, J_fit*a_line + T_friction_fit, color="#c44", lw=2,
                  label=f"fit: T = {J_fit:.3f}·alpha + {T_friction_fit:.3f}")
        ax_p.axhline(T_friction_fit, color="#888", linestyle="--", alpha=0.5,
                     label=f"T_friction = {T_friction_fit:.3f} Nm")
        for r in burst_results:
            ax_p.annotate(f"{r['target_iq']}A", (abs(r['alpha_rad_s2']), abs(r['target_T_nm'])),
                          xytext=(8, 0), textcoords="offset points", fontsize=9)
        ax_p.set_xlabel("Angular accel  alpha  (rad/s^2)")
        ax_p.set_ylabel("Motor torque (Nm)")
        ax_p.set_title(f"Inertia identification (v2, position-based)\nJ = {J_fit:.4f} kg·m^2,  T_friction = {T_friction_fit:.3f} Nm")
        ax_p.legend()
        ax_p.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(fit_png, dpi=130)
        plt.close(fig)
        print(f"  PNG (fit):        {fit_png}")
    except Exception as e:
        print(f"  fit plot failed: {e}")

print(f"  abort_reason: {abort_reason if abort_reason else 'completed all bursts'}")
print("Returned to IDLE.")
