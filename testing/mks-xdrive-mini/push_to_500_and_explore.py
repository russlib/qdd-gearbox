"""Multi-phase motor test designed to push the H2 flywheel to high speed
and characterize friction(ω) by stepping motor torque down at peak speed.

Phases:
  1. RAMP + ACCEL @ 45A (94s)  — get to ~260 motor RPM
  2. PUSH @ 55A (up to 30s)    — push higher; thermal abort at 78°C
  3. STEP-DOWN: 30A, 20A, 10A, 5A (15s each) — equilibrium at each gives T_friction(ω)
  4. COAST @ 0A (60s)          — pure friction sweep on coast-down

Aborts:
  - FET >= 78°C
  - |motor vel| > 9 t/s (540 RPM, well above target)
  - any ODrive error

Run BLE telemetry capture in parallel for trainer-side data.
"""
from __future__ import annotations
import math
import time
import odrive
from _logger import TestLogger

KT_NMA            = 0.04
DIRECTION         = -1
LOOP_DT_S         = 0.05

# Phase plan: list of (label, target_iq_A, duration_s)
PHASE_PLAN = [
    ("p1_ramp_accel", 45.0, 94.0),   # 4s ramp embedded then full hold
    ("p2_push",       55.0, 30.0),
    ("p3_step_30",    30.0, 15.0),
    ("p3_step_20",    20.0, 15.0),
    ("p3_step_10",    10.0, 15.0),
    ("p3_step_5",      5.0, 15.0),
    ("p4_coast",       0.0, 60.0),
]
RAMP_S            = 4.0   # only used for the first phase

VEL_ABORT_T_S     = 9.0   # 540 motor RPM
TEMP_WARN_C       = 70.0
TEMP_ABORT_C      = 78.0
CURRENT_LIM_A     = 70.0
DC_MAX_POS_CURRENT = 60.0

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

cc.control_mode = 1   # TORQUE
cc.input_mode = 1     # PASSTHROUGH

logger = TestLogger("push_to_500", extra_fields=["phase_iq_target"])

ax.controller.input_torque = 0
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"!! arm failed: state={ax.current_state} err={ax.error}")
print(f"\n== Armed in TORQUE mode ==")

# Per-phase results
phase_results = []

abort_reason = None
last_print = -1.0
peak_iq = 0.0
peak_temp = 0.0

t0 = time.time()
def now(): return time.time() - t0

def maybe_print(t, phase, iq_target, cmd, iq, vel, pos, temp, force=False):
    global last_print
    if force or t - last_print > 0.4:
        rpm = vel * 60.0
        print(f"   {t:7.2f} {phase:>14} {iq_target:>5.1f}A {cmd:+7.3f} {iq:+7.2f} {vel:+7.3f} ({rpm:+5.0f} RPM) {pos:+9.2f} {temp:5.1f}")
        last_print = t

def sample_and_log(phase, iq_target, cmd):
    t = now()
    vel = ax.encoder.vel_estimate
    pos = ax.encoder.pos_estimate
    iq = ax.motor.current_control.Iq_measured
    temp = ax.fet_thermistor.temperature
    aerr = ax.error; merr = ax.motor.error; eerr = ax.encoder.error
    logger.row(t_s=t, phase=phase, cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
               pos_turns=pos, fet_c=temp, axis_err=aerr,
               motor_err=merr, encoder_err=eerr,
               phase_iq_target=iq_target)
    global peak_iq, peak_temp
    if abs(iq) > abs(peak_iq): peak_iq = iq
    if temp > peak_temp: peak_temp = temp
    return t, vel, pos, iq, temp, aerr | merr | eerr

print(f"\n   {'t(s)':>7} {'phase':>14} {'tgt':>5} {'cmd':>7} {'Iq':>7} {'vel':>7}        {'pos':>9} {'FET':>5}")

try:
    for phase_idx, (phase_label, target_iq, duration) in enumerate(PHASE_PLAN):
        target_T = DIRECTION * target_iq * KT_NMA
        phase_start_t = now()
        phase_start_pos = ax.encoder.pos_estimate
        phase_start_vel = ax.encoder.vel_estimate
        print(f"\n--- PHASE {phase_idx+1}/{len(PHASE_PLAN)}: {phase_label}  target={target_iq}A ({target_T:+.2f} Nm)  for {duration}s ---")

        phase_end_wall = time.time() + duration
        while time.time() < phase_end_wall:
            elapsed_in_phase = time.time() - (phase_end_wall - duration)
            # Embed ramp into phase 1 only
            if phase_idx == 0 and elapsed_in_phase < RAMP_S:
                cmd = target_T * (elapsed_in_phase / RAMP_S)
            else:
                cmd = target_T
            ax.controller.input_torque = cmd
            t, vel, pos, iq, temp, errs = sample_and_log(phase_label, target_iq, cmd)
            maybe_print(t, phase_label, target_iq, cmd, iq, vel, pos, temp)
            if errs:
                abort_reason = f"err during {phase_label}: ax={ax.error} m={ax.motor.error}"; break
            if abs(vel) > VEL_ABORT_T_S:
                abort_reason = f"vel limit during {phase_label} ({vel*60:.0f} RPM)"; break
            if temp >= TEMP_ABORT_C:
                abort_reason = f"FET {temp:.1f}C during {phase_label}"; break
            time.sleep(LOOP_DT_S)

        phase_end_t = now()
        phase_end_pos = ax.encoder.pos_estimate
        phase_end_vel = ax.encoder.vel_estimate
        avg_rpm = abs(phase_end_pos - phase_start_pos) * 60.0 / max(phase_end_t - phase_start_t, 0.001)
        print(f"  -> end: vel={phase_end_vel*60:+.0f} RPM, avg_rpm_during_phase={avg_rpm:.0f}, FET={ax.fet_thermistor.temperature:.1f}C")
        phase_results.append({
            "phase": phase_label,
            "target_iq": target_iq,
            "start_t": phase_start_t,
            "end_t": phase_end_t,
            "start_vel_t_s": phase_start_vel,
            "end_vel_t_s": phase_end_vel,
            "start_pos_turns": phase_start_pos,
            "end_pos_turns": phase_end_pos,
            "avg_rpm": avg_rpm,
            "end_fet_c": ax.fet_thermistor.temperature,
        })

        if abort_reason:
            print(f"  !! aborting remaining phases: {abort_reason}")
            break

finally:
    ax.controller.input_torque = 0
    time.sleep(0.3)
    ax.requested_state = 1
    time.sleep(0.3)

logger.close()

print(f"\n=== PHASE SUMMARY ===")
print(f"  {'phase':>14} {'target':>7} {'start_RPM':>10} {'end_RPM':>9} {'avg_RPM':>9} {'FET_end':>8}")
for r in phase_results:
    print(f"  {r['phase']:>14} {r['target_iq']:>6.1f}A {r['start_vel_t_s']*60:>+10.0f} {r['end_vel_t_s']*60:>+9.0f} {r['avg_rpm']:>9.0f} {r['end_fet_c']:>7.1f}C")
print(f"\n  peak Iq      : {peak_iq:+.2f} A")
print(f"  peak FET     : {peak_temp:.1f} C")
print(f"  abort_reason : {abort_reason if abort_reason else 'all phases completed'}")

logger.plot(title="Push to 500 + step-down friction sweep",
            subtitle=f"phases stitched; peak FET {peak_temp:.1f}°C, abort={abort_reason or 'none'}")
print(f"  CSV: {logger.csv_path}")
print(f"  PNG: {logger.png_path}")
