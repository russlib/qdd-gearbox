"""90s hold at 45A loaded direction with cooldown logging.
Uses shared TestLogger. Designed to run in parallel with BLE capture for cross-correlation."""
from __future__ import annotations
import math
import time
import odrive
from _logger import TestLogger

# ---- knobs ----
KT_NMA            = 0.04
DIRECTION         = -1
TARGET_IQ_A       = 45.0
RAMP_S            = 4.0
HOLD_S            = 90.0
COOLDOWN_S        = 0.0   # bump back to 60-180s when fresh thermal cooling data is wanted
LOOP_DT_S         = 0.05
COOL_LOG_DT_S     = 0.5
VEL_ABORT_T_S     = 5.5         # 330 RPM motor (~1900 flywheel RPM via 5.8:1)
TEMP_WARN_C       = 70.0
TEMP_ABORT_C      = 78.0        # tighter, since we expect ~74C
CURRENT_LIM_A     = 60.0
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

cc.control_mode = 1
cc.input_mode = 1

logger = TestLogger(f"long_hold_{int(TARGET_IQ_A)}A")
target_T = DIRECTION * TARGET_IQ_A * KT_NMA

ax.controller.input_torque = 0
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"!! arm failed: state={ax.current_state} err={ax.error}")
print(f"\n== Armed: ramp 0 -> {target_T:+.2f} Nm over {RAMP_S}s, hold {HOLD_S}s, cooldown {COOLDOWN_S}s ==")
print(f"   abort: |vel| > {VEL_ABORT_T_S} t/s  or  FET >= {TEMP_ABORT_C} C  or  any error")
print(f"\n   {'t(s)':>7} {'phase':>9} {'cmd':>7} {'Iq':>7} {'vel':>7} {'pos':>9} {'FET':>5}")

abort_reason = None
last_print = -1.0
peak_iq = 0.0
peak_temp = 0.0
pos_at_hold_start = None
pos_at_hold_end = None

t0 = time.time()
def now(): return time.time() - t0

def maybe_print(t, phase, cmd, iq, vel, pos, temp, force=False):
    global last_print
    if force or t - last_print > 0.5:
        print(f"   {t:7.2f} {phase:>9} {cmd:+7.3f} {iq:+7.2f} {vel:+7.3f} {pos:+9.2f} {temp:5.1f}")
        last_print = t

def sample_and_log(phase, cmd):
    t = now()
    vel = ax.encoder.vel_estimate
    pos = ax.encoder.pos_estimate
    iq = ax.motor.current_control.Iq_measured
    temp = ax.fet_thermistor.temperature
    aerr = ax.error
    merr = ax.motor.error
    eerr = ax.encoder.error
    logger.row(t_s=t, phase=phase, cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
               pos_turns=pos, fet_c=temp, axis_err=aerr,
               motor_err=merr, encoder_err=eerr)
    global peak_iq, peak_temp
    if abs(iq) > abs(peak_iq): peak_iq = iq
    if temp > peak_temp: peak_temp = temp
    return t, vel, pos, iq, temp, aerr | merr | eerr

try:
    # --- RAMP ---
    ramp_start = time.time()
    while time.time() - ramp_start < RAMP_S:
        cmd = target_T * (time.time() - ramp_start) / RAMP_S
        ax.controller.input_torque = cmd
        t, vel, pos, iq, temp, errs = sample_and_log("ramp", cmd)
        maybe_print(t, "ramp", cmd, iq, vel, pos, temp)
        if errs: abort_reason = f"err during ramp: ax={ax.error} m={ax.motor.error}"; break
        if abs(vel) > VEL_ABORT_T_S: abort_reason = f"vel limit during ramp"; break
        if temp >= TEMP_ABORT_C: abort_reason = f"FET {temp:.1f}C during ramp"; break
        time.sleep(LOOP_DT_S)

    if abort_reason is None:
        # --- HOLD ---
        pos_at_hold_start = ax.encoder.pos_estimate
        hold_start = time.time()
        while time.time() - hold_start < HOLD_S:
            ax.controller.input_torque = target_T
            t, vel, pos, iq, temp, errs = sample_and_log("hold", target_T)
            maybe_print(t, "hold", target_T, iq, vel, pos, temp)
            if errs: abort_reason = f"err during hold: ax={ax.error} m={ax.motor.error}"; break
            if abs(vel) > VEL_ABORT_T_S: abort_reason = f"vel limit during hold ({vel*60:.0f} RPM)"; break
            if temp >= TEMP_ABORT_C: abort_reason = f"FET {temp:.1f}C during hold"; break
            time.sleep(LOOP_DT_S)
        pos_at_hold_end = ax.encoder.pos_estimate

finally:
    ax.controller.input_torque = 0
    time.sleep(0.3)
    ax.requested_state = 1
    time.sleep(0.3)

# --- COOLDOWN ---
print(f"\n   --- entering COOLDOWN logging for {COOLDOWN_S}s ---")
last_print = -1.0
cooldown_end = time.time() + COOLDOWN_S
while time.time() < cooldown_end:
    t, vel, pos, iq, temp, errs = sample_and_log("cool", 0.0)
    if t - last_print > 2.0:
        print(f"   {t:7.2f} {'cool':>9} {0.0:+7.3f} {iq:+7.2f} {vel:+7.3f} {pos:+9.2f} {temp:5.1f}")
        last_print = t
    time.sleep(COOL_LOG_DT_S)

logger.close()

# Summary
print(f"\n=== RESULT ===")
print(f"  abort_reason : {abort_reason if abort_reason else 'completed full ramp+hold'}")
print(f"  peak Iq      : {peak_iq:+.2f} A")
print(f"  peak FET     : {peak_temp:.1f} C")
if pos_at_hold_start is not None and pos_at_hold_end is not None:
    delta = pos_at_hold_end - pos_at_hold_start
    print(f"  hold pos delta: {delta:+.3f} motor turns ({delta*5.8:+.1f} flywheel turns @ N=5.8)")
    avg_motor_rpm = abs(delta) * 60.0 / HOLD_S
    print(f"  avg motor RPM during hold: {avg_motor_rpm:.1f} RPM")
    print(f"  avg flywheel RPM (estim): {avg_motor_rpm*5.8:.0f} RPM")

logger.plot(title=f"Long hold at {int(TARGET_IQ_A)}A loaded direction",
            subtitle=f"hold {HOLD_S}s, cooldown {COOLDOWN_S}s, peak FET {peak_temp:.1f}°C")
print(f"  CSV: {logger.csv_path}")
print(f"  PNG: {logger.png_path}")
