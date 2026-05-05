"""Hold motor at 30A loaded direction for the duration of an ERG sweep.
Mild thermal load (~45W in windings, FET stays well under 60°C).
Designed to run in parallel with ble_erg_sweep.py."""
from __future__ import annotations
import time
import odrive
from _logger import TestLogger

KT_NMA            = 0.04
DIRECTION         = -1
TARGET_IQ_A       = 30.0
RAMP_S            = 4.0
HOLD_S            = 105.0
LOOP_DT_S         = 0.05
VEL_ABORT_T_S     = 9.0
TEMP_ABORT_C      = 78.0
CURRENT_LIM_A     = 60.0
DC_MAX_POS_CURRENT = 60.0

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

print(f"Pre-test: bus={odrv.vbus_voltage:.2f}V state={ax.current_state} FET={ax.fet_thermistor.temperature:.1f}C")

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

logger = TestLogger("long_hold_30a_erg")
target_T = DIRECTION * TARGET_IQ_A * KT_NMA

ax.controller.input_torque = 0
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"arm failed: state={ax.current_state} err={ax.error}")
print(f"Armed. Ramp {RAMP_S}s -> {target_T:+.2f} Nm, hold {HOLD_S}s.")

abort_reason = None
last_print = -1.0
peak_iq = 0.0; peak_temp = 0.0

t0 = time.time()
def now(): return time.time() - t0

def sample_log(phase, cmd):
    t = now()
    vel = ax.encoder.vel_estimate
    pos = ax.encoder.pos_estimate
    iq = ax.motor.current_control.Iq_measured
    temp = ax.fet_thermistor.temperature
    aerr = ax.error; merr = ax.motor.error; eerr = ax.encoder.error
    logger.row(t_s=t, phase=phase, cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
               pos_turns=pos, fet_c=temp, axis_err=aerr,
               motor_err=merr, encoder_err=eerr)
    global peak_iq, peak_temp
    if abs(iq) > abs(peak_iq): peak_iq = iq
    if temp > peak_temp: peak_temp = temp
    return t, vel, pos, iq, temp, aerr | merr | eerr

try:
    ramp_start = time.time()
    while time.time() - ramp_start < RAMP_S:
        cmd = target_T * (time.time() - ramp_start) / RAMP_S
        ax.controller.input_torque = cmd
        t, vel, pos, iq, temp, errs = sample_log("ramp", cmd)
        if t - last_print > 0.5:
            print(f"  t={t:6.2f}s ramp  cmd={cmd:+.2f} Iq={iq:+5.1f} vel={vel*60:+4.0f}RPM FET={temp:.1f}")
            last_print = t
        if errs: abort_reason = f"err during ramp"; break
        if abs(vel) > VEL_ABORT_T_S: abort_reason = f"vel limit during ramp"; break
        if temp >= TEMP_ABORT_C: abort_reason = f"FET {temp:.1f}C ramp"; break
        time.sleep(LOOP_DT_S)

    if abort_reason is None:
        hold_end = time.time() + HOLD_S
        while time.time() < hold_end:
            ax.controller.input_torque = target_T
            t, vel, pos, iq, temp, errs = sample_log("hold", target_T)
            if t - last_print > 1.0:
                print(f"  t={t:6.2f}s hold  cmd={target_T:+.2f} Iq={iq:+5.1f} vel={vel*60:+4.0f}RPM FET={temp:.1f}")
                last_print = t
            if errs: abort_reason = f"err during hold"; break
            if abs(vel) > VEL_ABORT_T_S: abort_reason = f"vel limit during hold"; break
            if temp >= TEMP_ABORT_C: abort_reason = f"FET {temp:.1f}C hold"; break
            time.sleep(LOOP_DT_S)
finally:
    ax.controller.input_torque = 0
    time.sleep(0.3)
    ax.requested_state = 1

logger.close()

print(f"\n=== RESULT ===")
print(f"  abort: {abort_reason or 'completed'}")
print(f"  peak Iq: {peak_iq:+.2f}A  peak FET: {peak_temp:.1f}C")
logger.plot(title=f"30A hold during ERG sweep",
            subtitle=f"peak FET {peak_temp:.1f}°C")
print(f"  CSV: {logger.csv_path}")
print(f"  PNG: {logger.png_path}")
