"""Hold 35 A motor torque (~1.4 Nm) in the loaded direction with FET temp monitoring.
Aborts on:
- FET temp >= TEMP_ABORT_C
- |vel| >= VEL_ABORT_T_S (rotor broke free, don't keep pushing)
- Total elapsed >= MAX_HOLD_S (timeout)
- Any ODrive error
"""
import sys
import time
import odrive

# ---- knobs ----
DIRECTION         = -1       # +1 freewheel, -1 loaded (where flywheel resists)
RAMP_TO_NM        = 35.0 * 0.04   # 35A at Kt=0.04 -> 1.4 Nm
RAMP_S            = 4.0      # ramp duration
HOLD_S            = 6.0      # hold-at-target duration
LOOP_DT_S         = 0.05     # 50 ms loop
VEL_ABORT_T_S     = 5.0      # 300 RPM
TEMP_WARN_C       = 65.0
TEMP_ABORT_C      = 80.0     # FET protection (firmware trip is ~100C)
CURRENT_LIM_A     = 40.0     # gives 5A headroom over 35A target
# total wall time hard cap
MAX_HOLD_S        = RAMP_S + HOLD_S + 2.0

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

# ---- find FET temp attribute (varies by FW build) ----
def find_temp_reader(odrv, ax):
    candidates = [
        ("axis0.fet_thermistor.temperature",        lambda: ax.fet_thermistor.temperature),
        ("axis0.motor.fet_thermistor.temperature",  lambda: ax.motor.fet_thermistor.temperature),
        ("odrv.fet_thermistor.temperature",         lambda: odrv.fet_thermistor.temperature),
        ("axis0.motor.thermal_state.fet_temp",      lambda: ax.motor.thermal_state.fet_temp),
    ]
    for name, fn in candidates:
        try:
            v = fn()
            if v is not None and isinstance(v, (int, float)):
                print(f"  FET temp source: {name} = {v:.1f} C")
                return name, fn
        except Exception:
            continue
    return None, None

print("== Pre-test ==")
print(f"  bus={odrv.vbus_voltage:.2f} V  state={ax.current_state}  encoder.offset={ax.encoder.config.offset}")
temp_name, get_temp = find_temp_reader(odrv, ax)
if get_temp is None:
    print("  WARN: no FET temp attribute found in this FW. Continuing without thermal abort.")
else:
    print(f"  Initial FET temp: {get_temp():.1f} C")

# Disable watchdog, clear errors
ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

# Set limits
mc.current_lim = CURRENT_LIM_A
try:
    mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.3)
except Exception:
    pass
try:
    odrv.config.dc_max_positive_current = max(getattr(odrv.config, 'dc_max_positive_current', 0), 35.0)
except Exception:
    pass

cc.control_mode = 1   # TORQUE
cc.input_mode = 1     # PASSTHROUGH

# Arm
ax.controller.input_torque = 0
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"!! arm failed: state={ax.current_state} err={ax.error}")
print(f"\n== Armed in TORQUE mode, direction={'LOADED' if DIRECTION<0 else 'FREEWHEEL'} ==")
print(f"   ramp 0 -> {DIRECTION*RAMP_TO_NM:+.2f} Nm over {RAMP_S}s  then hold {HOLD_S}s")
print(f"   abort: |vel| > {VEL_ABORT_T_S} t/s ({VEL_ABORT_T_S*60:.0f} RPM)  or  FET >= {TEMP_ABORT_C} C  or  any error")

print(f"\n   {'t(s)':>5} {'phase':>8} {'cmd Nm':>8} {'Iq A':>7} {'vel t/s':>8} {'pos':>8} {'FET C':>6} {'errs':>6}")

abort_reason = None
peak_iq = 0.0
peak_temp = 0.0
final_pos_start = ax.encoder.pos_estimate
last_print = -1.0

t0 = time.time()
def now():
    return time.time() - t0

try:
    # Ramp
    while now() < RAMP_S:
        t = now()
        target_T = DIRECTION * RAMP_TO_NM * (t / RAMP_S)
        ax.controller.input_torque = target_T

        vel = ax.encoder.vel_estimate
        pos = ax.encoder.pos_estimate
        iq  = ax.motor.current_control.Iq_measured
        if abs(iq) > abs(peak_iq): peak_iq = iq
        temp = get_temp() if get_temp else None
        if temp is not None and temp > peak_temp: peak_temp = temp
        errs = ax.error | ax.motor.error | ax.encoder.error

        if t - last_print > 0.20:
            tdisp = f"{temp:6.1f}" if temp is not None else "  ---"
            print(f"   {t:5.2f} {'ramp':>8} {target_T:+8.3f} {iq:+7.2f} {vel:+8.3f} {pos:+8.2f} {tdisp} {errs:>6}")
            last_print = t

        if errs:
            abort_reason = f"ODrive error: axis={ax.error} motor={ax.motor.error} enc={ax.encoder.error}"
            break
        if abs(vel) > VEL_ABORT_T_S:
            abort_reason = f"vel {vel*60:.0f} RPM > {VEL_ABORT_T_S*60:.0f} RPM"
            break
        if temp is not None and temp >= TEMP_ABORT_C:
            abort_reason = f"FET temp {temp:.1f} C >= {TEMP_ABORT_C} C"
            break
        time.sleep(LOOP_DT_S)

    # Hold
    if abort_reason is None:
        hold_end = time.time() + HOLD_S
        while time.time() < hold_end:
            t = now()
            ax.controller.input_torque = DIRECTION * RAMP_TO_NM

            vel = ax.encoder.vel_estimate
            pos = ax.encoder.pos_estimate
            iq  = ax.motor.current_control.Iq_measured
            if abs(iq) > abs(peak_iq): peak_iq = iq
            temp = get_temp() if get_temp else None
            if temp is not None and temp > peak_temp: peak_temp = temp
            errs = ax.error | ax.motor.error | ax.encoder.error

            if t - last_print > 0.20:
                tdisp = f"{temp:6.1f}" if temp is not None else "  ---"
                temp_warn = "  WARN" if (temp is not None and temp >= TEMP_WARN_C) else ""
                print(f"   {t:5.2f} {'hold':>8} {DIRECTION*RAMP_TO_NM:+8.3f} {iq:+7.2f} {vel:+8.3f} {pos:+8.2f} {tdisp} {errs:>6}{temp_warn}")
                last_print = t

            if errs:
                abort_reason = f"ODrive error: axis={ax.error} motor={ax.motor.error} enc={ax.encoder.error}"
                break
            if abs(vel) > VEL_ABORT_T_S:
                abort_reason = f"vel {vel*60:.0f} RPM > {VEL_ABORT_T_S*60:.0f} RPM"
                break
            if temp is not None and temp >= TEMP_ABORT_C:
                abort_reason = f"FET temp {temp:.1f} C >= {TEMP_ABORT_C} C"
                break
            if now() > MAX_HOLD_S:
                abort_reason = "wall-time hard cap"
                break
            time.sleep(LOOP_DT_S)

finally:
    # Clean shutdown ALWAYS
    ax.controller.input_torque = 0
    time.sleep(0.5)
    ax.requested_state = 1
    time.sleep(0.3)

final_pos = ax.encoder.pos_estimate
delta_turns = final_pos - final_pos_start
delta_deg   = delta_turns * 360.0
final_temp  = get_temp() if get_temp else None

print(f"\n=== RESULT ===")
print(f"  abort_reason : {abort_reason if abort_reason else 'completed full ramp+hold'}")
print(f"  peak Iq      : {peak_iq:+.2f} A")
print(f"  peak FET temp: {peak_temp:.1f} C" if peak_temp else "  peak FET temp: n/a")
print(f"  final FET temp: {final_temp:.1f} C" if final_temp is not None else "  final FET temp: n/a")
print(f"  position delta: {delta_turns:+.4f} turns ({delta_deg:+.2f} deg) -- meaningful motion if > 1 turn")
print(f"\nReturned to IDLE.")
