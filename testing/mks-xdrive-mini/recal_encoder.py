"""Force a clean encoder offset recal. Reads new offset, optionally saves, then small torque test."""
import time
import odrive

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
ec = ax.encoder.config

print("== Pre-recal state ==")
print(f"  bus={odrv.vbus_voltage:.2f} V  state={ax.current_state}  axis.err={ax.error}")
print(f"  motor.pre_calibrated   = {mc.pre_calibrated}")
print(f"  encoder.pre_calibrated = {ec.pre_calibrated}")
print(f"  encoder.is_ready       = {ax.encoder.is_ready}")
print(f"  encoder.offset (saved) = {ec.offset}")
print(f"  motor.is_calibrated    = {ax.motor.is_calibrated}")
print(f"  Phase R                = {mc.phase_resistance*1000:.2f} mOhm")
print(f"  Phase L                = {mc.phase_inductance*1e6:.2f} uH")

# Disable watchdog, clear errors
ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

# Conservative current for cal
mc.current_lim = 10.0
mc.calibration_current = 5.0
mc.resistance_calib_max_voltage = 2.0

# Force fresh detection
print("\n== Forcing pre_calibrated = False on motor and encoder ==")
mc.pre_calibrated = False
ec.pre_calibrated = False

# Make sure axis is idle before requesting state 3
ax.requested_state = 1
time.sleep(0.3)

print("\n== Running state 3 (FULL_CALIBRATION_SEQUENCE) ==")
print("   motor R/L beep -> encoder direction find -> encoder offset find")
print("   Total ~15-25s. Rotor will spin slowly during offset search.")
ax.requested_state = 3

deadline = time.time() + 35.0
last_state = None
while time.time() < deadline:
    s = ax.current_state
    if s != last_state:
        print(f"  t={35.0-(deadline-time.time()):4.1f}s  state={s}  axis.err={ax.error}  motor.err={ax.motor.error}  enc.err={ax.encoder.error}")
        last_state = s
    if s == 1:
        break
    time.sleep(0.2)

print("\n== Post-cal state ==")
print(f"  state={ax.current_state}")
print(f"  axis.err    = {ax.error}")
print(f"  motor.err   = {ax.motor.error}")
print(f"  encoder.err = {ax.encoder.error}")
print(f"  motor.is_calibrated    = {ax.motor.is_calibrated}")
print(f"  encoder.is_ready       = {ax.encoder.is_ready}")
print(f"  encoder.offset (new)   = {ec.offset}")
print(f"  Phase R                = {mc.phase_resistance*1000:.2f} mOhm")
print(f"  Phase L                = {mc.phase_inductance*1e6:.2f} uH")

# Bail if cal failed
if ax.error or ax.motor.error or ax.encoder.error:
    print("\n!! CAL FAILED — not saving, not testing. Above errors are the diagnosis.")
    raise SystemExit(1)
if not ax.encoder.is_ready or not ax.motor.is_calibrated:
    print("\n!! encoder.is_ready or motor.is_calibrated still False — cal didn't complete cleanly.")
    raise SystemExit(1)

print("\n== Cal looks clean. Testing torque BEFORE saving (so we don't persist a bad cal) ==")
ax.controller.input_torque = 0
ax.controller.config.control_mode = 1  # TORQUE
ax.controller.config.input_mode = 1
ax.requested_state = 8
time.sleep(0.3)
if ax.current_state != 8:
    print(f"!! Failed to arm post-cal: state={ax.current_state} err={ax.error}")
    raise SystemExit(1)

# Small torque ramp 0 -> 0.10 Nm over 1s, should easily move with valid offset
print("Applying torque ramp 0 -> 0.10 Nm over 1.0s (peak Iq ~2.5 A)...")
T_max = 0.10
ramp_s = 1.0
t0 = time.time()
moved = False
while time.time() - t0 < ramp_s:
    t = time.time() - t0
    target = T_max * (t / ramp_s)
    ax.controller.input_torque = target
    vel = ax.encoder.vel_estimate
    pos = ax.encoder.pos_estimate
    iq  = ax.motor.current_control.Iq_measured
    print(f"  t={t:4.2f}s  cmd={target:.3f}  iq={iq:5.2f} A  vel={vel:6.3f} t/s  pos={pos:7.3f}")
    if abs(vel) > 0.01:
        moved = True
    time.sleep(0.1)

# Stop
ax.controller.input_torque = 0
time.sleep(0.5)
ax.requested_state = 1

print(f"\nFinal vel={ax.encoder.vel_estimate:.3f} t/s")
if moved:
    print("\n*** ROTOR MOVED — cal is good. ***")
    print("To save this cal so it persists across power cycles, re-run with --save (not yet implemented; will add).")
else:
    print("\n!! Rotor still didn't move. Encoder offset wasn't the issue — investigate phase wires or encoder coupling.")
