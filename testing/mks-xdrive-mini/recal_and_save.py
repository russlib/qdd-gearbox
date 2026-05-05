"""Run state 3 cal, verify clean, save to flash. After this the cal persists across power cycles."""
import time
import odrive

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
ec = ax.encoder.config

print(f"== Pre-cal ==")
print(f"  bus = {odrv.vbus_voltage:.2f} V")
print(f"  encoder.offset (saved) = {ec.offset}")
print(f"  motor.pre_calibrated   = {mc.pre_calibrated}")
print(f"  encoder.pre_calibrated = {ec.pre_calibrated}")

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
mc.pre_calibrated = False
ec.pre_calibrated = False

ax.requested_state = 1
time.sleep(0.3)

print("\n== Running state 3 (motor cal -> encoder dir -> encoder offset) ==")
ax.requested_state = 3
deadline = time.time() + 35.0
last_state = None
while time.time() < deadline:
    s = ax.current_state
    if s != last_state:
        elapsed = 35.0 - (deadline - time.time())
        print(f"  t={elapsed:4.1f}s  state={s}  axis.err={ax.error}  motor.err={ax.motor.error}  enc.err={ax.encoder.error}")
        last_state = s
    if s == 1:
        break
    time.sleep(0.2)

print(f"\n== Post-cal ==")
print(f"  state = {ax.current_state}")
print(f"  errors: axis={ax.error}  motor={ax.motor.error}  encoder={ax.encoder.error}")
print(f"  encoder.offset = {ec.offset}")
print(f"  motor.is_calibrated = {ax.motor.is_calibrated}")
print(f"  encoder.is_ready    = {ax.encoder.is_ready}")
print(f"  Phase R = {mc.phase_resistance*1000:.2f} mOhm  L = {mc.phase_inductance*1e6:.2f} uH")

if ax.error or ax.motor.error or ax.encoder.error:
    print("\n!! cal failed — NOT saving. Power-cycle and try again.")
    raise SystemExit(1)
if not ax.motor.is_calibrated or not ax.encoder.is_ready:
    print("\n!! cal incomplete — NOT saving.")
    raise SystemExit(1)

# Mark as pre-calibrated so the saved values are trusted on next boot
mc.pre_calibrated = True
ec.pre_calibrated = True

print("\n== Saving configuration to flash ==")
print("   The board will reboot. DeviceLostException is expected.")
try:
    odrv.save_configuration()
    print("   saved.")
except Exception as e:
    # save_configuration triggers a reboot, which is normally seen as DeviceLostException
    print(f"   reboot signaled ({type(e).__name__}): {e}")

# Reconnect to verify
print("\n== Reconnecting to verify saved values ==")
time.sleep(2.0)
odrv = odrive.find_any(timeout=20)
if odrv is None:
    print("   warn: did not reconnect cleanly. Power-cycle and verify manually.")
    raise SystemExit(0)

ax = odrv.axis0
print(f"  bus = {odrv.vbus_voltage:.2f} V  state = {ax.current_state}")
print(f"  encoder.offset (saved)   = {ax.encoder.config.offset}")
print(f"  motor.pre_calibrated     = {ax.motor.config.pre_calibrated}")
print(f"  encoder.pre_calibrated   = {ax.encoder.config.pre_calibrated}")
print(f"  Phase R = {ax.motor.config.phase_resistance*1000:.2f} mOhm")
print("\n   ✓ saved. Power cycles will now use this cal.")
