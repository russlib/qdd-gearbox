"""Diagnose why axis won't enter closed loop. No motion. Read-only."""
import time
import odrive

def safe(o, attr, default=None):
    try:
        return getattr(o, attr)
    except Exception:
        return default

odrv = odrive.find_any(timeout=15)
ax = odrv.axis0

print("== Pre-arm state ==")
print(f"  bus_voltage         = {odrv.vbus_voltage:.2f} V")
print(f"  axis.current_state  = {ax.current_state}")
print(f"  axis.error          = {ax.error}")
print(f"  motor.error         = {ax.motor.error}")
print(f"  encoder.error       = {ax.encoder.error}")
print(f"  controller.error    = {safe(ax.controller, 'error', 0)}")
print(f"  motor.is_calibrated     = {safe(ax.motor.config, 'pre_calibrated', None)} (pre_calibrated)")
print(f"  motor.is_calibrated     = {safe(ax.motor, 'is_calibrated', None)} (runtime)")
print(f"  encoder.is_ready        = {safe(ax.encoder, 'is_ready', None)}")
print(f"  encoder.pre_calibrated  = {safe(ax.encoder.config, 'pre_calibrated', None)}")
print(f"  encoder.pos_estimate    = {safe(ax.encoder, 'pos_estimate', None)}")
print(f"  encoder.shadow_count    = {safe(ax.encoder, 'shadow_count', None)}")
print(f"  encoder.count_in_cpr    = {safe(ax.encoder, 'count_in_cpr', None)}")
print(f"  encoder.config.cpr      = {ax.encoder.config.cpr}")
print(f"  encoder.config.mode     = {ax.encoder.config.mode}")
print(f"  motor.config.motor_type = {ax.motor.config.motor_type}")
print(f"  motor.config.pole_pairs = {ax.motor.config.pole_pairs}")
print(f"  motor.config.phase_R    = {ax.motor.config.phase_resistance*1000:.2f} mOhm")
print(f"  motor.config.phase_L    = {ax.motor.config.phase_inductance*1e6:.2f} uH")
print(f"  controller.control_mode = {ax.controller.config.control_mode}")
print(f"  controller.input_mode   = {ax.controller.config.input_mode}")
print(f"  controller.input_vel    = {ax.controller.input_vel}")
print(f"  watchdog_enabled        = {ax.config.enable_watchdog}")
print(f"  watchdog_timeout        = {ax.config.watchdog_timeout}")

print("\n== Clearing errors ==")
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try:
        tgt.error = 0
    except Exception:
        pass
print(f"  axis.error      = {ax.error}")
print(f"  motor.error     = {ax.motor.error}")
print(f"  encoder.error   = {ax.encoder.error}")

print("\n== Disabling watchdog (avoid timeout during diag) ==")
ax.config.enable_watchdog = False

print("\n== Requesting CLOSED_LOOP_CONTROL (state 8) ==")
ax.controller.input_vel = 0
ax.controller.config.control_mode = 2  # VELOCITY
ax.controller.config.input_mode = 1
ax.requested_state = 8

print("Waiting up to 3s for state transition...")
deadline = time.time() + 3.0
last = None
while time.time() < deadline:
    cs = ax.current_state
    if cs != last:
        print(f"  t={time.time()-(deadline-3.0):4.2f}s  current_state={cs}  err={ax.error}")
        last = cs
    if cs == 8:
        break
    time.sleep(0.05)

print("\n== Post-arm state ==")
print(f"  axis.current_state  = {ax.current_state}")
print(f"  axis.error          = {ax.error}")
print(f"  motor.error         = {ax.motor.error}")
print(f"  encoder.error       = {ax.encoder.error}")
print(f"  controller.error    = {safe(ax.controller, 'error', 0)}")

# Force back to idle for safety
ax.requested_state = 1
print("\n  Returned to IDLE.")
