"""3-second motor pulse to spin up the flywheel and wake the H2 BLE radio."""
import time
import odrive

DIRECTION = -1
TARGET_NM = 1.0   # 25A
PULSE_S   = 3.0
KT_NMA    = 0.04

odrv = odrive.find_any(timeout=15)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

mc.current_lim = 30.0
cc.control_mode = 1
cc.input_mode = 1

ax.controller.input_torque = 0
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"arm failed: state={ax.current_state} err={ax.error}")

print(f"Pulsing {DIRECTION*TARGET_NM:+.2f} Nm for {PULSE_S}s to spin up flywheel...")
ax.controller.input_torque = DIRECTION * TARGET_NM
t0 = time.time()
while time.time() - t0 < PULSE_S:
    vel = ax.encoder.vel_estimate
    print(f"  t={time.time()-t0:4.2f}s  vel={vel*60:.1f} motor RPM (~{abs(vel*60*5.8):.0f} flywheel RPM)")
    time.sleep(0.5)

ax.controller.input_torque = 0
ax.requested_state = 1
print("Wake pulse done — flywheel coasting, BLE should be awake.")
