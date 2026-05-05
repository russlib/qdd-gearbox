"""Torque-mode breakaway test. Apply known torque, watch if rotor moves."""
import time
import odrive

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

print(f"== Initial state ==")
print(f"  bus={odrv.vbus_voltage:.2f} V  state={ax.current_state}  err axis={ax.error} motor={ax.motor.error} enc={ax.encoder.error}")

# Disable watchdog, clear errors
ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

# Conservative current cap for this test
mc.current_lim = 19.0
print(f"  current_lim set to {mc.current_lim} A")

# Set torque control mode BEFORE arming
ax.controller.input_vel = 0
ax.controller.input_torque = 0
cc.control_mode = 1  # TORQUE
cc.input_mode = 1    # PASSTHROUGH

# Arm
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"Arm failed: state={ax.current_state} err={ax.error}")
print(f"== Armed in TORQUE mode ==")

# Test sequence: ramp torque from 0 to 0.20 Nm over 2s, hold for 1s, ramp back to 0
# Kt = 0.04 Nm/A -> 0.20 Nm = 5A, well under current_lim
T_max = 0.20  # Nm
ramp_up_s = 2.0
hold_s = 1.0
ramp_dn_s = 1.0

print(f"\n== Ramping torque 0 -> {T_max} Nm over {ramp_up_s}s, hold {hold_s}s, ramp down {ramp_dn_s}s ==")
print(f"   (Kt=0.04 -> peak Iq ~{T_max/0.04:.1f} A)")

t0 = time.time()
breakaway_seen = False
breakaway_t = None
breakaway_iq = None

def log_row(t, phase, target_T):
    global breakaway_seen, breakaway_t, breakaway_iq
    vel = ax.encoder.vel_estimate
    pos = ax.encoder.pos_estimate
    iq  = ax.motor.current_control.Iq_measured
    if not breakaway_seen and abs(vel) > 0.01:
        breakaway_seen = True
        breakaway_t = t
        breakaway_iq = iq
        marker = " *** BREAKAWAY ***"
    else:
        marker = ""
    print(f"  t={t:5.2f}s  {phase:8s} cmd={target_T:5.3f} Nm  iq={iq:6.2f} A  vel={vel:7.3f} t/s  pos={pos:8.3f}  err={ax.error}{marker}")

# Ramp up
while True:
    t = time.time() - t0
    if t >= ramp_up_s:
        break
    target_T = T_max * (t / ramp_up_s)
    ax.controller.input_torque = target_T
    log_row(t, "ramp-up", target_T)
    time.sleep(0.1)

# Hold
hold_start = time.time()
while time.time() - hold_start < hold_s:
    t = time.time() - t0
    ax.controller.input_torque = T_max
    log_row(t, "hold", T_max)
    time.sleep(0.1)

# Ramp down
dn_start = time.time()
while time.time() - dn_start < ramp_dn_s:
    t = time.time() - t0
    target_T = T_max * (1.0 - (time.time() - dn_start) / ramp_dn_s)
    ax.controller.input_torque = max(0.0, target_T)
    log_row(t, "ramp-dn", target_T)
    time.sleep(0.1)

# Stop
ax.controller.input_torque = 0
time.sleep(0.3)
ax.requested_state = 1

print(f"\n== Result ==")
if breakaway_seen:
    print(f"  Rotor broke free at t={breakaway_t:.2f}s with iq={breakaway_iq:.2f} A -> ~{breakaway_iq*0.04:.3f} Nm")
else:
    print(f"  Rotor never moved. Final iq={ax.motor.current_control.Iq_measured:.2f} A")
    print(f"  -> Either cogging detent stronger than {T_max} Nm, or something mechanically locked, or encoder is reading wrong.")
print("Returned to IDLE.")
