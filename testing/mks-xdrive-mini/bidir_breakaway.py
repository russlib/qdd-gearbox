"""Bidirectional torque ramp 0 -> 1.0 Nm (0-25A) over 5s, in both directions.
Aborts immediately if rotor exceeds 300 RPM (5.0 t/s).
30ms loop for fast velocity-abort response (low-inertia freewheel direction)."""
import time
import odrive

VEL_ABORT_T_S    = 5.0       # 300 RPM
T_PEAK_NM        = 1.00      # 25 A * 0.04 Nm/A
T_RAMP_S         = 5.0       # ramp duration
LOOP_DT_S        = 0.03      # 30 ms loop
SETTLE_S         = 1.0       # idle settle between directions
CURRENT_LIM_A    = 25.0      # well under MKS Mini ~30A continuous

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

print(f"== Pre-test state ==")
print(f"  bus={odrv.vbus_voltage:.2f} V  state={ax.current_state}")
print(f"  encoder.offset = {ax.encoder.config.offset}  is_ready={ax.encoder.is_ready}")
print(f"  motor.is_calibrated = {ax.motor.is_calibrated}")

# Disable watchdog, clear errors
ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

# Bump current limits
mc.current_lim = CURRENT_LIM_A
# requested_current_range must be >= current_lim or controller silently caps
try:
    mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.2)
except Exception:
    pass
# DC bus headroom — needs to be >= peak current draw
try:
    odrv.config.dc_max_positive_current = 30.0
except Exception:
    pass
print(f"\n== Limits ==")
print(f"  motor.current_lim            = {mc.current_lim} A")
print(f"  motor.requested_current_range= {getattr(mc, 'requested_current_range', '?')} A")

cc.control_mode = 1   # TORQUE
cc.input_mode = 1     # PASSTHROUGH

def run_direction(sign, label):
    print(f"\n== {label} direction (sign={sign:+d}) ==")
    print(f"   Ramp 0 -> {sign*T_PEAK_NM:+.2f} Nm over {T_RAMP_S}s, abort if |vel| > {VEL_ABORT_T_S} t/s ({VEL_ABORT_T_S*60:.0f} RPM)")

    # Make sure starting from idle
    ax.controller.input_torque = 0
    ax.requested_state = 1
    time.sleep(0.3)

    # Arm
    ax.requested_state = 8
    deadline = time.time() + 3.0
    while time.time() < deadline and ax.current_state != 8:
        time.sleep(0.05)
    if ax.current_state != 8:
        print(f"   !! arm failed: state={ax.current_state} err={ax.error}")
        return None

    breakaway_T = None
    breakaway_iq = None
    peak_iq = 0.0
    aborted_at_vel = None
    aborted_at_T = None
    rows = []

    t0 = time.time()
    # Print header
    print(f"   {'t(s)':>5} {'cmd':>7} {'iq':>7} {'vel':>8} {'pos':>8}")

    last_print = 0.0
    while True:
        t = time.time() - t0
        if t >= T_RAMP_S:
            break
        target_T = sign * T_PEAK_NM * (t / T_RAMP_S)
        ax.controller.input_torque = target_T

        vel = ax.encoder.vel_estimate
        pos = ax.encoder.pos_estimate
        iq  = ax.motor.current_control.Iq_measured
        if abs(iq) > abs(peak_iq):
            peak_iq = iq

        if breakaway_T is None and abs(vel) > 0.05:
            breakaway_T = target_T
            breakaway_iq = iq

        # Print every ~150 ms to keep log readable
        if t - last_print > 0.15:
            print(f"   {t:5.2f} {target_T:+7.3f} {iq:+7.2f} {vel:+8.3f} {pos:+8.2f}")
            last_print = t

        # Abort fast on overspeed
        if abs(vel) > VEL_ABORT_T_S:
            aborted_at_vel = vel
            aborted_at_T = target_T
            ax.controller.input_torque = 0
            print(f"   !! ABORT — |vel|={abs(vel):.2f} t/s ({abs(vel)*60:.0f} RPM) > {VEL_ABORT_T_S} t/s, torque cut to 0")
            break

        time.sleep(LOOP_DT_S)

    # Decelerate cleanly
    ax.controller.input_torque = 0
    time.sleep(0.5)
    final_vel = ax.encoder.vel_estimate
    ax.requested_state = 1
    time.sleep(0.3)

    print(f"   peak_iq={peak_iq:+.2f} A  breakaway: T={breakaway_T} iq={breakaway_iq}  abort: vel={aborted_at_vel} T={aborted_at_T}  final_vel={final_vel:.3f}")
    return {
        "peak_iq": peak_iq,
        "breakaway_T": breakaway_T,
        "breakaway_iq": breakaway_iq,
        "aborted_at_vel": aborted_at_vel,
        "aborted_at_T": aborted_at_T,
        "final_vel": final_vel,
    }

result_pos = run_direction(+1, "POSITIVE")

print(f"\n   settling {SETTLE_S}s...")
time.sleep(SETTLE_S)

result_neg = run_direction(-1, "NEGATIVE")

print(f"\n=== SUMMARY ===")
def fmt(r):
    if not r:
        return "  (failed)"
    bt = r['breakaway_T']
    bi = r['breakaway_iq']
    av = r['aborted_at_vel']
    return (f"  breakaway  : {bt:+.3f} Nm @ Iq={bi:+.2f} A" if bt is not None else "  breakaway  : not seen") + \
           (f"\n  abort vel  : {av:+.2f} t/s ({av*60:+.0f} RPM)" if av is not None else "\n  abort vel  : not hit") + \
           f"\n  peak Iq    : {r['peak_iq']:+.2f} A"

print("POSITIVE:")
print(fmt(result_pos))
print("NEGATIVE:")
print(fmt(result_neg))

ax.requested_state = 1
print("\nReturned to IDLE.")
