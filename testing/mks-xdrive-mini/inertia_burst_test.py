"""Multi-burst inertia identification test.
Apply torque steps at multiple Iq levels, measure steady-state alpha after breakaway.
Linear fit alpha(T) -> slope = 1/J, intercept = -T_friction/J -> isolate both.

Each burst:
- ramp 0 -> target T over 1.5s  (to avoid step shock)
- hold target T for 4s          (capture steady alpha at end)
- idle 12s                      (let flywheel coast / thermal cooldown)

Aborts on FET >= 80 C, |vel| > 5 t/s, or any ODrive error.
"""
import csv
import math
import time
from pathlib import Path
import odrive

KT_NMA            = 0.04       # conservative — analysis uses Iq, so Kt assumption only matters for label
DIRECTION         = -1         # loaded direction
BURST_CURRENTS_A  = [25, 35, 45, 55]
RAMP_S            = 1.5
HOLD_S            = 4.0
IDLE_S            = 12.0
LOOP_DT_S         = 0.05
VEL_ABORT_T_S     = 5.0
TEMP_WARN_C       = 65.0
TEMP_ABORT_C      = 80.0
CURRENT_LIM_A     = 60.0
DC_MAX_POS_CURRENT = 60.0

OUT_DIR = Path(__file__).parent / "thermal_data"
OUT_DIR.mkdir(exist_ok=True)
ts = time.strftime("%Y%m%d-%H%M%S")
CSV_PATH = OUT_DIR / f"inertia_burst_{ts}.csv"
PNG_PATH = OUT_DIR / f"inertia_burst_{ts}.png"

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

print("== Pre-test ==")
print(f"  bus={odrv.vbus_voltage:.2f} V  state={ax.current_state}  encoder.offset={ax.encoder.config.offset}")
print(f"  initial FET = {ax.fet_thermistor.temperature:.1f} C")

# Disable watchdog, clear errors
ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

mc.current_lim = CURRENT_LIM_A
try:
    mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.3)
except: pass
try:
    odrv.config.dc_max_positive_current = DC_MAX_POS_CURRENT
except: pass

cc.control_mode = 1   # TORQUE
cc.input_mode = 1     # PASSTHROUGH

# CSV setup
csv_file = open(CSV_PATH, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["t_s", "phase", "burst_iq_target", "cmd_nm", "iq_a", "vel_t_s", "pos_turns", "fet_c"])

# Arm
ax.controller.input_torque = 0
ax.requested_state = 8
deadline = time.time() + 3.0
while time.time() < deadline and ax.current_state != 8:
    time.sleep(0.05)
if ax.current_state != 8:
    raise SystemExit(f"!! arm failed: state={ax.current_state} err={ax.error}")
print("  armed in TORQUE mode")

t0 = time.time()
def now(): return time.time() - t0

burst_results = []  # (target_iq, vel_start, vel_end, hold_dt, alpha)

abort_reason = None

print(f"\n   {'t(s)':>6} {'phase':>10} {'tgt':>4} {'cmd':>7} {'Iq':>7} {'vel':>7} {'pos':>9} {'FET':>5}")

def sample(phase, target_iq, cmd_nm):
    t = now()
    vel = ax.encoder.vel_estimate
    pos = ax.encoder.pos_estimate
    iq  = ax.motor.current_control.Iq_measured
    temp = ax.fet_thermistor.temperature
    aerr = ax.error; merr = ax.motor.error; eerr = ax.encoder.error
    writer.writerow([f"{t:.3f}", phase, target_iq if target_iq else "", f"{cmd_nm:.4f}",
                     f"{iq:.3f}", f"{vel:.4f}", f"{pos:.4f}", f"{temp:.2f}"])
    return t, vel, pos, iq, temp, aerr | merr | eerr

last_print = -1.0
def maybe_print(t, phase, target_iq, cmd, iq, vel, pos, temp):
    global last_print
    if t - last_print > 0.2:
        print(f"   {t:6.2f} {phase:>10} {target_iq:>4} {cmd:+7.3f} {iq:+7.2f} {vel:+7.3f} {pos:+9.2f} {temp:5.1f}")
        last_print = t

try:
    for burst_idx, target_iq in enumerate(BURST_CURRENTS_A):
        target_T = DIRECTION * target_iq * KT_NMA
        print(f"\n--- BURST {burst_idx+1}/{len(BURST_CURRENTS_A)}: target Iq = {target_iq} A  ({target_T:+.3f} Nm) ---")

        # Capture starting velocity for this burst
        burst_start_t = now()
        vel_start_burst = ax.encoder.vel_estimate

        # --- RAMP ---
        ramp_start = time.time()
        while time.time() - ramp_start < RAMP_S:
            r = (time.time() - ramp_start) / RAMP_S
            cmd = target_T * r
            ax.controller.input_torque = cmd
            t, vel, pos, iq, temp, errs = sample("ramp", target_iq, cmd)
            maybe_print(t, "ramp", target_iq, cmd, iq, vel, pos, temp)
            if errs:
                abort_reason = f"err during burst {burst_idx+1} ramp: ax={ax.error} m={ax.motor.error}"; break
            if abs(vel) > VEL_ABORT_T_S:
                abort_reason = f"vel limit during burst {burst_idx+1} ramp"; break
            if temp >= TEMP_ABORT_C:
                abort_reason = f"FET {temp:.1f} C during burst {burst_idx+1} ramp"; break
            time.sleep(LOOP_DT_S)
        if abort_reason: break

        # --- HOLD ---
        hold_start_t = now()
        vel_at_hold_start = ax.encoder.vel_estimate
        hold_end = time.time() + HOLD_S
        last_vel = vel_at_hold_start
        last_t = hold_start_t
        while time.time() < hold_end:
            ax.controller.input_torque = target_T
            t, vel, pos, iq, temp, errs = sample("hold", target_iq, target_T)
            maybe_print(t, "hold", target_iq, target_T, iq, vel, pos, temp)
            last_vel = vel; last_t = t
            if errs:
                abort_reason = f"err during burst {burst_idx+1} hold: ax={ax.error} m={ax.motor.error}"; break
            if abs(vel) > VEL_ABORT_T_S:
                abort_reason = f"vel limit during burst {burst_idx+1} hold"; break
            if temp >= TEMP_ABORT_C:
                abort_reason = f"FET {temp:.1f} C during burst {burst_idx+1} hold"; break
            time.sleep(LOOP_DT_S)
        if abort_reason: break

        # alpha = (vel_end - vel_at_hold_start) / hold_duration
        hold_dt = last_t - hold_start_t
        d_vel = last_vel - vel_at_hold_start
        alpha_t_s2 = d_vel / hold_dt if hold_dt > 0 else 0.0
        alpha_rad_s2 = alpha_t_s2 * 2 * math.pi
        burst_results.append({
            "target_iq": target_iq,
            "target_T_nm": target_T,
            "vel_start": vel_at_hold_start,
            "vel_end": last_vel,
            "hold_dt": hold_dt,
            "alpha_t_s2": alpha_t_s2,
            "alpha_rad_s2": alpha_rad_s2,
        })
        print(f"  -> hold {hold_dt:.2f}s  vel {vel_at_hold_start:+.3f} -> {last_vel:+.3f} t/s  alpha = {alpha_t_s2:+.3f} t/s^2 = {alpha_rad_s2:+.3f} rad/s^2")

        # --- IDLE between bursts ---
        ax.controller.input_torque = 0
        # Drop to IDLE state to allow coastdown without controller fighting velocity
        ax.requested_state = 1
        time.sleep(0.3)
        idle_start = time.time()
        while time.time() - idle_start < IDLE_S:
            t, vel, pos, iq, temp, errs = sample("idle", target_iq, 0.0)
            if t - last_print > 1.0:
                print(f"   {t:6.2f} {'idle':>10} {target_iq:>4} {0.0:+7.3f} {iq:+7.2f} {vel:+7.3f} {pos:+9.2f} {temp:5.1f}")
                last_print = t
            time.sleep(0.5)

        # Re-arm for next burst
        if burst_idx < len(BURST_CURRENTS_A) - 1:
            for tgt in (ax, ax.motor, ax.encoder, ax.controller):
                try: tgt.error = 0
                except: pass
            ax.controller.input_torque = 0
            ax.requested_state = 8
            deadline = time.time() + 3.0
            while time.time() < deadline and ax.current_state != 8:
                time.sleep(0.05)
            if ax.current_state != 8:
                abort_reason = f"re-arm failed before burst {burst_idx+2}: state={ax.current_state} err={ax.error}"; break

finally:
    ax.controller.input_torque = 0
    ax.requested_state = 1
    csv_file.close()

# ---- analysis ----
print(f"\n=== BURST RESULTS ===")
print(f"  {'iq':>4} {'T_motor (Nm)':>14} {'alpha (rad/s^2)':>16}")
for r in burst_results:
    print(f"  {r['target_iq']:>4} {r['target_T_nm']:+14.3f} {r['alpha_rad_s2']:+16.3f}")

if len(burst_results) >= 2:
    import numpy as np
    T_arr = np.array([abs(r["target_T_nm"]) for r in burst_results])  # use absolute torque
    a_arr = np.array([abs(r["alpha_rad_s2"]) for r in burst_results])
    # T = J*alpha + T_friction  =>  T = J*alpha + b ; linear in alpha
    A = np.vstack([a_arr, np.ones_like(a_arr)]).T
    J_fit, T_friction_fit = np.linalg.lstsq(A, T_arr, rcond=None)[0]
    print(f"\n=== INERTIA / FRICTION FIT ===")
    print(f"  T_motor (Nm) = J * alpha (rad/s^2) + T_friction (Nm)")
    print(f"  J          = {J_fit:.4f} kg·m^2")
    print(f"  T_friction = {T_friction_fit:.3f} Nm")
    print(f"\n  geometric J of 9kg/30cm/10cm annulus = 0.1125 kg·m^2")
    print(f"  fitted J / geometric J = {J_fit/0.1125:.2f}x")

    # Plot
    try:
        import matplotlib.pyplot as plt
        fig, ax_plot = plt.subplots(1, 1, figsize=(8, 6))
        ax_plot.scatter(a_arr, T_arr, s=80, color="#26a", label="bursts (measured)")
        a_line = np.linspace(0, max(a_arr)*1.1, 50)
        T_line = J_fit * a_line + T_friction_fit
        ax_plot.plot(a_line, T_line, color="#c44", lw=2,
                     label=f"fit: T = {J_fit:.3f}·alpha + {T_friction_fit:.3f}")
        ax_plot.axhline(T_friction_fit, color="#888", linestyle="--", alpha=0.5,
                        label=f"T_friction = {T_friction_fit:.3f} Nm")
        for r in burst_results:
            ax_plot.annotate(f"{r['target_iq']}A", (abs(r['alpha_rad_s2']), abs(r['target_T_nm'])),
                             xytext=(8, 0), textcoords="offset points", fontsize=9)
        ax_plot.set_xlabel("Angular accel  alpha  (rad/s^2)")
        ax_plot.set_ylabel("Motor torque (Nm)")
        ax_plot.set_title(f"Inertia identification — burst test\nJ_fit = {J_fit:.4f} kg·m^2,  T_friction = {T_friction_fit:.3f} Nm")
        ax_plot.legend()
        ax_plot.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(PNG_PATH, dpi=130)
        print(f"  PNG: {PNG_PATH}")
    except Exception as e:
        print(f"  plot failed: {e}")

print(f"\n  CSV: {CSV_PATH}")
print(f"  abort_reason: {abort_reason if abort_reason else 'completed all bursts'}")
print("Returned to IDLE.")
