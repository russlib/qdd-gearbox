"""Hold 45 A motor torque (1.8 Nm) loaded direction with FET temp logging.
Continues logging FET temp at idle for THERMAL_COOLDOWN_S after the test
to capture the cooling curve. Saves CSV + plot.
"""
import csv
import math
import sys
import time
from pathlib import Path
import odrive

# ---- knobs ----
DIRECTION         = -1
RAMP_TO_NM        = 45.0 * 0.04   # 1.8 Nm
RAMP_S            = 4.0
HOLD_S            = 60.0
LOOP_DT_S         = 0.05
THERMAL_COOLDOWN_S = 180.0
COOLDOWN_LOG_DT_S = 0.5
VEL_ABORT_T_S     = 5.0
TEMP_WARN_C       = 65.0
TEMP_ABORT_C      = 80.0
CURRENT_LIM_A     = 50.0
DC_MAX_POS_CURRENT = 50.0
MAX_HOLD_S        = RAMP_S + HOLD_S + 2.0

OUT_DIR  = Path(__file__).parent / "thermal_data"
OUT_DIR.mkdir(exist_ok=True)
ts = time.strftime("%Y%m%d-%H%M%S")
CSV_PATH = OUT_DIR / f"thermal_45A_{ts}.csv"
PNG_PATH = OUT_DIR / f"thermal_45A_{ts}.png"

odrv = odrive.find_any(timeout=20)
if odrv is None:
    raise SystemExit("USB enum failed")
ax = odrv.axis0
mc = ax.motor.config
cc = ax.controller.config

print(f"== Pre-test ==")
print(f"  bus={odrv.vbus_voltage:.2f} V  state={ax.current_state}  encoder.offset={ax.encoder.config.offset}")

def get_temp():
    try:
        return ax.fet_thermistor.temperature
    except Exception:
        return None

initial_temp = get_temp()
print(f"  Initial FET temp: {initial_temp:.1f} C")

ax.config.enable_watchdog = False
for tgt in (ax, ax.motor, ax.encoder, ax.controller):
    try: tgt.error = 0
    except: pass

mc.current_lim = CURRENT_LIM_A
try:
    mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.3)
except Exception:
    pass
try:
    odrv.config.dc_max_positive_current = DC_MAX_POS_CURRENT
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
print(f"\n== Armed in TORQUE mode, direction=LOADED ==")
print(f"   ramp 0 -> {DIRECTION*RAMP_TO_NM:+.2f} Nm over {RAMP_S}s  then hold {HOLD_S}s  then idle+log {THERMAL_COOLDOWN_S}s")
print(f"   abort: |vel| > {VEL_ABORT_T_S} t/s ({VEL_ABORT_T_S*60:.0f} RPM)  or  FET >= {TEMP_ABORT_C} C  or  any error")

print(f"\n   {'t(s)':>6} {'phase':>9} {'cmd':>7} {'Iq':>7} {'vel':>7} {'pos':>8} {'FET':>6}")

# Open CSV
csv_file = open(CSV_PATH, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["t_s", "phase", "cmd_nm", "iq_a", "vel_t_s", "pos_turns", "fet_c", "axis_err", "motor_err", "encoder_err"])

abort_reason = None
peak_iq = 0.0
peak_temp = 0.0
final_pos_start = ax.encoder.pos_estimate
last_print = -1.0

t0 = time.time()
def now(): return time.time() - t0

def sample(phase, cmd_nm):
    global peak_iq, peak_temp, last_print
    t = now()
    vel = ax.encoder.vel_estimate
    pos = ax.encoder.pos_estimate
    iq  = ax.motor.current_control.Iq_measured
    temp = get_temp()
    aerr = ax.error
    merr = ax.motor.error
    eerr = ax.encoder.error
    if abs(iq) > abs(peak_iq): peak_iq = iq
    if temp is not None and temp > peak_temp: peak_temp = temp
    writer.writerow([f"{t:.3f}", phase, f"{cmd_nm:.4f}", f"{iq:.3f}", f"{vel:.4f}", f"{pos:.4f}",
                     f"{temp:.2f}" if temp is not None else "", aerr, merr, eerr])
    if t - last_print > 0.20 or phase == "ramp" and t < 0.05:
        tdisp = f"{temp:6.1f}" if temp is not None else "  ---"
        print(f"   {t:6.2f} {phase:>9} {cmd_nm:+7.3f} {iq:+7.2f} {vel:+7.3f} {pos:+8.2f} {tdisp}")
        last_print = t
    return vel, temp, aerr | merr | eerr

try:
    # --- RAMP ---
    while now() < RAMP_S:
        target_T = DIRECTION * RAMP_TO_NM * (now() / RAMP_S)
        ax.controller.input_torque = target_T
        vel, temp, errs = sample("ramp", target_T)
        if errs:
            abort_reason = f"ODrive error during ramp: ax={ax.error} m={ax.motor.error} e={ax.encoder.error}"; break
        if abs(vel) > VEL_ABORT_T_S:
            abort_reason = f"vel {vel*60:.0f} RPM during ramp"; break
        if temp is not None and temp >= TEMP_ABORT_C:
            abort_reason = f"FET {temp:.1f} C during ramp"; break
        time.sleep(LOOP_DT_S)

    # --- HOLD ---
    if abort_reason is None:
        hold_end = time.time() + HOLD_S
        while time.time() < hold_end:
            ax.controller.input_torque = DIRECTION * RAMP_TO_NM
            vel, temp, errs = sample("hold", DIRECTION * RAMP_TO_NM)
            if errs:
                abort_reason = f"ODrive error during hold: ax={ax.error} m={ax.motor.error} e={ax.encoder.error}"; break
            if abs(vel) > VEL_ABORT_T_S:
                abort_reason = f"vel {vel*60:.0f} RPM during hold"; break
            if temp is not None and temp >= TEMP_ABORT_C:
                abort_reason = f"FET {temp:.1f} C during hold"; break
            if now() > MAX_HOLD_S:
                abort_reason = "wall-time hard cap"; break
            time.sleep(LOOP_DT_S)

finally:
    # Always idle the motor before the cooling log
    ax.controller.input_torque = 0
    time.sleep(0.3)
    ax.requested_state = 1
    time.sleep(0.3)

heat_end_t = now()
print(f"\n   --- entering COOLDOWN logging for {THERMAL_COOLDOWN_S}s ---")
print(f"   {'t(s)':>6} {'phase':>9} {'FET':>6}")
last_print = -1.0

cooldown_end = time.time() + THERMAL_COOLDOWN_S
while time.time() < cooldown_end:
    t = now()
    temp = get_temp()
    writer.writerow([f"{t:.3f}", "cool", "", "", "", "", f"{temp:.2f}" if temp is not None else "", 0, 0, 0])
    if t - last_print > 1.0:
        tdisp = f"{temp:6.1f}" if temp is not None else "  ---"
        print(f"   {t:6.2f}    cool                                  {tdisp}")
        last_print = t
    time.sleep(COOLDOWN_LOG_DT_S)

csv_file.close()

final_temp = get_temp()
final_pos = ax.encoder.pos_estimate
delta_turns = final_pos - final_pos_start

print(f"\n=== RESULT ===")
print(f"  abort_reason  : {abort_reason if abort_reason else 'completed full ramp+hold'}")
print(f"  peak Iq       : {peak_iq:+.2f} A")
print(f"  peak FET temp : {peak_temp:.1f} C")
print(f"  final FET temp: {final_temp:.1f} C")
print(f"  position delta during heat phase: {delta_turns:+.4f} turns ({delta_turns*360:+.1f} deg)")
print(f"  CSV: {CSV_PATH}")

# ---- Plot ----
try:
    import numpy as np
    import matplotlib.pyplot as plt

    rows = []
    with open(CSV_PATH) as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)

    t_arr = np.array([float(r["t_s"]) for r in rows])
    fet_arr = np.array([float(r["fet_c"]) if r["fet_c"] else math.nan for r in rows])
    iq_arr  = np.array([float(r["iq_a"]) if r["iq_a"] else 0.0 for r in rows])
    pos_arr = np.array([float(r["pos_turns"]) if r["pos_turns"] else math.nan for r in rows])
    phase_arr = np.array([r["phase"] for r in rows])

    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax1 = axs[0]
    ax1.plot(t_arr, fet_arr, color="#c44", lw=1.5, label="FET temp")
    # shade phases
    for phase, color in [("ramp", "#ffd"), ("hold", "#ffb"), ("cool", "#def")]:
        idx = np.where(phase_arr == phase)[0]
        if len(idx) > 0:
            ax1.axvspan(t_arr[idx[0]], t_arr[idx[-1]], color=color, alpha=0.5, label=phase)
    ax1.axhline(TEMP_ABORT_C, color="#a00", linestyle="--", alpha=0.5, label=f"abort {TEMP_ABORT_C} C")
    ax1.set_ylabel("FET temp (C)")
    ax1.set_title(f"MKS Mini thermal response — 45 A loaded direction\npeak FET = {peak_temp:.1f} C  (initial {initial_temp:.1f} C, final {final_temp:.1f} C)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2 = axs[1]
    ax2.plot(t_arr, iq_arr, color="#26a", lw=1.0, label="Iq")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Iq (A)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(PNG_PATH, dpi=130)
    print(f"  PNG: {PNG_PATH}")
except Exception as e:
    print(f"  plotting failed: {e}")

print("\nReturned to IDLE.")
