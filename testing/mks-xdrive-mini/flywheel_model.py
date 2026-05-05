"""Flywheel inertia model + torque/current sizing for D6374 + MKS Mini.

Computes:
1. Annular-disk inertia from geometry
2. Torque vs angular acceleration (with and without measured friction floor)
3. Speed-vs-time under constant Iq scenarios (kinematic, no back-EMF)
4. Back-calc: required Iq to reach a target speed within a target time

Saves a 3-panel plot to flywheel_model.png next to this script.
"""
from __future__ import annotations
import math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# ---- inputs ----
M_KG       = 9.0          # flywheel mass
R_OUTER_M  = 0.15         # outer radius (30 cm OD)
R_INNER_M  = 0.05         # inner hole radius (10 cm ID)

KT_NMA     = 0.04         # D6374 conservative Kt
T_FRICTION = 0.05         # estimated motor cogging in loaded dir (NOT the 0.115 freewheel measurement, which decoupled the flywheel)

# Hardware envelopes
I_CONT_A   = 25.0         # MKS Mini continuous (Russell-authorized)
I_PEAK_A   = 60.0         # MKS Mini peak short bursts
I_HARD_A   = 70.0         # Aaron's measured "effective Iq cap"

# Targets to back-calculate
TARGET_RPMS  = [300, 600, 1000, 2000]
TARGET_TIMES_S = [0.5, 1.0, 5.0, 10.0]

# ---- inertia ----
J = 0.5 * M_KG * (R_OUTER_M**2 + R_INNER_M**2)
print(f"Annular-disk inertia: J = 0.5 * {M_KG} * ({R_OUTER_M}^2 + {R_INNER_M}^2)")
print(f"                        = 0.5 * {M_KG} * {R_OUTER_M**2 + R_INNER_M**2:.4f}")
print(f"                        = {J:.4f} kg·m^2")

# ---- panel A: torque-acceleration ----
T_array = np.linspace(0, 3.0, 200)  # Nm
alpha_ideal     = T_array / J
alpha_realistic = np.maximum(0, (T_array - T_FRICTION)) / J  # net of friction floor

# ---- panel B: speed vs time at constant current ----
def speed_curve(I_a, t_array, friction=T_FRICTION):
    T_motor = I_a * KT_NMA
    T_net   = T_motor - friction
    if T_net <= 0:
        return np.zeros_like(t_array)
    alpha = T_net / J
    return alpha * t_array  # rad/s

t_array = np.linspace(0, 10.0, 200)
scenarios = [
    (5,  "5 A"),
    (10, "10 A"),
    (25, "25 A continuous"),
    (60, "60 A peak"),
]

# ---- panel C: required current table ----
table = []
for rpm in TARGET_RPMS:
    omega_rad_s = rpm * 2.0 * math.pi / 60.0
    row = [rpm]
    for t_s in TARGET_TIMES_S:
        alpha_req = omega_rad_s / t_s
        T_req     = J * alpha_req + T_FRICTION
        I_req     = T_req / KT_NMA
        row.append(I_req)
    table.append(row)

# ---- plot ----
fig, axs = plt.subplots(1, 3, figsize=(18, 5.5))

# Panel A: alpha(T)
ax = axs[0]
ax.plot(T_array, alpha_ideal,     label="ideal (no friction)", lw=2)
ax.plot(T_array, alpha_realistic, label=f"with T_friction = {T_FRICTION} Nm", lw=2)
# mark current envelopes (in Nm)
for I_label, I_a, color, ls in [(f"25 A cont = {25*KT_NMA:.2f} Nm", 25, "#888", "--"),
                                  (f"60 A peak = {60*KT_NMA:.2f} Nm", 60, "#c44", ":")]:
    T_mark = I_a * KT_NMA
    ax.axvline(T_mark, color=color, linestyle=ls, alpha=0.7, label=I_label)
ax.set_xlabel("Motor torque T (Nm)")
ax.set_ylabel("Angular accel  alpha  (rad/s^2)")
ax.set_title(f"Torque -> Acceleration\nJ = {J:.4f} kg·m^2  (m={M_KG}kg, r_o={R_OUTER_M*100:.0f}cm, r_i={R_INNER_M*100:.0f}cm)")
ax.legend(loc="upper left", fontsize=9)
ax.grid(True, alpha=0.3)

# Panel B: omega(t)
ax = axs[1]
for I_a, label in scenarios:
    omega = speed_curve(I_a, t_array)
    ax.plot(t_array, omega * 60.0 / (2 * math.pi), label=label, lw=2)
for rpm_target in [300, 1000]:
    ax.axhline(rpm_target, color="#888", linestyle="--", alpha=0.5)
    ax.text(t_array[-1]*0.95, rpm_target+30, f"{rpm_target} RPM", fontsize=9, color="#666", ha="right")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Output speed (RPM)")
ax.set_title("Speed-up curves at constant Iq\n(ignores back-EMF, valid until v_back ~ supply)")
ax.legend(loc="lower right", fontsize=9)
ax.grid(True, alpha=0.3)

# Panel C: required-Iq table as bar chart
ax = axs[2]
x = np.arange(len(TARGET_RPMS))
width = 0.18
for i, t_s in enumerate(TARGET_TIMES_S):
    vals = [row[i+1] for row in table]
    bars = ax.bar(x + (i - 1.5) * width, vals, width, label=f"in {t_s}s")
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width()/2, b.get_height() + 1, f"{v:.0f}A",
                ha="center", fontsize=8)
# Annotate hardware envelope lines
for I_a, label, color in [(25, "25 A cont", "#666"), (60, "60 A peak", "#c44"), (70, "70 A cap", "#a00")]:
    ax.axhline(I_a, color=color, linestyle="--", alpha=0.6)
    ax.text(len(TARGET_RPMS)-0.5, I_a+1, label, fontsize=8, color=color, ha="right")
ax.set_xticks(x)
ax.set_xticklabels([f"{rpm} RPM" for rpm in TARGET_RPMS])
ax.set_ylabel("Required Iq (A)")
ax.set_title("Iq to reach target speed in target time\n(includes T_friction floor)")
ax.legend(loc="upper left", fontsize=9)
ax.grid(True, alpha=0.3, axis="y")

plt.tight_layout()
out_path = Path(__file__).parent / "flywheel_model.png"
plt.savefig(out_path, dpi=130)
print(f"\nSaved plot: {out_path}")

# ---- text table ----
print(f"\nRequired Iq (A) to reach target speed in target time:")
header = "  target RPM | " + " | ".join(f"{t}s" for t in TARGET_TIMES_S)
print(header)
print("  " + "-" * (len(header) - 2))
for row in table:
    rpm = row[0]
    cells = " | ".join(f"{v:6.1f}" for v in row[1:])
    print(f"  {rpm:>9} | {cells}")

print(f"\nKey numbers:")
print(f"  J          = {J:.4f} kg·m^2")
print(f"  Kt         = {KT_NMA:.4f} Nm/A")
print(f"  T_friction = {T_FRICTION:.3f} Nm  (= {T_FRICTION/KT_NMA:.2f} A breakaway)")
print(f"  At 25 A continuous: alpha_max = ({25*KT_NMA - T_FRICTION:.3f})/{J:.4f} = {(25*KT_NMA - T_FRICTION)/J:.2f} rad/s^2 = {(25*KT_NMA - T_FRICTION)/J * 60/(2*math.pi):.1f} RPM/s")
print(f"  At 60 A peak:       alpha_max = ({60*KT_NMA - T_FRICTION:.3f})/{J:.4f} = {(60*KT_NMA - T_FRICTION)/J:.2f} rad/s^2 = {(60*KT_NMA - T_FRICTION)/J * 60/(2*math.pi):.1f} RPM/s")

print(f"\nNOTE: This model is for the LOADED direction (flywheel engaged).")
print(f"      The 305 RPM spin-up we measured was in the FREEWHEEL direction,")
print(f"      so the flywheel was decoupled and only ~{0.0003} kg·m^2 (rotor) was spinning.")
print(f"      Those numbers are not comparable to this model.")
