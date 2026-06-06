---
status: draft
tags:
  - qdd-gearbox
  - gears
  - agma
  - stress
---

# AGMA Stress Reference (QDD Pointer)

Short pointer doc — full theory lives in the vault under `Resources/Engineering/Machine Design/01_Theory/`. This file captures the QDD-specific knobs and the default K-factor stack the calculator uses.

## Theory Sources

- Bending: [[../../../../MegaVault/Resources/Engineering/Machine Design/01_Theory/AGMA_Bending_Stress]]
- Contact / pitting: [[../../../../MegaVault/Resources/Engineering/Machine Design/01_Theory/AGMA_Contact_Stress]]
- Mesh geometry worked example: [[../../../../MegaVault/Resources/Engineering/Machine Design/04_Derivations/Example_12_1_Spur_Gear_Mesh_Geometry]]

## Calculator

`calc/tooth_stress.py` now exposes two modes:

- **Lewis** — quick bending check via `analyze_stresses()` (Lewis form factor + textbook Hertz). Keep using for fast sweeps.
- **AGMA** — full K-factor stack via `analyze_agma_stresses()`. Use before committing to a geometry.

Both run side-by-side in `main()` and the iterative solver still uses Lewis (fast); the recommended config is then re-checked under AGMA.

## QDD Defaults (justify before changing)

| Factor | Value | Why |
|---|---:|---|
| $K_o$ | 1.25 | ODrive driving a robot leg — uniform source, moderate shock load (impact landings) |
| $Q_v$ | 5 | FDM-printed teeth; AGMA gives $K_v \approx 1.7$–$2.2$ at expected pitch-line velocities |
| $K_s$ | 1.0 | Tooth size is modest ($m \le 2$ mm) |
| $K_H$ | 1.4 | Commercial enclosed gearing, single-shoulder mount on planet pins, no crowning |
| $K_B$ — sun, planet | 1.0 | Solid gears |
| $K_B$ — ring | **measure** | Thin printed ring wall; compute backup ratio from CATIA wall thickness |
| $Z_R$ | 1.0 | FDM surface is rough but no residual tensile stress |
| Reliability | 0.99 | Hobby actuator, not safety-critical → $Y_Z = Z_Z = 1.0$ |
| Load-sharing | $m_N = 1.0$ external, sun shared by 3 planets with **1.10** unequal-share factor | AGMA recommends ≥1.1 unless sun is floating |

The "unequal-share factor" is applied by dividing $F_t$ across only 2.7 planets instead of 3 in the worst case.

## Open Questions

- Real $Y_J$ for $Z = 18$ (planet) mating $Z = 18$ (sun) — interpolate Norton Fig 12-23 vs use AGMA 908-B89 table when accessed.
- Plastic $\sigma_{FP}$ from coupon tests vs catalog — current calc uses material yield strength, not AGMA allowable. Acceptable for printed-PLA prototypes; replace with measured fatigue allowable before any production batch.
- Hardness-ratio factor $Z_W$ — N/A for matched PLA/Nylon, will matter if sun is ever upgraded to steel.

## When to re-check

- Any change to module, face width, or tooth counts.
- Material change (PLA → Nylon → POM → steel).
- Operating torque envelope changes (peak or duty-cycle).
- Print orientation / layer adhesion changes (affects $\sigma_{FP}$, not the K-stack).
