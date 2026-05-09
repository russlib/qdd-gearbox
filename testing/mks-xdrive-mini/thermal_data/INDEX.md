# Thermal Data — Test Index

All test artifacts in this folder. Each `<name>.csv` has a paired `<name>_meta.json` (test conditions + result) and most have a `<name>.png` plot.

## How to read this

- **Pass criteria for brake-disable tests** (motor +1 dir, 30 A IQ, 1.2 Nm):
  - `< 50 RPM` peak = FAIL (matches braked baseline)
  - `50-100` = MARGINAL · `100-500` = PARTIAL · `≥ 500` = brake released
- **Continuous-current tests**: target FET temp at 80 °C; report `continuous_iq_a` from the last 60 s average.

## 2026-05-06 — Brake trigger probes (early diagnostic)

Goal: find which BLE GATT operation engages the H2 brake. Discovered freewheel direction asymmetry.

| Time | Test | Direction | IQ | Peak RPM | Verdict |
|---|---|---|---|---|---|
| 11:06:48 | brake_probe_trialA_none | -1 | 30 A | 824 | free spin (freewheel disengaged) |
| 11:07:37 | brake_probe_trialB_connect | -1 | 15 A | 1244 | free spin |
| 11:08:11 | brake_probe_trialC_cps | -1 | 15 A | 1236 | free spin (CPS subscribe brake-safe) |
| 11:09:14 | brake_probe_trialA_none | -1 | 30 A | 1244 | free spin baseline |
| 11:10:07 | brake_probe_trialA_none | +1 | 30 A | 16.5 | brake locked (freewheel engaged) |

**Key learning:** -1 dir = freewheel disengaged (motor spins free, flywheel not loaded). +1 dir = engaged. All meaningful brake tests must use +1 direction.

## 2026-05-07 — Brake disable matrix (all FAIL, +1 dir)

Goal: find a BLE command that releases the H2 brake. Outcome: every documented and undocumented BLE path fails.

| Time | Test | Mode/path | Peak RPM | Outcome |
|---|---|---|---|---|
| 17:57:21 | brake_disable_none | T0 baseline (no BLE) | 33 | FAIL — braked default state |
| 18:02:09 | brake_disable_warmup | T1 WARM_UP (0x04) | 24.7 | FAIL — heartbeat shows MANUAL_POWER 0W |
| 18:03:08 | brake_disable_headless | T3 HEADLESS (0x00) | 33 | FAIL |
| 18:03:50 | brake_disable_power-range-0 | T4 POWER_RANGE (0x03) | 24.7 | FAIL |
| 18:04:32 | brake_disable_rolldown_mr | T2 ROLL_DOWN (0x05) + multi-ramp | 24.7 | FAIL — status 0x03 init only |
| 18:06:07 | brake_disable_warmup_disc | T5 WARM_UP + GATT disconnect | 24.7 | FAIL |
| 18:07:47 | brake_disable_sim-flat | SIM grade 0% | 24.7 | FAIL (BLE may have dropped) |
| 18:30:55 | rapid_brake_cycle | 9-mode cycle at 45 A | 49.4 | FAIL (also exposed BLE write-queue bug) |
| 18:55:35 | brake_disable_none | post-rolldown disconnect, 45 A | 41.2 | FAIL — trainer reverts to default |
| 18:58:18 | rolldown_then_spin v1 | ROLL_DOWN keepalive, 45 A | 41.2 | FAIL — RollDownFailed |
| 19:00:17 | rolldown_then_spin v2 | HEADLESS pre-seq + heartbeat-confirmed HEADLESS | 41.2 | **DEFINITIVE FAIL** — HEADLESS = Fluid2 curve, not free spin |
| 19:08:57 | brake_disable_none | trainer AC UNPLUGGED | 41.2 | FAIL — brake persists without AC (bias-magnet hypothesis) |
| 19:48:28 | brake_disable_mode-07 | undocumented mode 0x07 | 24.7 | FAIL — fuzz hit, acked but inert |
| 19:49:23 | brake_disable_slope-down-1000 | SIM grade -100% | 33 | FAIL — accepted, internally clamped |
| 20:09:35 | offset_comp_then_spin | CPS standard 0x0C Start Offset Comp | 41.2 | FAIL — `20 0c 01 ff ff` Success but no brake change |
| 20:16:04 | wahoo_erg_0w_then_spin | Wahoo 0x42 ERG 0W on `a026e005` | 33 | FAIL — Wahoo opcodes accepted but inert |

**Conclusion:** brake is firmware-floor-clamped on this H2 firmware. Cannot be released by any BLE protocol path tested.

## 2026-05-08 — Continuous current characterization (POSITIVE results)

Goal: find the max continuous IQ where FET temperature stabilizes at the safe limit. Run with adaptive PI controller targeting 80 °C.

| Time | ESC fan | Motor fan | Continuous IQ | Continuous torque | Steady FET | Convergence |
|---|---|---|---|---|---|---|
| 14:45:02 | **ON** | on | **45.8 A** | **1.84 Nm** | 79.5 °C | drifting (ax=2 abort, supply hiccup) |
| 18:38:38 | **OFF** | on | **25.2 A** | **1.01 Nm** | 79.1 °C | **fully converged** (std=0.00 over last 60 s) |

**Key takeaway:** ESC fan delivers ≈ 3.4× heat removal vs natural convection (`(45/25)² ≈ 3.4`).

## Comparison plots

- `all_brake_tests_rpm.png` — overlay of all brake-disable RPM curves (all identical shape, proves no mode released the brake)
- `all_brake_tests_grid.png` — small-multiples grid, easier reading

## Per-CSV metadata

Each test has a JSON sidecar with this schema:

```json
{
  "test_name": "...",
  "timestamp": "YYYYMMDD-HHMMSS",
  "purpose": "one-line description",
  "hardware": { "motor": "...", "controller": "...", ... },
  "conditions": {
    "esc_fan": true/false,
    "motor_fan": true/false,
    "trainer_ac": "on|off|unplugged",
    "trainer_ble_state": "...",
    "direction": +1 | -1,
    "bus_v": 34.0,
    "ambient_c": 25,
    "current_lim_a": 60
  },
  "parameters": { "target_iq_a": ..., "hold_s": ..., ... },
  "result": { "peak_rpm": ..., "verdict": "...", ... }
}
```

To query: `Get-Content thermal_data\<name>_meta.json | ConvertFrom-Json`.

## Tools used

- `brake_disable_probe.py` — unified BLE-then-spin probe (modes: none/warmup/headless/...)
- `adaptive_iq_at_temp.py` — closed-loop PI controller for continuous-current search
- `byte_fuzz_probe.py` — Wahoo/proprietary char fuzzer
- `cps_control_probe.py` — standard CPS Control Point opcode probe
- `probe_wahoo_opcodes.py` — Wahoo KICKR opcodes on `a026e005`
- `compare_brake_tests.py` — generates the comparison plots above
- `_logger.py` — shared CSV/PNG/meta writer
- `saris_h2/ble_threaded.py` — robust threaded BLE session class (replaces broken queue-poll)
