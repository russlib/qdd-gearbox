# Saris H2 Brake Disable — Test Matrix

Goal: find a reliable software path to disable the H2's eddy brake so we can
spin its flywheel from the D6374 motor without resistance.

## Pass criterion (single test)

Motor in **+1 direction** (freewheel-engaged), 30 A torque (1.2 Nm), 5 s ramp +
15 s hold. Read peak motor RPM:

| Peak RPM | Verdict |
|---|---|
| ≥ 500 RPM | **PASS** — brake fully released |
| 100–500 RPM | **PARTIAL** — reduced load, brake softened but not off |
| 50–100 RPM | **MARGINAL** — slight improvement over fully-braked |
| < 50 RPM | **FAIL** — brake still engaged (matches braked baseline 49 RPM) |
| 0 RPM, 0 motion | **STALL** — brake clamped beyond breakaway, possibly worse than baseline |

Note: motor pre-test FET temp must be < 35 °C; abort at 78 °C. Test self-aborts
at 20 t/s = 1200 motor RPM for safety.

## Pre-flight (do once at start)

| Step | Action | Verify |
|---|---|---|
| 0a | Trainer powered, USB to ODrive plugged in | `bus ≈ 34 V`, `state` reachable |
| 0b | **Power-cycle the H2** — unplug AC, wait 15 s, replug. Wait for advertise LED. | trainer asleep until flywheel spun |
| 0c | Spin flywheel by hand to wake H2 | LED indicates connection-ready |

## Test sequence — run in order until PASS

Run each, log peak RPM, abort the matrix at first PASS. **Power-cycle the H2 between
every test** to clear any latched brake state from the previous attempt
(power-cycle is the only confirmed-clean reset).

| # | Path | Sequence | Confidence | Effort |
|---|---|---|---|---|
| **T0** | **Fresh-boot baseline** | Just-power-cycled, no BLE, motor spin | n/a | 1 min |
| **T1** | `WARM_UP` (0x04) | subscribe → write `WARM_UP` → 5 s settle → motor spin | HIGH | 2 min |
| **T2** | `ROLL_DOWN` (0x05) calibration window | subscribe → write `ROLL_DOWN` → motor RPM ramp 0→200 to trigger `SpeedUp`→`InProcess` → motor spin during 30 s window | HIGH | 5 min |
| **T3** | `HEADLESS` (0x00) on fresh boot | subscribe → write `HEADLESS` → motor spin | MED | 2 min |
| **T4** | `POWER_RANGE` (0x03) min=0 max=0 | subscribe → write `POWER_RANGE 0,0` → motor spin | LOW | 2 min |
| **T5** | `WARM_UP` + **disconnect** | subscribe → write `WARM_UP` → 5 s → disconnect GATT → motor spin (no BLE) | MED | 2 min |
| **T6** | `HEADLESS` + **disconnect** | same as T5 with HEADLESS | LOW-MED | 2 min |
| **T7** | **Decode 0x1005 heartbeat** | subscribe (no writes) → log heartbeat for 10 s, parse offset 4 (mode) and offset 9 (status) → read brake state directly | DIAGNOSTIC | 5 min |
| **T8** | **AC unplugged** | unplug trainer AC, motor spin only | RELIABILITY: HIGH<br>UTILITY: low (no telemetry) | 2 min |
| **T9** | ANT+ FE-C grade = −200% | Requires Garmin ANT+ stick + `openant`. BLE NOT subscribed. Broadcast Page 51 with grade=0x0000. | MED-HIGH | 1 hr (incl. install) |
| **T10** | nRF52840 BLE sniffer + Zwift | Pair Zwift to H2 with sniffer running. Capture proprietary-char writes during Zwift "ride out" / unpaired states. Replay any commands not in our enum. | HIGH (if a hidden cmd exists) | 1–2 hr (incl. flash) |

## Result table (fill in as you go)

| # | Date/Time | Peak RPM | Verdict | Notes |
|---|---|---|---|---|
| T0 |  |  |  |  |
| T1 |  |  |  |  |
| T2 |  |  |  |  |
| T3 |  |  |  |  |
| T4 |  |  |  |  |
| T5 |  |  |  |  |
| T6 |  |  |  |  |
| T7 |  |  | (parse heartbeat — see notes file) |  |
| T8 |  |  |  |  |
| T9 |  |  |  |  |
| T10 |  |  |  |  |

## Recommended order

1. **T0** baseline — confirms power-cycle gave clean state
2. **T7** heartbeat decoder — read trainer's mode/status directly (diagnostic)
3. **T1** WARM_UP — highest confidence
4. **T2** ROLL_DOWN with RPM ramp — also high confidence
5. **T3** HEADLESS on fresh boot
6. **T5** WARM_UP + disconnect
7. **T6** HEADLESS + disconnect
8. **T4** POWER_RANGE 0,0
9. **T8** AC unplugged — fallback if all above fail

T9 (ANT+) and T10 (BLE sniffer) need new hardware — separate session.

## How to run

**Walk through the whole matrix interactively** (recommended):

```powershell
cd C:\Obsidian\_dev\qdd-gearbox\testing\mks-xdrive-mini
.\run_brake_matrix.ps1
```

The runner prompts you to power-cycle the H2 between every test, runs the
probe, prints peak RPM + verdict, then asks whether to continue.

Skip ahead or run a subset:
```powershell
.\run_brake_matrix.ps1 -StartFrom T2
.\run_brake_matrix.ps1 -OnlyTests T1,T2,T3
```

**Or run a single test manually:**

```powershell
. .\mks-python-env.ps1
& $script:MksPython brake_disable_probe.py --mode <mode> [--disconnect-before-spin] [--multi-ramp]
& $script:MksPython heartbeat_decoder.py --duration 30
```

| # | Single-test command |
|---|---|
| T0 | `brake_disable_probe.py --mode none` |
| T1 | `brake_disable_probe.py --mode warmup` |
| T2 | `brake_disable_probe.py --mode rolldown --multi-ramp` |
| T3 | `brake_disable_probe.py --mode headless` |
| T4 | `brake_disable_probe.py --mode power-range-0` |
| T5 | `brake_disable_probe.py --mode warmup --disconnect-before-spin` |
| T6 | `brake_disable_probe.py --mode headless --disconnect-before-spin` |
| T7 | `heartbeat_decoder.py --duration 30` |
| T8 | `brake_disable_probe.py --mode none` (unplug trainer AC first) |
| T9 | (separate, ANT+ stack — not implemented) |
| T10 | (separate, sniffer + Wireshark — not implemented) |

## Stop conditions

- **First PASS** ⇒ stop matrix, document the working sequence in `reference_saris_h2_protocol.md`.
- **All tiers 1–3 fail** ⇒ commit to T8 (AC unplugged) for now; order smart plug for automation; plan T10 (sniffer) for next session.
- **Any STALL with motor drawing > 35 A** ⇒ stop motor immediately, investigate before next test (could indicate brake locked + motor pushing; thermal risk).

## Data layout (auto-generated by probe scripts)

```
thermal_data/
  brake_disable_T0_<ts>.csv   + .png
  brake_disable_T1_<ts>.csv   + .png
  ...
  heartbeat_decode_<ts>.csv   (raw notifications + parsed fields)
```

Each PNG: 4-panel (Iq, vel, FET, position) over time, with phase shading.
