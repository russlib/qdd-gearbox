# H2 Motor Bench — Key Findings

**Last updated: 2026-05-08**

Two-day campaign characterizing the **D6374-150KV BLDC motor** on the **MKS XDrive Mini** (ODrive 0.5.1 fw) using a **Saris Hammer H2** trainer as the load. AMT-102 ABI encoder, 34 V DC bus, fan cooling on the controller PCB, motor fan on the BLDC body.

## Continuous current rating (D6374 + MKS XDrive Mini)

Adaptive PI controller targeting 80 °C FET, +1 direction (brake-loaded stall — worst-case thermal):

| Cooling | Continuous IQ | Continuous torque | Steady-state FET |
|---|---|---|---|
| **ESC fan ON + motor fan ON** | **45 A** | **1.8 Nm** | 79.5 °C |
| **Motor fan ON, ESC fan OFF** | **25 A** | **1.0 Nm** | 79.1 °C |

- Conservative design point (FET ≤ 75 °C, longevity): ~40 A with both fans, ~22 A with ESC fan off.
- Test was at low RPM (brake-loaded). Spinning the motor freely would let the rotor self-cool; expect 10-20 % more continuous capacity at ≥ 500 RPM.
- Headline number for spec sheets: **45 A continuous IQ at 80 °C FET, fan-cooled.**

## H2 brake disable

**Verdict: cannot be disabled via BLE on this H2 firmware. Cannot be disabled by removing AC power either.**

Tested 30+ BLE commands across three characteristics:

| Path | Outcome |
|---|---|
| Saris proprietary `ca31a533` — all 6 documented modes (HEADLESS, WARM_UP, ROLL_DOWN, etc.) | All accepted, all produce identical brake drag |
| Proprietary — undocumented mode 0x07 (fuzz find) | Accepted, inert |
| Proprietary — negative ERG watts | Hard-rejected by firmware (`fd` status) |
| Proprietary — extreme negative slope | Accepted but firmware clamps internally |
| Standard CPS Control Point `0x2A66` — all 17 BT-spec opcodes | Only 0x01 (Cumulative) and 0x0C (Start Offset Compensation) supported; both inert on brake |
| Wahoo-derived `a026e005` — opcodes 0x42-0x47 (Set ERG / SIM / Grade / etc.) | All accepted with `0x01` Success status, all inert on brake |
| AC unplugged | Brake unchanged — eddy brake doesn't need power (likely permanent-magnet bias) |
| Power-cycle | Trainer state persists across cycles in firmware NVRAM |

**Root-cause hypothesis:** the H2's eddy brake has a **permanent-magnet bias** that produces baseline drag at all times. Coil current can ADD to this baseline but cannot subtract. No software path exists to remove the bias magnet's contribution.

**Workarounds for the bench:**
1. **Use H2 as-is and characterize the drag** — `T_drag(ω) ≈ 0.3 Nm·s/rad` linear viscous, repeatable
2. **Replace with passive trainer** — CycleOps Fluid2 / Kinetic Road Machine ($80–180 used) — no eddy brake at all
3. **Hardware mod** — open H2 housing, remove or shim the bias magnet 5+ mm from flywheel disc

## Thermal model (FET PCB thermistor)

First-order lumped:

$$T_{FET}(t) = T_{ambient} + \Delta T_{ss} (1 - e^{-t/\tau})$$

| Cooling | τ (estimated) | Notes |
|---|---|---|
| ESC fan + motor fan | **47 s** | from rapid_brake_cycle data |
| Motor fan only (ESC fan off) | **~90 s** | from full 600 s adaptive run |
| Heat scaling | ∝ I² | textbook copper losses; verified |

## BLE protocol notes

The H2 exposes three control characteristics:

| Char UUID | Service | Properties | Function |
|---|---|---|---|
| `ca31a533-a858-4dc7-a650-fdeb6dad4c14` | `c0f4013a-...` (Saris/CycleOps) | indicate, write | Saris proprietary control |
| `a026e005-0a7d-4ab3-97fa-f1500f9feb8b` | `0x1818` (CPS) | indicate, write | **Wahoo-derived** control point (UUID base is Wahoo KICKR) |
| `0x2A66` | `0x1818` (CPS) | indicate, write | Standard BT CPS Control Point |

**Saris proprietary frame format** (10 bytes):
```
[0x00] [0x10] [mode] [p1_lo] [p1_hi] [p2_lo] [p2_hi] [0x00] [0x00] [0x00]
```

**Modes accepted:**
- `0x00` HEADLESS — Fluid2 curve simulation (NOT free spin, contrary to qdomyos-zwift docs)
- `0x01` MANUAL_POWER — ERG mode at target watts (param1 = watts)
- `0x02` MANUAL_SLOPE — SIM mode (p1 = weight × 100, p2 = grade × 10 signed)
- `0x03` POWER_RANGE — min/max watts
- `0x04` WARM_UP — transient
- `0x05` ROLL_DOWN — calibration; status `0x03` init, `0x04` in-process, `0x06` failed
- `0x07` — accepted, undocumented, inert (we found this via fuzzing; not in any open driver)

**Status code map (proprietary char responses):**
- `0x01` = ack/success
- `0xfd` = invalid params (reserved bits set, out-of-range, wrong mode within enum)
- `0xfe` = unknown command (wrong command-id byte [1] or prefix byte [0])

**Heartbeat (cmdId 0x1005, after writes):** byte [4] is the trainer's current ControlMode.

**Wahoo-derived `a026e005` accepted opcodes** (status `0x01` Success, but inert on brake):
- `0x42` Set ERG (uint16 watts)
- `0x43` Set SIM mode (weight + drag params)
- `0x44` Set Grade
- `0x45` Set Wind Resistance
- `0x46` Set Rolling Resistance
- `0x47` Set Wheel Diameter

Rejected (status `0x02`): `0x40`, `0x41`, `0x4A`, `0x4B`-`0x4E`. Wahoo unlock keys (0x4F, 0x20 with 0xee 0xfc) all rejected.

## Encoder behavior (AMT-102 ABI, no index)

- Motor calibration **does** persist in flash (motor R, L, Kt, current_lim, encoder direction/CPR).
- Encoder offset **does not** persist on DC bus power cycle. Need state 7 (ENCODER_OFFSET_CALIBRATION) after every bus reset.
- Offset values vary between cal runs (e.g. 6782, 18591, 18627, 18805 across this campaign). All valid; not a bug.
- USB cycle without DC bus reset preserves the offset.

## Things missing / future work

- **Motor body temperature.** We only have FET PCB thermistor. Motor windings could be 30+ °C hotter at high IQ. A clip-on thermocouple or in-winding NTC would give a real motor thermal limit.
- **Bus current logging.** Should add `odrv.ibus` to logged columns to compute mechanical efficiency vs supply input.
- **Brake characterization.** With brake undisableable, build a coast-down + multi-current-step test to fit `T_drag(ω, ω̇)` for the H2. Then subtract from motor torque measurements.
- **Index pin on AMT-102.** Wiring the Z pin and enabling `use_index = True` would persist encoder offset across power cycles.
- **Saris Utility / Rouvy / Zwift BLE sniff.** Highest-leverage unfinished experiment. Could reveal a privileged command we missed.
- **ANT+ FE-C path.** $25 Garmin ANT+ stick + `openant`. The ANT+ codepath in the H2 firmware may have different brake-control semantics than BLE.

## File index

See `thermal_data/INDEX.md` for the per-test catalog.
