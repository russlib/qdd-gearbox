"""Rapid brake-mode cycling at speed.

Hypothesis: the H2 may behave differently at higher RPM than near stall, and
"brake release" commands might only take effect when the trainer sees the
flywheel moving. This test:

  1. BLE connect + subscribe to proprietary char.
  2. Motor TORQUE mode, +1 direction, ramp to TARGET_IQ_A.
  3. Hold torque for SETTLE_S so wheel reaches whatever RPM equilibrium the
     brake allows.
  4. Rapidly cycle through every documented release-candidate mode, ~3 s each.
     Log motor vel/Iq and BLE responses on a single timeline.
  5. After cycling, log a final "brake state" hold for OBSERVE_S and disarm.

If any mode releases the brake at speed, motor RPM jumps mid-cycle and we see
which command preceded the jump.

Aborts: FET >= 70 C, vel > 20 t/s (1200 RPM), any ODrive error.

Sequence in each cycle is configurable; default explores all 6 documented modes
plus a sim-downhill (-50%) and an erg-high (1500W) wildcard.
"""
from __future__ import annotations
import argparse
import asyncio
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from bleak import BleakClient, BleakScanner
from saris_h2.protocol import (
    DEFAULT_ADDR,
    SARIS_RESISTANCE_UUID,
    CPS_MEASUREMENT_UUID,
    ResistanceMode,
    build_resistance_cmd,
    parse_cps,
)
from _logger import TestLogger

# --- Motor / safety ---
KT_NMA              = 0.04
DIRECTION           = +1
TARGET_IQ_A         = 45.0
RAMP_S              = 4.0
SETTLE_S            = 6.0      # spin up + reach equilibrium
PER_CMD_S           = 3.0      # how long each command is held during cycling
OBSERVE_S           = 6.0      # final observation after cycling
LOOP_DT_S           = 0.05
VEL_ABORT_T_S       = 20.0
TEMP_ABORT_C        = 70.0
CURRENT_LIM_A       = 60.0


# Cycle order: try "release" candidates first, then SIM with negative grade,
# then control variants. After each, observe RPM. If any boost RPM, we win.
DEFAULT_CYCLE = [
    ("HEADLESS",       ResistanceMode.HEADLESS,       0,    0),
    ("WARM_UP",        ResistanceMode.WARM_UP,        0,    0),
    ("SIM_FLAT",       ResistanceMode.MANUAL_SLOPE,   8000, 0),
    ("SIM_DOWN_50",    ResistanceMode.MANUAL_SLOPE,   8000, -500),
    ("SIM_DOWN_99",    ResistanceMode.MANUAL_SLOPE,   8000, -990),
    ("ROLL_DOWN",      ResistanceMode.ROLL_DOWN,      0,    0),
    ("POWER_RANGE_0",  ResistanceMode.POWER_RANGE,    0,    0),
    ("ERG_1500W",      ResistanceMode.MANUAL_POWER,   1500, 0),
    ("HEADLESS_again", ResistanceMode.HEADLESS,       0,    0),
]


# ============================================================
# BLE side
# ============================================================

class BleSession:
    def __init__(self, addr: str):
        self.addr = addr
        self.thread: threading.Thread | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.client: BleakClient | None = None
        self.ready = threading.Event()
        self.stop_request = threading.Event()
        self.error: Exception | None = None
        self.events: list[tuple[float, str, str]] = []   # (ble_t, kind, info)
        self.t0_ble: float | None = None

        self.cps_count = 0
        self.last_power = None
        # Lock for cross-thread cmd queue
        self._cmd_queue: list[tuple[str, bytes]] = []
        self._queue_lock = threading.Lock()

    def submit_cmd(self, label: str, cmd_bytes: bytes):
        with self._queue_lock:
            self._cmd_queue.append((label, cmd_bytes))

    def _drain_queue(self):
        with self._queue_lock:
            q = self._cmd_queue[:]
            self._cmd_queue.clear()
        return q

    async def _run(self):
        try:
            try:
                self.client = BleakClient(self.addr, timeout=15.0)
                await self.client.connect()
            except Exception:
                device = await BleakScanner.find_device_by_address(self.addr, timeout=15.0)
                if not device:
                    raise RuntimeError("trainer not found")
                self.client = BleakClient(device, timeout=15.0)
                await self.client.connect()

            self.t0_ble = time.time()

            def on_cps(_, data: bytearray):
                try:
                    m = parse_cps(bytes(data))
                    self.cps_count += 1
                    self.last_power = m.power_w
                    t = time.time() - self.t0_ble
                    self.events.append((t, "cps",
                        f"pwr={m.power_w}W acc_t={m.acc_torque_nm:.2f}Nm hex={bytes(data).hex()}"))
                except Exception as e:
                    self.events.append((time.time() - self.t0_ble, "cps_err", str(e)))

            def on_resp(_, data: bytearray):
                t = time.time() - self.t0_ble
                self.events.append((t, "resp", f"hex={bytes(data).hex()}"))

            await self.client.start_notify(CPS_MEASUREMENT_UUID, on_cps)
            await self.client.start_notify(SARIS_RESISTANCE_UUID, on_resp)
            self.events.append((0.0, "subscribed", ""))
            self.ready.set()

            # Main pump loop: drain submitted commands, poll
            while not self.stop_request.is_set():
                queued = self._drain_queue()
                for label, cmd in queued:
                    t = time.time() - self.t0_ble
                    self.events.append((t, "write", f"{label} hex={cmd.hex()}"))
                    try:
                        await self.client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd, response=False)
                    except Exception as e:
                        self.events.append((time.time() - self.t0_ble, "write_err", str(e)))
                await asyncio.sleep(0.05)

            # Teardown
            try: await self.client.stop_notify(CPS_MEASUREMENT_UUID)
            except Exception: pass
            try: await self.client.stop_notify(SARIS_RESISTANCE_UUID)
            except Exception: pass
            await self.client.disconnect()
        except Exception as e:
            self.error = e
            self.ready.set()

    def start(self):
        def thread_target():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._run())
            self.loop.close()
        self.thread = threading.Thread(target=thread_target, daemon=True)
        self.thread.start()
        self.ready.wait(timeout=30.0)
        if self.error:
            raise self.error

    def stop(self):
        self.stop_request.set()
        if self.thread:
            self.thread.join(timeout=10.0)


# ============================================================
# Motor side
# ============================================================

def _arm(odrv, ax, mc, cc):
    ax.config.enable_watchdog = False
    for tgt in (ax, ax.motor, ax.encoder, ax.controller):
        try: tgt.error = 0
        except: pass
    mc.current_lim = CURRENT_LIM_A
    try: mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.3)
    except: pass
    try: odrv.config.dc_max_positive_current = CURRENT_LIM_A
    except: pass
    cc.control_mode = 1
    cc.input_mode = 1
    ax.controller.input_torque = 0
    ax.requested_state = 8
    deadline = time.time() + 3.0
    while time.time() < deadline and ax.current_state != 8:
        time.sleep(0.05)
    if ax.current_state != 8:
        raise SystemExit(f"arm failed: state={ax.current_state} err={ax.error}")


def _disarm(ax):
    try:
        ax.controller.input_torque = 0
        time.sleep(0.3)
        ax.requested_state = 1
    except Exception:
        pass


# ============================================================
# Main
# ============================================================

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--target-iq", type=float, default=TARGET_IQ_A)
    p.add_argument("--per-cmd", type=float, default=PER_CMD_S)
    p.add_argument("--settle", type=float, default=SETTLE_S)
    args = p.parse_args()

    print(f"========== RAPID CYCLE @ {args.target_iq}A ==========")
    print(f"  cycle: {[c[0] for c in DEFAULT_CYCLE]}")
    print(f"  per_cmd={args.per_cmd}s  settle={args.settle}s")

    # BLE up
    ble = BleSession(args.addr)
    print("  [BLE] connecting...")
    ble.start()
    if ble.error:
        raise SystemExit(f"BLE error: {ble.error}")
    print("  [BLE] connected + subscribed.")

    # ODrive up
    import odrive
    odrv = odrive.find_any(timeout=20)
    if odrv is None:
        ble.stop()
        raise SystemExit("USB enum failed")
    ax = odrv.axis0
    mc = ax.motor.config
    cc = ax.controller.config
    print(f"  [ODRV] bus={odrv.vbus_voltage:.2f}V  state={ax.current_state}  FET={ax.fet_thermistor.temperature:.1f}C")
    if ax.fet_thermistor.temperature > 50.0:
        print(f"  [WARN] FET={ax.fet_thermistor.temperature:.1f}C is high; consider cooldown before high-current test")
    _arm(odrv, ax, mc, cc)

    target_T = DIRECTION * args.target_iq * KT_NMA
    logger = TestLogger("rapid_brake_cycle", extra_fields=["mode_label"])

    abort = None
    peak_rpm = 0.0
    peak_fet = 0.0
    cycle_results = []   # (label, avg_rpm_during_cmd, peak_rpm_during_cmd)
    t0_motor = time.time()
    def t_now(): return time.time() - t0_motor
    last_print = -10.0
    current_mode_label = "init"

    def sample(phase: str, cmd_nm: float):
        nonlocal peak_rpm, peak_fet, last_print
        t = t_now()
        vel = ax.encoder.vel_estimate
        pos = ax.encoder.pos_estimate
        iq  = ax.motor.current_control.Iq_measured
        temp = ax.fet_thermistor.temperature
        errs = ax.error | ax.motor.error | ax.encoder.error
        rpm = abs(vel) * 60.0
        if rpm > peak_rpm: peak_rpm = rpm
        if temp > peak_fet: peak_fet = temp
        logger.row(t_s=t, phase=phase, cmd_nm=cmd_nm, iq_a=iq, vel_t_s=vel,
                   pos_turns=pos, fet_c=temp, axis_err=ax.error,
                   motor_err=ax.motor.error, encoder_err=ax.encoder.error,
                   mode_label=current_mode_label)
        if t - last_print > 0.5:
            print(f"    t={t:5.1f}s  vel={vel*60:+7.1f} RPM  Iq={iq:+5.1f}  FET={temp:.1f}C  mode={current_mode_label}")
            last_print = t
        return t, vel, errs, temp

    try:
        # Phase 1: ramp
        rs = time.time()
        current_mode_label = "ramp"
        while time.time() - rs < RAMP_S:
            cmd = target_T * (time.time() - rs) / RAMP_S
            ax.controller.input_torque = cmd
            t, vel, errs, temp = sample("ramp", cmd)
            if errs: abort = f"err: {ax.error} {ax.motor.error}"; break
            if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
            if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
            time.sleep(LOOP_DT_S)

        # Phase 2: settle at target torque
        if not abort:
            current_mode_label = "settle"
            ss = time.time()
            while time.time() - ss < args.settle:
                ax.controller.input_torque = target_T
                t, vel, errs, temp = sample("settle", target_T)
                if errs: abort = f"err: {ax.error}"; break
                if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
                if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
                time.sleep(LOOP_DT_S)

        # Phase 3: rapid cycling
        if not abort:
            for label, mode, p1, p2 in DEFAULT_CYCLE:
                cmd_bytes = build_resistance_cmd(mode, p1, p2)
                current_mode_label = label
                print(f"\n  >>> CYCLE: {label}  ({cmd_bytes.hex()})")
                ble.submit_cmd(label, cmd_bytes)
                rpm_samples = []

                cs = time.time()
                while time.time() - cs < args.per_cmd:
                    ax.controller.input_torque = target_T
                    t, vel, errs, temp = sample(f"cycle_{label}", target_T)
                    rpm_samples.append(abs(vel) * 60.0)
                    if errs: abort = f"err in {label}: {ax.error}"; break
                    if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
                    if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
                    time.sleep(LOOP_DT_S)

                if rpm_samples:
                    avg_r = sum(rpm_samples) / len(rpm_samples)
                    pk_r  = max(rpm_samples)
                    cycle_results.append((label, avg_r, pk_r))
                    print(f"      end:  avg_rpm={avg_r:.1f}  peak_rpm={pk_r:.1f}")
                if abort: break

        # Phase 4: final observe
        if not abort:
            current_mode_label = "observe"
            os_t = time.time()
            while time.time() - os_t < OBSERVE_S:
                ax.controller.input_torque = target_T
                t, vel, errs, temp = sample("observe", target_T)
                if errs: abort = f"err: {ax.error}"; break
                if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
                if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
                time.sleep(LOOP_DT_S)
    finally:
        try: ax.controller.input_torque = 0
        except Exception: pass
        time.sleep(0.3)
        _disarm(ax)
        ble.stop()

    logger.close()
    logger.plot(title=f"rapid brake cycle @ {args.target_iq}A",
                subtitle=f"peak_rpm={peak_rpm:.1f}  peak_fet={peak_fet:.1f}C  abort={abort or 'ok'}")

    print(f"\n========== RESULT ==========")
    print(f"  peak_rpm overall : {peak_rpm:.1f}")
    print(f"  peak_fet         : {peak_fet:.1f} C")
    print(f"  abort            : {abort or 'completed'}")
    print(f"\n  Per-cycle RPM (sorted by peak desc):")
    for label, avg_r, pk_r in sorted(cycle_results, key=lambda x: -x[2]):
        flag = "  <-- WINNER?" if pk_r > 80 else ""
        print(f"    {label:>16}  avg={avg_r:6.1f}  peak={pk_r:6.1f}{flag}")
    print(f"\n  CSV : {logger.csv_path}")
    print(f"  PNG : {logger.png_path}")
    print(f"\n  BLE timeline events: {len(ble.events)}")
    # Save BLE events to a sidecar file
    ble_log_path = Path(str(logger.csv_path).replace(".csv", "_ble.log"))
    with open(ble_log_path, "w") as fh:
        for et, kind, info in ble.events:
            fh.write(f"{et:.3f}\t{kind}\t{info}\n")
    print(f"  BLE log: {ble_log_path}")
    # Print just write+resp events (high signal)
    for et, kind, info in ble.events:
        if kind in ("write", "resp"):
            print(f"    [BLE t={et:5.2f}s] {kind:5s}  {info}")


if __name__ == "__main__":
    main()
