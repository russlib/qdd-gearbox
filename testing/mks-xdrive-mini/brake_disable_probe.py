"""Unified brake-disable probe for the Saris H2.

Runs one experiment: optionally do BLE setup (subscribe + a single write to the
proprietary control char), optionally disconnect, then drive the motor at fixed
torque in the freewheel-engaged (+1) direction and measure peak RPM.

Pass criteria (motor +1, 30 A, 15 s hold):
  >= 500 RPM        : PASS    (brake fully released)
  100..500 RPM      : PARTIAL (brake softened)
   50..100 RPM      : MARGINAL
  < 50 RPM          : FAIL    (matches braked baseline 49 RPM)
   0 RPM, no motion : STALL   (clamped beyond breakaway)

Modes:
  none           : no BLE at all                         (T0, T8 with AC unplugged)
  warmup         : write WARM_UP   (0x04)                (T1)
  headless       : write HEADLESS  (0x00)                (T3)
  power-range-0  : write POWER_RANGE (0x03) p1=0 p2=0    (T4)
  rolldown       : write ROLL_DOWN (0x05); use --multi-ramp to satisfy SpeedUp window (T2)

Flags:
  --disconnect-before-spin : after the write + settle, GATT-disconnect, THEN spin motor
                             (T5 with --mode warmup, T6 with --mode headless)
  --multi-ramp             : during the hold, cycle motor torque (3 ramps from 0 to target)
                             to give ROLL_DOWN the rising-speed signature it expects (T2)
  --skip-arm               : do not arm the motor; just run the BLE side (debug)

Outputs to thermal_data/brake_disable_<mode>[_disc][_mr]_<ts>.{csv,png}.
NEVER touches the motor without a clean disarm in finally.
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

# --- Motor / safety constants ---
KT_NMA          = 0.04
DIRECTION       = +1
TARGET_IQ_A     = 30.0
RAMP_S          = 5.0
HOLD_S          = 15.0
LOOP_DT_S       = 0.05
VEL_ABORT_T_S   = 20.0
TEMP_ABORT_C    = 78.0
CURRENT_LIM_A   = 60.0
SETTLE_AFTER_WRITE_S = 5.0


# ============================================================
# BLE side: handled in a background thread with its own loop
# ============================================================

class BleSession:
    def __init__(self, addr: str, mode: str):
        self.addr = addr
        self.mode = mode  # "none", "warmup", "headless", "power-range-0", "rolldown"
        self.thread: threading.Thread | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.ready = threading.Event()
        self.disconnect_request = threading.Event()
        self.stop_request = threading.Event()
        self.error: Exception | None = None
        self.cmd_responses: list[bytes] = []
        self.cps_count = 0
        self.last_cps_power = None

    async def _do_setup(self):
        # Connect
        try:
            client = BleakClient(self.addr, timeout=15.0)
            await client.connect()
        except Exception:
            device = await BleakScanner.find_device_by_address(self.addr, timeout=15.0)
            if not device:
                raise RuntimeError("trainer not found")
            client = BleakClient(device, timeout=15.0)
            await client.connect()

        # Always subscribe CPS for telemetry (brake-safe)
        def on_cps(_, data: bytearray):
            try:
                m = parse_cps(bytes(data))
                self.cps_count += 1
                self.last_cps_power = m.power_w
            except Exception:
                pass
        await client.start_notify(CPS_MEASUREMENT_UUID, on_cps)

        # If the test mode requires a write, subscribe to the proprietary char
        # (this is the brake-engaging step; we accept it for the modes that need it)
        if self.mode != "none":
            def on_resp(_, data: bytearray):
                self.cmd_responses.append(bytes(data))
            await client.start_notify(SARIS_RESISTANCE_UUID, on_resp)

            cmd = self._build_cmd()
            print(f"  [BLE] write to proprietary char: {cmd.hex()}  (mode={self.mode})")
            await client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd, response=False)
            print(f"  [BLE] write returned. Settling {SETTLE_AFTER_WRITE_S:.1f}s...")
            await asyncio.sleep(SETTLE_AFTER_WRITE_S)
            print(f"  [BLE] settle done. responses_so_far={len(self.cmd_responses)}")

        return client

    def _build_cmd(self) -> bytes:
        if self.mode == "warmup":
            return build_resistance_cmd(ResistanceMode.WARM_UP, 0, 0)
        if self.mode == "headless":
            return build_resistance_cmd(ResistanceMode.HEADLESS, 0, 0)
        if self.mode == "power-range-0":
            return build_resistance_cmd(ResistanceMode.POWER_RANGE, 0, 0)
        if self.mode == "rolldown":
            return build_resistance_cmd(ResistanceMode.ROLL_DOWN, 0, 0)
        if self.mode == "sim-flat":
            # weight 80kg * 100 = 8000, grade 0%
            return build_resistance_cmd(ResistanceMode.MANUAL_SLOPE, 8000, 0)
        if self.mode == "sim-downhill":
            # weight 80kg * 100 = 8000, grade -50% * 10 = -500 (signed int16)
            return build_resistance_cmd(ResistanceMode.MANUAL_SLOPE, 8000, -500)
        if self.mode == "erg-high":
            # ERG 1500W -- past max brake limit, may saturate to "give up"
            return build_resistance_cmd(ResistanceMode.MANUAL_POWER, 1500, 0)
        if self.mode == "mode-07":
            # Undocumented mode 0x07 -- fuzz probe found firmware accepts it (status 01)
            return bytes.fromhex("00100700000000000000")
        if self.mode == "mode-08":
            return bytes.fromhex("00100800000000000000")
        if self.mode == "mode-09":
            return bytes.fromhex("00100900000000000000")
        if self.mode == "mode-0a":
            return bytes.fromhex("00100a00000000000000")
        if self.mode == "slope-down-1000":
            # MANUAL_SLOPE with grade=-100% (param2 = -1000 in 0.1% units)
            return build_resistance_cmd(ResistanceMode.MANUAL_SLOPE, 8000, -1000)
        raise ValueError(f"no command for mode '{self.mode}'")

    async def _run(self):
        client = None
        try:
            client = await self._do_setup()
            self.ready.set()

            # Wait for either an explicit disconnect request or stop
            while not self.stop_request.is_set() and not self.disconnect_request.is_set():
                await asyncio.sleep(0.1)

            if self.disconnect_request.is_set():
                print(f"  [BLE] disconnect requested before spin. Tearing down GATT...")
                if self.mode != "none":
                    try: await client.stop_notify(SARIS_RESISTANCE_UUID)
                    except Exception: pass
                try: await client.stop_notify(CPS_MEASUREMENT_UUID)
                except Exception: pass
                await client.disconnect()
                client = None
                # Now wait for stop_request from the main thread
                while not self.stop_request.is_set():
                    await asyncio.sleep(0.1)

            # Final teardown
            if client is not None:
                if self.mode != "none":
                    try: await client.stop_notify(SARIS_RESISTANCE_UUID)
                    except Exception: pass
                try: await client.stop_notify(CPS_MEASUREMENT_UUID)
                except Exception: pass
                await client.disconnect()
        except Exception as e:
            self.error = e
            self.ready.set()

    def start(self):
        if self.mode == "none":
            self.ready.set()
            return
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

    def request_disconnect(self):
        self.disconnect_request.set()

    def stop(self):
        if self.mode == "none":
            return
        self.stop_request.set()
        if self.thread:
            self.thread.join(timeout=10.0)


# ============================================================
# Motor side: blocking on main thread
# ============================================================

def _arm(odrv, ax, mc, cc, current_lim_a: float = CURRENT_LIM_A):
    import odrive  # noqa
    ax.config.enable_watchdog = False
    for tgt in (ax, ax.motor, ax.encoder, ax.controller):
        try: tgt.error = 0
        except: pass
    mc.current_lim = current_lim_a
    try: mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), current_lim_a * 1.3)
    except: pass
    try: odrv.config.dc_max_positive_current = current_lim_a
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


def torque_test(ax, label: str, target_iq: float, direction: int,
                ramp_s: float, hold_s: float,
                multi_ramp: bool, temp_abort_c: float = TEMP_ABORT_C) -> dict:
    """Drive motor and log peak RPM.

    multi_ramp: instead of fixed-torque hold, cycle 0 -> target three times during the
    hold window. Useful for ROLL_DOWN, which expects a rising-speed signature.
    """
    target_T = direction * target_iq * KT_NMA
    logger = TestLogger(f"brake_disable_{label}")
    peak_rpm = 0.0
    peak_fet = 0.0
    abort = None

    t0 = time.time()
    def now(): return time.time() - t0
    last_print = -10.0

    print(f"\n  --- motor torque test: target={target_iq:+.1f}A ({target_T:+.2f} Nm) "
          f"dir={direction} ramp={ramp_s}s hold={hold_s}s multi_ramp={multi_ramp} ---")

    def sample(phase: str, cmd: float):
        nonlocal peak_rpm, peak_fet, last_print
        t = now()
        vel = ax.encoder.vel_estimate
        pos = ax.encoder.pos_estimate
        iq  = ax.motor.current_control.Iq_measured
        temp = ax.fet_thermistor.temperature
        errs = ax.error | ax.motor.error | ax.encoder.error
        rpm = abs(vel) * 60.0
        if rpm > peak_rpm: peak_rpm = rpm
        if temp > peak_fet: peak_fet = temp
        logger.row(t_s=t, phase=phase, cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
                   pos_turns=pos, fet_c=temp, axis_err=ax.error,
                   motor_err=ax.motor.error, encoder_err=ax.encoder.error)
        if t - last_print > 1.0:
            print(f"    t={t:5.1f}s  vel={vel*60:+7.1f} RPM  Iq={iq:+5.1f}  FET={temp:.1f}C  phase={phase}")
            last_print = t
        return t, vel, errs, temp

    try:
        # Initial ramp
        rs = time.time()
        while time.time() - rs < ramp_s:
            cmd = target_T * (time.time() - rs) / ramp_s
            ax.controller.input_torque = cmd
            t, vel, errs, temp = sample("ramp", cmd)
            if errs: abort = f"err: ax={ax.error} m={ax.motor.error} e={ax.encoder.error}"; break
            if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
            if temp >= temp_abort_c: abort = f"FET {temp:.1f}C"; break
            time.sleep(LOOP_DT_S)

        if abort is None:
            if multi_ramp:
                # Three additional 0 -> target ramps inside the hold window
                cycles = 3
                cycle_dur = hold_s / cycles
                for c in range(cycles):
                    cs = time.time()
                    while time.time() - cs < cycle_dur:
                        frac = (time.time() - cs) / cycle_dur
                        cmd = target_T * frac
                        ax.controller.input_torque = cmd
                        t, vel, errs, temp = sample(f"hold_cycle{c+1}", cmd)
                        if errs: abort = f"err: {ax.error}"; break
                        if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
                        if temp >= temp_abort_c: abort = f"FET {temp:.1f}C"; break
                        time.sleep(LOOP_DT_S)
                    if abort: break
            else:
                he = time.time() + hold_s
                while time.time() < he:
                    ax.controller.input_torque = target_T
                    t, vel, errs, temp = sample("hold", target_T)
                    if errs: abort = f"err: {ax.error}"; break
                    if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
                    if temp >= temp_abort_c: abort = f"FET {temp:.1f}C"; break
                    time.sleep(LOOP_DT_S)
    finally:
        try: ax.controller.input_torque = 0
        except Exception: pass
        time.sleep(0.3)

    logger.close()
    logger.plot(title=f"brake_disable {label}",
                subtitle=f"peak_rpm={peak_rpm:.1f}  peak_fet={peak_fet:.1f}C  abort={abort or 'ok'}")

    return {"peak_rpm": peak_rpm, "peak_fet": peak_fet, "abort": abort,
            "csv": str(logger.csv_path), "png": str(logger.png_path)}


# ============================================================
# Verdict
# ============================================================

def verdict(peak_rpm: float) -> str:
    if peak_rpm >= 500: return "PASS (brake fully released)"
    if peak_rpm >= 100: return "PARTIAL (brake softened)"
    if peak_rpm >= 50:  return "MARGINAL"
    if peak_rpm > 0:    return "FAIL (matches braked baseline)"
    return "STALL"


# ============================================================
# Main
# ============================================================

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", required=True,
                   choices=["none", "warmup", "headless", "power-range-0", "rolldown",
                            "sim-flat", "sim-downhill", "erg-high",
                            "mode-07", "mode-08", "mode-09", "mode-0a",
                            "slope-down-1000"])
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--target-iq", type=float, default=TARGET_IQ_A)
    p.add_argument("--direction", type=int, default=DIRECTION, choices=[-1, 1])
    p.add_argument("--ramp", type=float, default=RAMP_S)
    p.add_argument("--hold", type=float, default=HOLD_S)
    p.add_argument("--temp-abort", type=float, default=TEMP_ABORT_C,
                   help=f"FET temp abort threshold (default {TEMP_ABORT_C})")
    p.add_argument("--current-lim", type=float, default=CURRENT_LIM_A,
                   help=f"motor current limit (default {CURRENT_LIM_A})")
    p.add_argument("--multi-ramp", action="store_true",
                   help="cycle torque 0->target during hold (T2 ROLL_DOWN)")
    p.add_argument("--disconnect-before-spin", action="store_true",
                   help="GATT disconnect after BLE write + settle, before motor spin (T5/T6)")
    p.add_argument("--skip-motor", action="store_true",
                   help="do BLE setup only; do not arm motor (debug)")
    args = p.parse_args()

    label = args.mode
    if args.disconnect_before_spin:
        label += "_disc"
    if args.multi_ramp:
        label += "_mr"
    print(f"========== {label.upper()} ==========")

    # --- Set up BLE ---
    ble = BleSession(args.addr, args.mode)
    if args.mode != "none":
        print(f"  [BLE] connecting + setup (mode={args.mode})...")
        ble.start()
        print(f"  [BLE] setup done. cmd_responses_so_far={len(ble.cmd_responses)}")
        if args.disconnect_before_spin:
            ble.request_disconnect()
            time.sleep(2.0)
            print(f"  [BLE] GATT disconnected.")
    else:
        print(f"  [BLE] skipping (mode=none)")

    if args.skip_motor:
        print("  --skip-motor set; not arming motor.")
        time.sleep(2.0)
        ble.stop()
        return

    # --- Motor side ---
    import odrive
    odrv = odrive.find_any(timeout=20)
    if odrv is None:
        ble.stop()
        raise SystemExit("USB enum failed")
    ax = odrv.axis0
    mc = ax.motor.config
    cc = ax.controller.config
    print(f"  [ODRV] bus={odrv.vbus_voltage:.2f}V  state={ax.current_state}  FET={ax.fet_thermistor.temperature:.1f}C")
    _arm(odrv, ax, mc, cc, current_lim_a=args.current_lim)

    result = None
    try:
        result = torque_test(ax, label, args.target_iq, args.direction,
                             args.ramp, args.hold, args.multi_ramp,
                             temp_abort_c=args.temp_abort)
    finally:
        _disarm(ax)
        ble.stop()

    if result is None:
        return

    print(f"\n========== RESULT: {label} ==========")
    print(f"  peak_rpm  : {result['peak_rpm']:.1f}")
    print(f"  peak_fet  : {result['peak_fet']:.1f} C")
    print(f"  abort     : {result['abort'] or 'completed'}")
    print(f"  verdict   : {verdict(result['peak_rpm'])}")
    print(f"  ble responses received : {len(ble.cmd_responses)}")
    if ble.cmd_responses:
        for i, r in enumerate(ble.cmd_responses[:5]):
            print(f"    response[{i}]: {r.hex()}")
    print(f"  cps notifications      : {ble.cps_count}  last_power={ble.last_cps_power}")
    print(f"  CSV : {result['csv']}")
    print(f"  PNG : {result['png']}")


if __name__ == "__main__":
    main()
