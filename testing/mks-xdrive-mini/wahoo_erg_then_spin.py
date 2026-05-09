"""Wahoo ERG 0W via 0x42 on a026e005, then spin motor.

The H2 exposes a Wahoo-derived control char `a026e005-0a7d-4ab3-97fa-f1500f9feb8b`
inside its CPS service. We've shown the firmware accepts opcodes 0x42-0x47
(Set ERG, Set Sim, Set Grade, etc.) with status 0x01 = Success.

In Wahoo KICKR protocol:
  - 0x42 [watts_LE_u16]  = Set ERG mode at target watts
  - ERG 0 W is "no resistance" (opposite of Saris's proprietary char where 0W = max brake)

This script:
  1. Connect, subscribe to Wahoo char
  2. Write 0x42 0x00 0x00 (ERG 0W via Wahoo path)
  3. Verify success indication
  4. Drive motor at 30A in +1 dir for 25s
  5. Watch for RPM > 50 RPM ceiling

If motor blows past 50 RPM: brake released. We win.
"""
from __future__ import annotations
import argparse
import asyncio
import sys
import threading
import time
from concurrent.futures import Future, TimeoutError as FutTimeout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from bleak import BleakClient, BleakScanner
from saris_h2.protocol import DEFAULT_ADDR, CPS_MEASUREMENT_UUID, parse_cps
from _logger import TestLogger

WAHOO_CHAR_UUID = "a026e005-0a7d-4ab3-97fa-f1500f9feb8b"

KT_NMA          = 0.04
DIRECTION       = +1
TARGET_IQ_A     = 30.0
RAMP_S          = 4.0
HOLD_S          = 25.0
LOOP_DT_S       = 0.05
VEL_ABORT_T_S   = 20.0
TEMP_ABORT_C    = 70.0
CURRENT_LIM_A   = 60.0


# Predefined sequences to test
WAHOO_SEQUENCES = {
    "erg-0w":         bytes([0x42, 0x00, 0x00]),
    "erg-500w":       bytes([0x42, 0xf4, 0x01]),
    "erg-9999w":      bytes([0x42, 0x0f, 0x27]),
    "sim-flat":       bytes([0x43, 0x40, 0x1f, 0x00, 0x00, 0x00, 0x00]),  # 80kg, 0% rolling, 0 wind
    "grade-0":        bytes([0x44, 0x00, 0x00]),     # 0 (signed, with possibly mid-encoding)
    "grade-neg":      bytes([0x44, 0x00, 0x80]),     # signed -32768
    "grade-pos":      bytes([0x44, 0xff, 0x7f]),     # signed +32767
    "wind-0":         bytes([0x45, 0x00, 0x00]),
    "rolling-0":      bytes([0x46, 0x00, 0x00]),
}


class WahooSession:
    def __init__(self, addr: str):
        self.addr = addr
        self._client: BleakClient | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._setup_error: BaseException | None = None
        self.cps_count = 0
        self.last_power = None
        self.responses: list[tuple[float, bytes]] = []
        self.t0: float | None = None

    async def _setup(self):
        try:
            self._client = BleakClient(self.addr, timeout=15.0)
            await self._client.connect()
        except Exception:
            device = await BleakScanner.find_device_by_address(self.addr, timeout=15.0)
            if not device:
                raise RuntimeError("trainer not found")
            self._client = BleakClient(device, timeout=15.0)
            await self._client.connect()
        self.t0 = time.time()

        def on_meas(_, data: bytearray):
            try:
                m = parse_cps(bytes(data))
                self.cps_count += 1
                self.last_power = m.power_w
            except Exception: pass

        def on_wahoo(_, data: bytearray):
            t_rel = time.time() - self.t0
            self.responses.append((t_rel, bytes(data)))

        await self._client.start_notify(CPS_MEASUREMENT_UUID, on_meas)
        await self._client.start_notify(WAHOO_CHAR_UUID, on_wahoo)

    async def _teardown(self):
        if self._client is None: return
        for u in (CPS_MEASUREMENT_UUID, WAHOO_CHAR_UUID):
            try: await self._client.stop_notify(u)
            except Exception: pass
        try: await self._client.disconnect()
        except Exception: pass

    def _thread_main(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._setup())
        except BaseException as e:
            self._setup_error = e
            self._ready.set()
            try: self._loop.close()
            except: pass
            return
        self._ready.set()
        try: self._loop.run_forever()
        finally:
            try: self._loop.run_until_complete(self._teardown())
            except: pass
            self._loop.close()

    def start(self):
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=25.0)
        if self._setup_error: raise self._setup_error

    def write(self, label: str, payload: bytes, timeout_s=3.0) -> bool:
        async def _do():
            await self._client.write_gatt_char(WAHOO_CHAR_UUID, payload, response=True)
        fut = asyncio.run_coroutine_threadsafe(_do(), self._loop)
        try:
            fut.result(timeout=timeout_s); return True
        except (FutTimeout, Exception): return False

    def stop(self):
        if self._loop is None: return
        try: self._loop.call_soon_threadsafe(self._loop.stop)
        except: pass
        if self._thread: self._thread.join(timeout=10.0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--target-iq", type=float, default=TARGET_IQ_A)
    p.add_argument("--hold", type=float, default=HOLD_S)
    p.add_argument("--seq", default="erg-0w", choices=list(WAHOO_SEQUENCES.keys()),
                   help="Which Wahoo command to send before motor test")
    args = p.parse_args()

    payload = WAHOO_SEQUENCES[args.seq]
    print(f"========== WAHOO_{args.seq.upper()}_THEN_SPIN ==========")
    print(f"  payload: {payload.hex()}  target_iq={args.target_iq}A  hold={args.hold}s")

    sess = WahooSession(args.addr)
    print("  [BLE] starting session...")
    sess.start()
    print("  [BLE] connected.")

    print(f"  [BLE] writing {args.seq} ({payload.hex()})...")
    ok = sess.write(args.seq, payload, timeout_s=3.0)
    print(f"  [BLE]   ok={ok}")
    print(f"  [BLE] settling 3s for indication...")
    time.sleep(3.0)
    print(f"  [BLE] responses so far: {len(sess.responses)}")
    for t, raw in sess.responses:
        print(f"    [t={t:5.2f}s] {raw.hex()}")

    # --- Motor side ---
    import odrive
    odrv = odrive.find_any(timeout=15)
    if odrv is None:
        sess.stop(); raise SystemExit("USB enum failed")
    ax = odrv.axis0; mc = ax.motor.config; cc = ax.controller.config
    print(f"  [ODRV] bus={odrv.vbus_voltage:.2f}V state={ax.current_state} FET={ax.fet_thermistor.temperature:.1f}C")

    ax.config.enable_watchdog = False
    for tgt in (ax, ax.motor, ax.encoder, ax.controller):
        try: tgt.error = 0
        except: pass
    mc.current_lim = CURRENT_LIM_A
    try: mc.requested_current_range = max(getattr(mc, 'requested_current_range', 0), CURRENT_LIM_A * 1.3)
    except: pass
    try: odrv.config.dc_max_positive_current = CURRENT_LIM_A
    except: pass
    cc.control_mode = 1; cc.input_mode = 1
    ax.controller.input_torque = 0
    ax.requested_state = 8
    deadline = time.time() + 3.0
    while time.time() < deadline and ax.current_state != 8: time.sleep(0.05)
    if ax.current_state != 8:
        sess.stop(); raise SystemExit(f"arm failed: {ax.current_state} {ax.error}")

    target_T = DIRECTION * args.target_iq * KT_NMA
    logger = TestLogger(f"wahoo_{args.seq.replace('-', '_')}_then_spin")
    abort = None; peak_rpm = 0.0; peak_fet = 0.0
    last_print = -10.0
    t0 = time.time()
    def now(): return time.time() - t0

    try:
        rs = time.time()
        while time.time() - rs < RAMP_S:
            cmd = target_T * (time.time() - rs) / RAMP_S
            ax.controller.input_torque = cmd
            t = now(); vel = ax.encoder.vel_estimate
            iq = ax.motor.current_control.Iq_measured
            temp = ax.fet_thermistor.temperature
            errs = ax.error | ax.motor.error | ax.encoder.error
            rpm = abs(vel) * 60.0
            if rpm > peak_rpm: peak_rpm = rpm
            if temp > peak_fet: peak_fet = temp
            logger.row(t_s=t, phase="ramp", cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
                       pos_turns=ax.encoder.pos_estimate, fet_c=temp,
                       axis_err=ax.error, motor_err=ax.motor.error, encoder_err=ax.encoder.error)
            if t - last_print > 0.4:
                print(f"    t={t:5.1f}s  vel={vel*60:+7.1f} RPM  Iq={iq:+5.1f}  FET={temp:.1f}C  ramp")
                last_print = t
            if errs: abort = f"err: {ax.error}"; break
            if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
            if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
            time.sleep(LOOP_DT_S)

        if not abort:
            he = time.time() + args.hold
            while time.time() < he:
                ax.controller.input_torque = target_T
                t = now(); vel = ax.encoder.vel_estimate
                iq = ax.motor.current_control.Iq_measured
                temp = ax.fet_thermistor.temperature
                errs = ax.error | ax.motor.error | ax.encoder.error
                rpm = abs(vel) * 60.0
                if rpm > peak_rpm: peak_rpm = rpm
                if temp > peak_fet: peak_fet = temp
                logger.row(t_s=t, phase="hold", cmd_nm=target_T, iq_a=iq, vel_t_s=vel,
                           pos_turns=ax.encoder.pos_estimate, fet_c=temp,
                           axis_err=ax.error, motor_err=ax.motor.error, encoder_err=ax.encoder.error)
                if t - last_print > 0.4:
                    print(f"    t={t:5.1f}s  vel={vel*60:+7.1f} RPM  Iq={iq:+5.1f}  FET={temp:.1f}C")
                    last_print = t
                if errs: abort = f"err: {ax.error}"; break
                if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
                if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
                time.sleep(LOOP_DT_S)
    finally:
        try:
            ax.controller.input_torque = 0
            time.sleep(0.3); ax.requested_state = 1
        except: pass
        sess.stop()

    logger.close()
    logger.plot(title=f"wahoo {args.seq} @ {args.target_iq}A",
                subtitle=f"peak_rpm={peak_rpm:.1f}  peak_fet={peak_fet:.1f}C")

    print(f"\n========== RESULT ==========")
    verdict = ("PASS" if peak_rpm >= 500 else
               "PARTIAL" if peak_rpm >= 100 else
               "MARGINAL" if peak_rpm >= 50 else
               "FAIL")
    print(f"  peak_rpm  : {peak_rpm:.1f}  -> {verdict}")
    print(f"  peak_fet  : {peak_fet:.1f} C")
    print(f"  abort     : {abort or 'completed'}")
    print(f"  cps cnt   : {sess.cps_count}  last_power: {sess.last_power}")
    print(f"  wahoo responses:")
    for t, raw in sess.responses:
        print(f"    [t={t:5.2f}s] {raw.hex()}")
    print(f"\n  CSV : {logger.csv_path}")
    print(f"  PNG : {logger.png_path}")


if __name__ == "__main__":
    main()
