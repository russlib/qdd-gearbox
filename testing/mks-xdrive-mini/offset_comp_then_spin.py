"""Trigger standard CPS Start-Offset-Compensation (0x0C) and immediately spin motor.

Major finding from cps_control_probe.py: H2 firmware DOES support the
BT-spec opcode 0x0C on char 0x2A66, returning 20 0c 01 ff ff (Success,
offset uncalibrated). Per spec, this opcode triggers a spindown calibration
sequence which requires the trainer to release the brake during the
deceleration window.

This script:
  1. Connect, subscribe to CPS measurement (0x2A63) + CPS Control Point (0x2A66)
  2. Write 0x0C to CP
  3. Verify success indication
  4. Within same connection, drive motor at TARGET_IQ_A in +1 dir
  5. Look for RPM blowing past ~50 RPM ceiling = brake released

If we see RPM > 80 RPM during the test, the cal window is the brake-off window
we've been chasing.
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

CPS_CP_UUID = "00002a66-0000-1000-8000-00805f9b34fb"

KT_NMA          = 0.04
DIRECTION       = +1
TARGET_IQ_A     = 30.0
RAMP_S          = 4.0
HOLD_S          = 25.0
LOOP_DT_S       = 0.05
VEL_ABORT_T_S   = 20.0
TEMP_ABORT_C    = 70.0
CURRENT_LIM_A   = 60.0


# --- Threaded BLE: run asyncio loop on a dedicated thread ---

class CpsSession:
    def __init__(self, addr: str):
        self.addr = addr
        self._client: BleakClient | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._setup_error: BaseException | None = None
        self.events: list[tuple[float, str, str]] = []
        self.t0: float | None = None
        self.cps_count = 0
        self.cps_last_power = None
        self.cp_responses: list[tuple[float, bytes]] = []

    def _ev(self, kind: str, info: str):
        t = (time.time() - self.t0) if self.t0 else 0.0
        self.events.append((t, kind, info))

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
        self._ev("connected", "")

        def on_meas(_, data: bytearray):
            try:
                m = parse_cps(bytes(data))
                self.cps_count += 1
                self.cps_last_power = m.power_w
                self._ev("cps", f"pwr={m.power_w}W")
            except Exception as e:
                self._ev("cps_err", str(e))

        def on_cp(_, data: bytearray):
            t_rel = (time.time() - self.t0) if self.t0 else 0.0
            self.cp_responses.append((t_rel, bytes(data)))
            self._ev("cp_ind", f"hex={bytes(data).hex()}")

        await self._client.start_notify(CPS_MEASUREMENT_UUID, on_meas)
        self._ev("subscribed_cps", "")
        await self._client.start_notify(CPS_CP_UUID, on_cp)
        self._ev("subscribed_cp", "")

    async def _teardown(self):
        if self._client is None: return
        for u in (CPS_MEASUREMENT_UUID, CPS_CP_UUID):
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
            except Exception: pass
            return
        self._ready.set()
        try:
            self._loop.run_forever()
        finally:
            try: self._loop.run_until_complete(self._teardown())
            except Exception: pass
            self._loop.close()

    def start(self):
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=25.0)
        if self._setup_error:
            raise self._setup_error

    def write_cp(self, payload: bytes, timeout_s: float = 3.0) -> bool:
        async def _do():
            await self._client.write_gatt_char(CPS_CP_UUID, payload, response=True)
        fut = asyncio.run_coroutine_threadsafe(_do(), self._loop)
        try:
            fut.result(timeout=timeout_s)
            self._ev("cp_write_ok", payload.hex())
            return True
        except FutTimeout:
            self._ev("cp_write_timeout", payload.hex())
            return False
        except Exception as e:
            self._ev("cp_write_err", f"{e!r}")
            return False

    def stop(self):
        if self._loop is None: return
        try: self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception: pass
        if self._thread: self._thread.join(timeout=10.0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--target-iq", type=float, default=TARGET_IQ_A)
    p.add_argument("--hold", type=float, default=HOLD_S)
    p.add_argument("--retrigger-every", type=float, default=0,
                   help="re-issue 0x0C every N seconds during motor run (0=never)")
    p.add_argument("--skip-motor", action="store_true")
    args = p.parse_args()

    print(f"========== OFFSET_COMP_THEN_SPIN ==========")
    print(f"  target_iq={args.target_iq}A  hold={args.hold}s  retrigger_every={args.retrigger_every}s")

    sess = CpsSession(args.addr)
    print("  [BLE] starting session...")
    sess.start()
    print("  [BLE] connected + subscribed.")

    print(f"  [BLE] writing START_OFFSET_COMPENSATION (0x0C)...")
    ok = sess.write_cp(bytes([0x0C]), timeout_s=3.0)
    print(f"  [BLE]   ok={ok}")
    print(f"  [BLE] settling 2s for indication...")
    time.sleep(2.0)
    print(f"  [BLE] CP responses so far: {len(sess.cp_responses)}")
    for t, raw in sess.cp_responses:
        print(f"    [t={t:5.2f}s] {raw.hex()}")

    if args.skip_motor:
        time.sleep(2.0)
        sess.stop()
        return

    # --- Motor side ---
    import odrive
    odrv = odrive.find_any(timeout=15)
    if odrv is None:
        sess.stop(); raise SystemExit("USB enum failed")
    ax = odrv.axis0
    mc = ax.motor.config
    cc = ax.controller.config
    print(f"  [ODRV] bus={odrv.vbus_voltage:.2f}V  state={ax.current_state}  FET={ax.fet_thermistor.temperature:.1f}C")

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
        sess.stop(); raise SystemExit(f"arm failed: state={ax.current_state} err={ax.error}")

    target_T = DIRECTION * args.target_iq * KT_NMA
    logger = TestLogger("offset_comp_then_spin")
    abort = None; peak_rpm = 0.0; peak_fet = 0.0
    last_print = -10.0
    last_retrigger = time.time()
    retrigger_count = 0

    t0 = time.time()
    def now(): return time.time() - t0

    def maybe_retrigger():
        nonlocal last_retrigger, retrigger_count
        if args.retrigger_every <= 0: return
        if time.time() - last_retrigger < args.retrigger_every: return
        ok = sess.write_cp(bytes([0x0C]), timeout_s=2.0)
        last_retrigger = time.time()
        retrigger_count += 1
        print(f"    [BLE] re-trigger 0x0C #{retrigger_count} ({'ok' if ok else 'FAIL'})")

    try:
        rs = time.time()
        while time.time() - rs < RAMP_S:
            cmd = target_T * (time.time() - rs) / RAMP_S
            ax.controller.input_torque = cmd
            t = now()
            vel = ax.encoder.vel_estimate
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
                maybe_retrigger()
                t = now()
                vel = ax.encoder.vel_estimate
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
            time.sleep(0.3)
            ax.requested_state = 1
        except Exception: pass
        sess.stop()

    logger.close()
    logger.plot(title=f"offset_comp_then_spin @ {args.target_iq}A",
                subtitle=f"peak_rpm={peak_rpm:.1f}  peak_fet={peak_fet:.1f}C")

    print(f"\n========== RESULT ==========")
    print(f"  peak_rpm  : {peak_rpm:.1f}")
    print(f"  peak_fet  : {peak_fet:.1f} C")
    print(f"  abort     : {abort or 'completed'}")
    print(f"  retriggers sent: {retrigger_count}")
    print(f"  cps cnt   : {sess.cps_count}  last_power: {sess.cps_last_power}")
    print(f"\n  CP indications:")
    for t, raw in sess.cp_responses:
        print(f"    [t={t:5.2f}s] {raw.hex()}")
    print(f"\n  CSV : {logger.csv_path}")
    print(f"  PNG : {logger.png_path}")


if __name__ == "__main__":
    main()
