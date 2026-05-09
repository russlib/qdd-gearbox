"""Localize brake-on trigger by RPM ceiling under fixed motor torque.

Past tests show motor caps at ~49 RPM at 30-55A when the brake is engaged
(push_to_500_20260505-140607.csv shows 49.4 RPM peak even at 55A).
Free-spin RPM at 30A should blow past that easily.

Three trials in one run, motor held at 30A for ~25s each:
  A : no BLE connection at all              (baseline free spin)
  B : BLE connected, no subscribes, no writes
  C : BLE connected + CPS measurement subscribe only

The proprietary 0xc0f4013a service is NEVER touched.

Inter-trial cooldown lets the motor cool. Each trial logs to its own CSV/PNG
under thermal_data/.
"""
from __future__ import annotations
import argparse
import asyncio
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

import odrive
from bleak import BleakClient, BleakScanner
from saris_h2.protocol import DEFAULT_ADDR, CPS_MEASUREMENT_UUID, parse_cps
from _logger import TestLogger

KT_NMA          = 0.04
DIRECTION       = +1
TARGET_IQ_A     = 30.0
HOLD_S          = 15.0
RAMP_S          = 5.0
COOLDOWN_S      = 20.0
LOOP_DT_S       = 0.05
VEL_ABORT_T_S   = 20.0
TEMP_ABORT_C    = 78.0
CURRENT_LIM_A   = 60.0


def arm_motor(odrv, ax, mc, cc):
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


def disarm_motor(ax):
    ax.controller.input_torque = 0
    time.sleep(0.3)
    ax.requested_state = 1


def torque_hold(label: str, ax, hold_s: float, ramp_s: float):
    """Hold motor at TARGET_IQ_A. Returns (peak_rpm, peak_fet, abort_reason)."""
    target_T = DIRECTION * TARGET_IQ_A * KT_NMA
    logger = TestLogger(f"brake_probe_{label}")
    peak_rpm = 0.0
    peak_fet = 0.0
    abort = None
    last_print = -10.0
    t0 = time.time()
    def now(): return time.time() - t0

    print(f"  --- motor torque hold: ramp {ramp_s:.1f}s -> {target_T:+.2f} Nm, hold {hold_s:.0f}s ---")
    try:
        # Ramp
        rs = time.time()
        while time.time() - rs < ramp_s:
            cmd = target_T * (time.time() - rs) / ramp_s
            ax.controller.input_torque = cmd
            t = now()
            vel = ax.encoder.vel_estimate
            pos = ax.encoder.pos_estimate
            iq = ax.motor.current_control.Iq_measured
            temp = ax.fet_thermistor.temperature
            errs = ax.error | ax.motor.error | ax.encoder.error
            rpm = abs(vel) * 60.0
            if rpm > peak_rpm: peak_rpm = rpm
            if temp > peak_fet: peak_fet = temp
            logger.row(t_s=t, phase="ramp", cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
                       pos_turns=pos, fet_c=temp, axis_err=ax.error,
                       motor_err=ax.motor.error, encoder_err=ax.encoder.error)
            if errs:
                abort = f"err: ax={ax.error} m={ax.motor.error} e={ax.encoder.error}"; break
            if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
            if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
            time.sleep(LOOP_DT_S)

        # Hold
        if abort is None:
            he = time.time() + hold_s
            while time.time() < he:
                ax.controller.input_torque = target_T
                t = now()
                vel = ax.encoder.vel_estimate
                pos = ax.encoder.pos_estimate
                iq = ax.motor.current_control.Iq_measured
                temp = ax.fet_thermistor.temperature
                errs = ax.error | ax.motor.error | ax.encoder.error
                rpm = abs(vel) * 60.0
                if rpm > peak_rpm: peak_rpm = rpm
                if temp > peak_fet: peak_fet = temp
                logger.row(t_s=t, phase="hold", cmd_nm=target_T, iq_a=iq, vel_t_s=vel,
                           pos_turns=pos, fet_c=temp, axis_err=ax.error,
                           motor_err=ax.motor.error, encoder_err=ax.encoder.error)
                if t - last_print > 2.0:
                    print(f"    t={t:5.1f}s  vel={vel*60:+6.1f} RPM  Iq={iq:+5.1f}  FET={temp:.1f}C")
                    last_print = t
                if errs:
                    abort = f"err: ax={ax.error} m={ax.motor.error} e={ax.encoder.error}"; break
                if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
                if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
                time.sleep(LOOP_DT_S)
    finally:
        ax.controller.input_torque = 0
        time.sleep(0.3)
    logger.close()
    logger.plot(title=f"brake probe: {label}",
                subtitle=f"peak_rpm={peak_rpm:.1f}  peak_fet={peak_fet:.1f}C  abort={abort or 'ok'}")
    print(f"    -> peak RPM = {peak_rpm:.1f}, peak FET = {peak_fet:.1f}C, abort={abort or 'completed'}")
    print(f"    -> {logger.csv_path}")
    return peak_rpm, peak_fet, abort


# --- BLE side, runs in its own thread with its own asyncio loop ---

class BleBackground:
    """Manages a Bleak client lifecycle on a background thread for one trial."""
    def __init__(self, addr: str, mode: str):
        self.addr = addr
        self.mode = mode  # "none", "connect", "cps"
        self.thread: threading.Thread | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.client: BleakClient | None = None
        self.cps_count = 0
        self.last_power = None
        self.ready_event = threading.Event()
        self.stop_event = threading.Event()
        self.error: Exception | None = None

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

            if self.mode == "cps":
                def on_cps(_, data: bytearray):
                    try:
                        m = parse_cps(bytes(data))
                        self.cps_count += 1
                        self.last_power = m.power_w
                    except Exception:
                        pass
                await self.client.start_notify(CPS_MEASUREMENT_UUID, on_cps)

            self.ready_event.set()
            while not self.stop_event.is_set():
                await asyncio.sleep(0.1)

            if self.mode == "cps":
                try: await self.client.stop_notify(CPS_MEASUREMENT_UUID)
                except Exception: pass
            await self.client.disconnect()
        except Exception as e:
            self.error = e
            self.ready_event.set()

    def start(self):
        if self.mode == "none":
            self.ready_event.set()
            return
        def thread_target():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._run())
            self.loop.close()
        self.thread = threading.Thread(target=thread_target, daemon=True)
        self.thread.start()
        self.ready_event.wait(timeout=25.0)
        if self.error:
            raise self.error

    def stop(self):
        if self.mode == "none":
            return
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=10.0)


def main(addr: str, hold_s: float, trials: list[str]):
    odrv = odrive.find_any(timeout=20)
    if odrv is None:
        raise SystemExit("USB enum failed")
    ax = odrv.axis0
    mc = ax.motor.config
    cc = ax.controller.config

    print(f"== Pre-test ==")
    print(f"  bus={odrv.vbus_voltage:.2f}V  state={ax.current_state}  FET={ax.fet_thermistor.temperature:.1f}C")
    arm_motor(odrv, ax, mc, cc)
    print(f"  armed in TORQUE mode\n")

    plan = {
        "A": ("none",    "no BLE connection at all"),
        "B": ("connect", "BLE connected, no subscribes, no writes"),
        "C": ("cps",     "BLE connected + CPS measurement subscribe"),
    }

    summary = []
    try:
        for tlabel in trials:
            mode, desc = plan[tlabel]
            print(f"\n========== TRIAL {tlabel}: {desc} ==========")
            ble = BleBackground(addr, mode)
            try:
                if mode != "none":
                    print(f"  -> BLE: connecting in mode '{mode}'...")
                    ble.start()
                    print(f"  -> BLE ready (mode={mode}).")
                else:
                    print(f"  -> no BLE for this trial.")

                peak_rpm, peak_fet, abort = torque_hold(f"trial{tlabel}_{mode}", ax, hold_s, RAMP_S)
                summary.append((tlabel, mode, peak_rpm, peak_fet, abort, ble.cps_count if mode == "cps" else None))
            finally:
                if mode != "none":
                    print(f"  -> BLE: disconnecting (cps_count={ble.cps_count}, last_power={ble.last_power}).")
                    ble.stop()

            if tlabel != trials[-1]:
                print(f"\n  cooldown {COOLDOWN_S:.0f}s before next trial...")
                cooldown_end = time.time() + COOLDOWN_S
                while time.time() < cooldown_end:
                    temp = ax.fet_thermistor.temperature
                    rem = cooldown_end - time.time()
                    print(f"    t-{rem:5.1f}s  FET={temp:.1f}C")
                    time.sleep(5.0)
    finally:
        disarm_motor(ax)

    print(f"\n========== SUMMARY ==========")
    print(f"  {'trial':>6} {'mode':>10} {'peak_rpm':>10} {'peak_fet':>9} {'abort':>20} {'cps_n':>6}")
    for s in summary:
        tlabel, mode, peak_rpm, peak_fet, abort, cps_n = s
        print(f"  {tlabel:>6} {mode:>10} {peak_rpm:>10.1f} {peak_fet:>8.1f}C {str(abort or 'ok'):>20} {str(cps_n):>6}")
    print()
    if len(summary) >= 2:
        a_rpm = summary[0][2]
        for s in summary[1:]:
            print(f"  trial {s[0]} vs trial A peak RPM: {s[2]:.1f} vs {a_rpm:.1f}  "
                  f"(delta {s[2]-a_rpm:+.1f}, ratio {s[2]/max(a_rpm,0.01):.2f})")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--hold", type=float, default=HOLD_S)
    p.add_argument("--trials", default="A,B,C", help="comma-separated subset of A,B,C")
    args = p.parse_args()
    trials = [t.strip().upper() for t in args.trials.split(",") if t.strip()]
    for t in trials:
        if t not in ("A", "B", "C"):
            raise SystemExit(f"unknown trial: {t}")
    main(args.addr, args.hold, trials)
