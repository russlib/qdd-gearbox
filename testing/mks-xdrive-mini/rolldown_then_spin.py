"""ROLL_DOWN -> wait for HEADLESS state -> spin motor while BLE connected.

Hypothesis: writing ROLL_DOWN starts a calibration sequence that aborts (no
wheel motion) and lands the trainer in mode HEADLESS, which is supposed to
mean "no resistance." The trainer holds this state only as long as a client
remains connected. Test the motor during this window.

Sequence:
  1. BLE connect + subscribe (this engages brake).
  2. Write ROLL_DOWN.
  3. Poll responses for up to WAIT_HEADLESS_S, looking for a heartbeat
     (cmdId 0x1005) with mode byte == 0x00.
  4. If seen, immediately arm motor and ramp to TARGET_IQ_A in +1 dir.
  5. While motor runs, every REFRESH_S re-issue ROLL_DOWN (or HEADLESS) to
     keep trainer awake in the desired state.
  6. Hold for HOLD_S, log peak RPM.
  7. Disarm + disconnect.

If brake is actually off, motor at 45A in +1 dir should accelerate well past
50 RPM (which has been our braked ceiling all session).

Aborts: FET 70 C, vel 20 t/s.
"""
from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from saris_h2.protocol import (
    DEFAULT_ADDR,
    ResistanceMode,
    build_resistance_cmd,
)
from saris_h2.ble_threaded import SarisH2Session
from _logger import TestLogger

KT_NMA          = 0.04
DIRECTION       = +1
TARGET_IQ_A     = 45.0
RAMP_S          = 4.0
HOLD_S          = 15.0
LOOP_DT_S       = 0.05
VEL_ABORT_T_S   = 20.0
TEMP_ABORT_C    = 70.0
CURRENT_LIM_A   = 60.0

WAIT_HEADLESS_S = 6.0    # max wait for trainer to enter HEADLESS heartbeat
REFRESH_S       = 5.0    # re-issue keep-alive command every X seconds during motor run


def find_headless_event(events: list[tuple[float, str, str]],
                        since_t: float = 0.0) -> tuple[float, str] | None:
    """Look for a `resp` event whose payload is a 0x1005 heartbeat with mode=0x00.

    Format we care about:
      bytes 0..1: 01 00
      bytes 2..3: 05 10  (cmdId 0x1005 little-endian)
      byte    4 : current mode  (we want 0x00 = HEADLESS)
    """
    for t, kind, info in events:
        if t < since_t:
            continue
        if kind != "resp":
            continue
        # info is "hex=<hex>"
        hex_part = info.split("hex=", 1)[-1]
        try:
            raw = bytes.fromhex(hex_part)
        except ValueError:
            continue
        if len(raw) < 5:
            continue
        cmd_id = int.from_bytes(raw[2:4], "little")
        mode = raw[4]
        if cmd_id == 0x1005 and mode == 0x00:
            return t, hex_part
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--target-iq", type=float, default=TARGET_IQ_A)
    p.add_argument("--hold", type=float, default=HOLD_S)
    p.add_argument("--keepalive-mode", default="ROLL_DOWN",
                   choices=["ROLL_DOWN", "HEADLESS", "WARM_UP", "none"])
    p.add_argument("--skip-motor", action="store_true",
                   help="Run BLE side only (BLE diagnostics, no motor)")
    args = p.parse_args()

    print(f"========== ROLLDOWN_THEN_SPIN ==========")
    print(f"  target_iq={args.target_iq}A  hold={args.hold}s  keepalive={args.keepalive_mode}")

    # --- BLE up ---
    sess = SarisH2Session(args.addr, subscribe_proprietary=True)
    print("  [BLE] starting session...")
    sess.start()
    print("  [BLE] connected + subscribed.")

    # --- Pre-sequence: prime trainer's fallback state to HEADLESS ---
    # This replicates the sanity-test sequence that made the trainer transition
    # to HEADLESS after ROLL_DOWN failed. Hypothesis: HEADLESS write registers
    # as the "fallback" state, then ROLL_DOWN's auto-fail lands us back in it.
    pre_seq = [
        ("HEADLESS_pre", build_resistance_cmd(ResistanceMode.HEADLESS, 0, 0)),
        ("WARM_UP_pre",  build_resistance_cmd(ResistanceMode.WARM_UP,  0, 0)),
    ]
    for label, cmd in pre_seq:
        print(f"  [BLE] pre-seq writing {label} ({cmd.hex()})...")
        sess.write_proprietary(label, cmd, timeout_s=3.0)
        time.sleep(2.5)

    # --- Send ROLL_DOWN ---
    rd_cmd = build_resistance_cmd(ResistanceMode.ROLL_DOWN, 0, 0)
    rd_t = time.time()
    print(f"  [BLE] writing ROLL_DOWN ({rd_cmd.hex()})...")
    if not sess.write_proprietary("ROLL_DOWN", rd_cmd, timeout_s=3.0):
        print("  !! ROLL_DOWN write failed; aborting")
        sess.stop()
        return

    # --- Poll for HEADLESS heartbeat ---
    print(f"  [BLE] waiting up to {WAIT_HEADLESS_S}s for HEADLESS heartbeat (mode=0x00, cmdId=0x1005)...")
    t_start = time.time()
    headless_seen_at = None
    while time.time() - t_start < WAIT_HEADLESS_S:
        # Snapshot the events list (thread-safe due to GIL on append/read)
        snapshot = list(sess.events)
        # Convert sess.t0 (wall time start) to our reference: ble events are
        # in sess-relative seconds since t0; we just look at all events.
        hit = find_headless_event(snapshot, since_t=0.0)
        if hit:
            ev_t, hex_str = hit
            headless_seen_at = ev_t
            print(f"  [BLE] HEADLESS heartbeat seen at t={ev_t:.2f}s  hex={hex_str}")
            break
        time.sleep(0.1)

    if headless_seen_at is None:
        print(f"  !! No HEADLESS heartbeat in {WAIT_HEADLESS_S}s — trainer didn't transition.")
        print(f"     Will run motor anyway to record outcome.")
    else:
        print(f"  [BLE] proceeding to motor test with trainer in HEADLESS state.")

    if args.skip_motor:
        time.sleep(2.0)
        sess.stop()
        return

    # --- Motor side ---
    import odrive
    odrv = odrive.find_any(timeout=20)
    if odrv is None:
        sess.stop()
        raise SystemExit("USB enum failed")
    ax = odrv.axis0
    mc = ax.motor.config
    cc = ax.controller.config
    print(f"  [ODRV] bus={odrv.vbus_voltage:.2f}V  state={ax.current_state}  FET={ax.fet_thermistor.temperature:.1f}C")
    if ax.fet_thermistor.temperature > 50.0:
        print(f"  !! FET={ax.fet_thermistor.temperature:.1f}C is high. Aborting; let it cool first.")
        sess.stop()
        return

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
        sess.stop()
        raise SystemExit(f"arm failed: state={ax.current_state} err={ax.error}")

    target_T = DIRECTION * args.target_iq * KT_NMA
    logger = TestLogger("rolldown_then_spin", extra_fields=["ble_keepalive_t"])
    abort = None
    peak_rpm = 0.0
    peak_fet = 0.0
    last_keepalive = time.time()
    last_print = -10.0

    t0 = time.time()
    def now(): return time.time() - t0

    keepalive_count = 0

    def maybe_keepalive():
        nonlocal last_keepalive, keepalive_count
        if args.keepalive_mode == "none": return
        if time.time() - last_keepalive < REFRESH_S: return
        if args.keepalive_mode == "ROLL_DOWN":
            cmd = build_resistance_cmd(ResistanceMode.ROLL_DOWN, 0, 0)
        elif args.keepalive_mode == "HEADLESS":
            cmd = build_resistance_cmd(ResistanceMode.HEADLESS, 0, 0)
        elif args.keepalive_mode == "WARM_UP":
            cmd = build_resistance_cmd(ResistanceMode.WARM_UP, 0, 0)
        else:
            return
        ok = sess.write_proprietary(f"keepalive_{args.keepalive_mode}", cmd, timeout_s=2.0)
        last_keepalive = time.time()
        keepalive_count += 1
        print(f"    [BLE] keepalive {args.keepalive_mode} ({'ok' if ok else 'FAIL'})")

    try:
        # Ramp
        rs = time.time()
        while time.time() - rs < RAMP_S:
            cmd = target_T * (time.time() - rs) / RAMP_S
            ax.controller.input_torque = cmd
            t = now()
            vel = ax.encoder.vel_estimate
            pos = ax.encoder.pos_estimate
            iq  = ax.motor.current_control.Iq_measured
            temp = ax.fet_thermistor.temperature
            errs = ax.error | ax.motor.error | ax.encoder.error
            rpm = abs(vel) * 60.0
            if rpm > peak_rpm: peak_rpm = rpm
            if temp > peak_fet: peak_fet = temp
            logger.row(t_s=t, phase="ramp", cmd_nm=cmd, iq_a=iq, vel_t_s=vel,
                       pos_turns=pos, fet_c=temp, axis_err=ax.error,
                       motor_err=ax.motor.error, encoder_err=ax.encoder.error,
                       ble_keepalive_t=keepalive_count)
            if t - last_print > 0.5:
                print(f"    t={t:5.1f}s  vel={vel*60:+7.1f} RPM  Iq={iq:+5.1f}  FET={temp:.1f}C  ramp")
                last_print = t
            if errs: abort = f"err: {ax.error}"; break
            if abs(vel) > VEL_ABORT_T_S: abort = "vel limit"; break
            if temp >= TEMP_ABORT_C: abort = f"FET {temp:.1f}C"; break
            time.sleep(LOOP_DT_S)

        # Hold
        if not abort:
            he = time.time() + args.hold
            while time.time() < he:
                ax.controller.input_torque = target_T
                maybe_keepalive()
                t = now()
                vel = ax.encoder.vel_estimate
                pos = ax.encoder.pos_estimate
                iq  = ax.motor.current_control.Iq_measured
                temp = ax.fet_thermistor.temperature
                errs = ax.error | ax.motor.error | ax.encoder.error
                rpm = abs(vel) * 60.0
                if rpm > peak_rpm: peak_rpm = rpm
                if temp > peak_fet: peak_fet = temp
                logger.row(t_s=t, phase="hold", cmd_nm=target_T, iq_a=iq, vel_t_s=vel,
                           pos_turns=pos, fet_c=temp, axis_err=ax.error,
                           motor_err=ax.motor.error, encoder_err=ax.encoder.error,
                           ble_keepalive_t=keepalive_count)
                if t - last_print > 0.5:
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
        except Exception:
            pass
        sess.stop()

    logger.close()
    logger.plot(title=f"rolldown_then_spin @ {args.target_iq}A keepalive={args.keepalive_mode}",
                subtitle=f"peak_rpm={peak_rpm:.1f}  peak_fet={peak_fet:.1f}C")

    print(f"\n========== RESULT ==========")
    print(f"  peak_rpm  : {peak_rpm:.1f}")
    print(f"  peak_fet  : {peak_fet:.1f} C")
    print(f"  abort     : {abort or 'completed'}")
    print(f"  headless heartbeat seen at: {headless_seen_at}")
    print(f"  keepalives sent: {keepalive_count}")
    print(f"  ble cps count: {sess.cps_count}, last power: {sess.last_power}")

    counts = sess.event_count_by_kind()
    print(f"  ble event counts: {counts}")

    print(f"\n  CSV : {logger.csv_path}")
    print(f"  PNG : {logger.png_path}")

    # Show resp events for debugging
    print(f"\n  BLE responses received:")
    for t, kind, info in sess.events:
        if kind == "resp":
            print(f"    [t={t:6.2f}s] {info}")


if __name__ == "__main__":
    main()
