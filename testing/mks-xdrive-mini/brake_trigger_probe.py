"""Localize what GATT operation engages the H2 brake.

The trainer doesn't expose FTMS, so GoldenCheetah's path doesn't apply
directly. Hypothesis: the brake might engage when ANY GATT op happens
against ANY service, not just the proprietary CycleOps one. This probe
walks through escalating GATT activity in three trials with user
self-report between each.

Trials (run in order, separately, with disconnect between):
  --trial 1 : connect only (no read/notify/write), hold, disconnect.
  --trial 2 : connect + start_notify(0x2A63 CPS measurement), hold, disconnect.
  --trial 3 : connect + start_notify(0x2A63) + start_notify(0x2A66 CP Control Point), hold.

NEVER touches the proprietary 0xc0f4013a service.
The user reports brake state (free / braked) when prompted.
"""
from __future__ import annotations
import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from bleak import BleakClient, BleakScanner
from saris_h2.protocol import DEFAULT_ADDR, CPS_MEASUREMENT_UUID, parse_cps

CPS_CONTROL_POINT_UUID = "00002a66-0000-1000-8000-00805f9b34fb"
PROPRIETARY_PREFIX = "c0f4013a"


async def connect(addr: str) -> BleakClient:
    print(f"Connecting to {addr}...")
    try:
        client = BleakClient(addr, timeout=15.0)
        await client.connect()
        return client
    except Exception:
        device = await BleakScanner.find_device_by_address(addr, timeout=20.0)
        if not device:
            raise SystemExit("Trainer not found. Spin flywheel + retry.")
        client = BleakClient(device, timeout=15.0)
        await client.connect()
        return client


async def trial(addr: str, trial_num: int, hold_s: float):
    t0 = time.time()
    def t() -> float: return time.time() - t0

    client = await connect(addr)
    print(f"[t={t():5.2f}s] Connected.  TRIAL {trial_num}")

    # Confirm we are not touching the proprietary service
    proprietary_seen = any(s.uuid.lower().startswith(PROPRIETARY_PREFIX) for s in client.services)
    print(f"   proprietary service in advertisement: {proprietary_seen}  (will NOT touch)")

    cps_count = {"n": 0, "last_power": None}
    if trial_num >= 2:
        def on_cps(_, data: bytearray):
            try:
                m = parse_cps(bytes(data))
                cps_count["n"] += 1
                cps_count["last_power"] = m.power_w
                if cps_count["n"] <= 2 or cps_count["n"] % 5 == 0:
                    print(f"  [t={t():5.2f}s] CPS  pwr={m.power_w}W  rpm_revs={m.wheel_revs}  "
                          f"acc_t={m.acc_torque_nm:.2f}Nm")
            except Exception as e:
                print(f"  [t={t():5.2f}s] CPS parse err: {e}")
        await client.start_notify(CPS_MEASUREMENT_UUID, on_cps)
        print(f"[t={t():5.2f}s] Subscribed to CPS measurement (0x2A63).")

    if trial_num >= 3:
        def on_cp(_, data: bytearray):
            print(f"  [t={t():5.2f}s] CP-CTRL ind: {bytes(data).hex()}")
        try:
            await client.start_notify(CPS_CONTROL_POINT_UUID, on_cp)
            print(f"[t={t():5.2f}s] Subscribed to CPS Control Point (0x2A66) indications.")
        except Exception as e:
            print(f"   CP subscribe failed: {e}")

    print(f"\n[t={t():5.2f}s] >>> SPIN THE FLYWHEEL BY HAND. Free or braked?")
    print(f"      Holding {hold_s:.0f}s...")

    # Periodic prompts during hold
    end = time.time() + hold_s
    next_prompt = time.time() + 5.0
    while time.time() < end:
        await asyncio.sleep(0.2)
        if time.time() >= next_prompt:
            elapsed = t()
            remaining = end - time.time()
            print(f"  [t={elapsed:5.2f}s] still listening, {remaining:.0f}s left.  "
                  f"CPS samples so far: {cps_count['n']}  last power: {cps_count['last_power']}")
            next_prompt = time.time() + 5.0

    print(f"\n[t={t():5.2f}s] Disconnecting.")
    if trial_num >= 2:
        try: await client.stop_notify(CPS_MEASUREMENT_UUID)
        except Exception: pass
    if trial_num >= 3:
        try: await client.stop_notify(CPS_CONTROL_POINT_UUID)
        except Exception: pass
    await client.disconnect()
    print(f"=== TRIAL {trial_num} done ===")
    print(f"   CPS notifications received: {cps_count['n']}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--trial", type=int, choices=[1, 2, 3], required=True,
                   help="1=connect only, 2=+CPS subscribe, 3=+CPS-CP subscribe")
    p.add_argument("--hold", type=float, default=15.0)
    args = p.parse_args()
    asyncio.run(trial(args.addr, args.trial, args.hold))
