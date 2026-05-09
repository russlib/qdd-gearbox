"""FTMS connect-only probe for the Saris H2.

Goal: confirm the brake stays released when we connect via standard FTMS
(0x1826) instead of the proprietary CycleOps service. No motor, no torque
writes — just listen to Indoor Bike Data and Control Point indications.

Procedure (per GoldenCheetah BT40Device.cpp):
  1. Connect.
  2. List discovered services. Verify FTMS (0x1826) is exposed.
  3. start_notify on Indoor Bike Data (0x2AD2)  — bleak picks notifications.
  4. start_notify on Control Point (0x2AD9)     — bleak picks indications
     based on the characteristic's properties.
  5. Write REQUEST_CONTROL (0x00) to Control Point.
  6. Listen for the success indication (0x80 0x00 0x01).
  7. Hold for HOLD_S seconds. User spins flywheel by hand and reports whether
     the brake is engaged or free.

NO writes to the proprietary 0xc0f4013a service. Spin the flywheel before
running so the trainer is awake and advertising.
"""
from __future__ import annotations
import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from bleak import BleakClient, BleakScanner
from saris_h2.protocol import DEFAULT_ADDR
from saris_h2.ftms import (
    FTMS_SERVICE_UUID,
    FTMS_INDOOR_BIKE_UUID,
    FTMS_CONTROL_POINT_UUID,
    FTMS_STATUS_UUID,
    FTMS_FEATURE_UUID,
    cmd_request_control,
    parse_indoor_bike,
    parse_control_response,
)

HOLD_S = 30.0


async def connect(addr: str) -> BleakClient:
    print(f"Connecting to {addr}...")
    try:
        client = BleakClient(addr, timeout=15.0)
        await client.connect()
        return client
    except Exception:
        print("Direct connect failed, scanning...")
        device = await BleakScanner.find_device_by_address(addr, timeout=20.0)
        if not device:
            raise SystemExit("Trainer not found. Spin the flywheel and retry.")
        client = BleakClient(device, timeout=15.0)
        await client.connect()
        return client


async def main(addr: str, hold_s: float):
    t0 = time.time()
    def t() -> float: return time.time() - t0

    client = await connect(addr)
    print(f"[t={t():5.2f}s] Connected.")

    # --- Discover and report services ---
    svcs = client.services
    print("\n=== Services discovered ===")
    ftms_present = False
    proprietary_present = False
    for s in svcs:
        marker = ""
        if s.uuid.lower() == FTMS_SERVICE_UUID.lower():
            marker = "  <-- FTMS (target)"
            ftms_present = True
        elif s.uuid.lower().startswith("c0f4013a"):
            marker = "  <-- proprietary CycleOps (avoid)"
            proprietary_present = True
        print(f"  {s.uuid}{marker}")
        for c in s.characteristics:
            props = ",".join(c.properties)
            short = c.uuid.split("-")[0]
            print(f"      char 0x{short[-4:].upper()}  props=[{props}]")
    if not ftms_present:
        print("\n!! FTMS service NOT advertised by this trainer. Aborting.")
        await client.disconnect()
        return
    if proprietary_present:
        print("\n   (proprietary service present but we will NOT touch it)")

    # --- Read FTMS feature ---
    try:
        feat = await client.read_gatt_char(FTMS_FEATURE_UUID)
        print(f"\n[t={t():5.2f}s] FTMS Feature read: {feat.hex()}  (8 bytes machine + 4 bytes targets)")
    except Exception as e:
        print(f"   FTMS Feature read failed: {e}")

    # --- Subscribe to Indoor Bike Data ---
    bike_count = {"n": 0}
    def on_bike(_, data: bytearray):
        m = parse_indoor_bike(bytes(data))
        bike_count["n"] += 1
        if bike_count["n"] <= 2 or bike_count["n"] % 5 == 0:
            print(f"  [t={t():5.2f}s] BIKE  spd={m.inst_speed_kmh}km/h "
                  f"cad={m.inst_cadence_rpm}rpm pwr={m.inst_power_w}W "
                  f"res={m.resistance_level}  raw={m.raw.hex()}")

    await client.start_notify(FTMS_INDOOR_BIKE_UUID, on_bike)
    print(f"[t={t():5.2f}s] Subscribed to Indoor Bike Data (0x2AD2).")

    # --- Subscribe to Control Point (indications) ---
    cp_responses = []
    def on_cp(_, data: bytearray):
        resp = parse_control_response(bytes(data))
        cp_responses.append(resp)
        print(f"  [t={t():5.2f}s] CP    {resp}")

    await client.start_notify(FTMS_CONTROL_POINT_UUID, on_cp)
    print(f"[t={t():5.2f}s] Subscribed to Control Point (0x2AD9) for indications.")

    # --- Subscribe to Status (optional, informational) ---
    def on_status(_, data: bytearray):
        print(f"  [t={t():5.2f}s] STAT  raw={bytes(data).hex()}")
    try:
        await client.start_notify(FTMS_STATUS_UUID, on_status)
        print(f"[t={t():5.2f}s] Subscribed to Status (0x2ADA).")
    except Exception as e:
        print(f"   Status subscribe failed: {e}")

    print(f"\n[t={t():5.2f}s] >>> SPIN THE FLYWHEEL BY HAND. Is it free?  Holding {hold_s:.0f}s before REQUEST_CONTROL...")
    await asyncio.sleep(min(hold_s, 8.0))

    # --- Write REQUEST_CONTROL only ---
    cmd = cmd_request_control()
    print(f"\n[t={t():5.2f}s] WRITE REQUEST_CONTROL ({cmd.hex()}) ...")
    try:
        await client.write_gatt_char(FTMS_CONTROL_POINT_UUID, cmd, response=True)
        print(f"[t={t():5.2f}s] write_gatt_char returned. Awaiting indication...")
    except Exception as e:
        print(f"!! write_gatt_char failed: {e}")

    print(f"\n[t={t():5.2f}s] >>> SPIN THE FLYWHEEL AGAIN. Brake state?")
    await asyncio.sleep(hold_s)

    print(f"\n[t={t():5.2f}s] Disconnecting.")
    await client.stop_notify(FTMS_INDOOR_BIKE_UUID)
    await client.stop_notify(FTMS_CONTROL_POINT_UUID)
    try: await client.stop_notify(FTMS_STATUS_UUID)
    except Exception: pass
    await client.disconnect()
    print(f"\n=== Summary ===")
    print(f"  Indoor Bike notifications: {bike_count['n']}")
    print(f"  Control Point indications: {len(cp_responses)}")
    for r in cp_responses:
        print(f"    {r}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--hold", type=float, default=HOLD_S)
    args = p.parse_args()
    asyncio.run(main(args.addr, args.hold))
