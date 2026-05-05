"""ERG-mode resistance sweep over BLE while logging trainer telemetry.

Connects to the Saris H2, subscribes to CPS notifications (telemetry) and the
proprietary resistance characteristic (required to enable writes). Then steps
through ERG target wattages at scheduled times, logging both the trainer-side
CPS data and the wall-clock timestamps of each command write.

Run a parallel motor torque hold (e.g. long_hold_30a.py) for the matching
duration to get motor-side data on the same timeline.
"""
from __future__ import annotations
import argparse
import asyncio
import csv
import struct
import sys
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
)

OUT_DIR = Path(__file__).parent / "thermal_data"
OUT_DIR.mkdir(exist_ok=True)
ts = time.strftime("%Y%m%d-%H%M%S")
CSV_PATH = OUT_DIR / f"ble_erg_sweep_{ts}.csv"

# (relative_t_s, label, mode, param1, param2)
SCHEDULE = [
    ( 0.0, "subscribe_only_brake_default",  None, 0,    0),
    ( 5.0, "ERG_0W",                        0x01, 0,    0),
    (20.0, "ERG_50W",                       0x01, 50,   0),
    (35.0, "ERG_100W",                      0x01, 100,  0),
    (50.0, "ERG_200W",                      0x01, 200,  0),
    (65.0, "ERG_500W",                      0x01, 500,  0),
    (80.0, "ERG_1000W",                     0x01, 1000, 0),
    (95.0, "DONE",                          None, 0,    0),
]
TOTAL_DURATION_S = 100.0

# Track the latest commanded label so we tag every CPS sample
state = {"label": "subscribe_only_brake_default", "t0": None}


def parse_cps(data: bytes) -> dict:
    flags = struct.unpack_from("<H", data, 0)[0]
    power_w = struct.unpack_from("<h", data, 2)[0]
    offset = 4
    if flags & 0x01: offset += 1
    acc_torque = None; wheel_revs = None; wheel_event = None
    if flags & 0x04:
        raw = struct.unpack_from("<H", data, offset)[0]
        acc_torque = raw / 32.0
        offset += 2
    if flags & 0x10:
        wheel_revs = struct.unpack_from("<I", data, offset)[0]
        wheel_event = struct.unpack_from("<H", data, offset + 4)[0]
        offset += 6
    return {"power_w": power_w, "acc_torque_nm": acc_torque,
            "wheel_revs": wheel_revs, "wheel_event_time": wheel_event}


def cps_handler_factory(writer):
    def on_cps(sender, data: bytearray):
        if state["t0"] is None: return
        t = time.time() - state["t0"]
        m = parse_cps(bytes(data))
        writer.writerow({
            "t_s": f"{t:.3f}",
            "kind": "cps",
            "label": state["label"],
            "power_w": m["power_w"],
            "acc_torque_nm": f"{m['acc_torque_nm']:.4f}" if m["acc_torque_nm"] is not None else "",
            "wheel_revs": m["wheel_revs"] if m["wheel_revs"] is not None else "",
            "wheel_event_time": m["wheel_event_time"] if m["wheel_event_time"] is not None else "",
            "raw_hex": bytes(data).hex(),
        })
    return on_cps


def resistance_handler_factory(writer):
    def on_resistance(sender, data: bytearray):
        if state["t0"] is None: return
        t = time.time() - state["t0"]
        # Saris response: bytes[2..3] cmdId, [4] mode, [5..6] p1, [7..8] p2, [9] status
        cmd_id = int.from_bytes(bytes(data[2:4]), "little") if len(data) >= 4 else None
        writer.writerow({
            "t_s": f"{t:.3f}",
            "kind": "resistance_resp",
            "label": state["label"],
            "power_w": "",
            "acc_torque_nm": "",
            "wheel_revs": "",
            "wheel_event_time": "",
            "raw_hex": bytes(data).hex(),
        })
    return on_resistance


async def connect_h2(addr: str) -> BleakClient:
    print(f"Connecting to H2 at {addr}...")
    try:
        client = BleakClient(addr, timeout=15.0)
        await client.connect()
        return client
    except Exception:
        device = await BleakScanner.find_device_by_address(addr, timeout=20.0)
        if not device:
            print("Trainer not found. Wake the flywheel and retry.")
            raise SystemExit(1)
        client = BleakClient(device, timeout=15.0)
        await client.connect()
        return client


async def main(addr: str):
    csv_file = open(CSV_PATH, "w", newline="")
    fieldnames = ["t_s", "kind", "label", "power_w", "acc_torque_nm",
                  "wheel_revs", "wheel_event_time", "raw_hex"]
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()

    client = await connect_h2(addr)
    print("Connected.")
    try:
        # Subscribe to resistance char first (qdomyos-zwift convention; engages brake)
        await client.start_notify(SARIS_RESISTANCE_UUID, resistance_handler_factory(writer))
        await client.start_notify(CPS_MEASUREMENT_UUID, cps_handler_factory(writer))
        print("Subscribed to CPS + resistance notifications.")

        state["t0"] = time.time()
        print(f"\nSweep schedule (relative seconds):")
        for (rt, label, mode, p1, p2) in SCHEDULE:
            print(f"  t={rt:>5.1f}s  {label}  (mode={mode}, p1={p1}, p2={p2})")

        next_idx = 0
        while True:
            t = time.time() - state["t0"]
            if next_idx < len(SCHEDULE):
                rt, label, mode, p1, p2 = SCHEDULE[next_idx]
                if t >= rt:
                    state["label"] = label
                    if mode is not None:
                        cmd = build_resistance_cmd(ResistanceMode(mode), p1, p2)
                        await client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd, response=False)
                        print(f"[t={t:6.2f}s] WRITE {label}  ({cmd.hex()})")
                    else:
                        print(f"[t={t:6.2f}s] STATE -> {label}")
                    next_idx += 1
            if t >= TOTAL_DURATION_S:
                break
            await asyncio.sleep(0.1)

    finally:
        try:
            await client.disconnect()
            print("Disconnected.")
        except Exception:
            pass
        csv_file.close()
        print(f"\nCSV: {CSV_PATH}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR, help="BLE address override")
    args = p.parse_args()
    asyncio.run(main(args.addr))
