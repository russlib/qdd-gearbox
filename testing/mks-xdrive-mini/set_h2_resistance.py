"""Set the Saris H2 resistance via BLE.

Verified protocol from qdomyos-zwift cycleopsphantombike.cpp.

Usage:
    python set_h2_resistance.py headless                  # Free spin / Fluid2 curve (RELEASES brake)
    python set_h2_resistance.py erg 100                    # ERG mode, 100 W target (NB: 0W = max brake)
    python set_h2_resistance.py sim 80 0                   # SIM, 80 kg rider, 0% grade
    python set_h2_resistance.py sim 80 5                   # SIM, 80 kg rider, 5% grade
    python set_h2_resistance.py sim 80 -3                  # SIM, 80 kg, -3% (descent)

The H2 needs the flywheel to be moving for BLE advertising. Wake first
with `python wake_flywheel.py` if needed.
"""
import argparse
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from bleak import BleakClient, BleakScanner
from saris_h2.protocol import DEFAULT_ADDR, SARIS_RESISTANCE_UUID
from saris_h2.control import (
    enable_resistance_notifications,
    release_brake,
    set_erg_power,
    set_sim,
)


async def connect():
    print(f"Connecting to H2 at {DEFAULT_ADDR}...")
    try:
        client = BleakClient(DEFAULT_ADDR, timeout=15.0)
        await client.connect()
    except Exception as e:
        print(f"Direct connect failed ({e}); scanning...")
        device = await BleakScanner.find_device_by_address(DEFAULT_ADDR, timeout=20.0)
        if not device:
            print("Trainer not found — make sure the flywheel is moving to wake BLE.")
            sys.exit(1)
        client = BleakClient(device, timeout=15.0)
        await client.connect()
    print("Connected.")
    return client


async def main_headless():
    client = await connect()
    try:
        await enable_resistance_notifications(client)
        await release_brake(client)
        await asyncio.sleep(0.5)
    finally:
        await client.disconnect()
        print("Disconnected.")


async def main_erg(watts: int):
    client = await connect()
    try:
        await enable_resistance_notifications(client)
        await set_erg_power(client, watts)
        await asyncio.sleep(0.5)
    finally:
        await client.disconnect()
        print("Disconnected.")


async def main_sim(weight: float, grade: float):
    client = await connect()
    try:
        await enable_resistance_notifications(client)
        await set_sim(client, weight, grade)
        await asyncio.sleep(0.5)
    finally:
        await client.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Set Saris H2 resistance")
    sub = p.add_subparsers(dest="mode", required=True)
    sub.add_parser("headless", help="Release brake (free spin / Fluid2 curve)")
    erg = sub.add_parser("erg", help="ERG mode (target watts; 0 W ENGAGES MAX BRAKE)")
    erg.add_argument("watts", type=int)
    sim = sub.add_parser("sim", help="SIM mode: weight kg + grade %")
    sim.add_argument("weight", type=float, help="Rider weight (kg)")
    sim.add_argument("grade", type=float, help="Grade percent (e.g. 5.0 or -3.5)")
    args = p.parse_args()

    if args.mode == "headless":
        asyncio.run(main_headless())
    elif args.mode == "erg":
        asyncio.run(main_erg(args.watts))
    elif args.mode == "sim":
        asyncio.run(main_sim(args.weight, args.grade))
