"""Probe the H2's mystery custom characteristic 0xE005.

The H2 exposes a custom char inside the standard CPS service:
  Service:    0x1818 (Cycling Power Service)
  Char UUID:  0000e005-0000-1000-8000-00805f9b34fb (indicate + write)

We've never written to it. It might be:
  - Saris DFU/firmware-update entry point
  - A privileged service-mode/factory channel
  - A debug/log streaming channel
  - A real "release brake" path

Strategy: subscribe to it, then send a small set of likely-safe probes:
  - Single-byte reads of common opcode ranges
  - Standard "request" patterns (read-style opcodes)
Watch for any indication response.

DOES NOT engage motor. BLE only.
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

E005_UUID = "0000e005-0000-1000-8000-00805f9b34fb"


# Cautious probes — short payloads only, avoid anything that looks like a flash write
PROBES = [
    # Single-byte reads: try common "info" / "status" opcodes
    ("REQ_00",        bytes([0x00])),
    ("REQ_01",        bytes([0x01])),
    ("REQ_02",        bytes([0x02])),
    ("REQ_03",        bytes([0x03])),
    ("REQ_FF",        bytes([0xFF])),
    # Common Saris cmd-id structure (echo from proprietary char):
    ("HDR_0010",      bytes([0x00, 0x10])),
    ("HDR_0020",      bytes([0x00, 0x20])),
    # 4-byte common request frames
    ("REQ_4_00",      bytes([0x00, 0x00, 0x00, 0x00])),
    ("GET_VER",       bytes([0x10, 0x01, 0x00, 0x00])),  # generic "get firmware version"
    ("GET_INFO",      bytes([0x10, 0x02, 0x00, 0x00])),
    # Saris-style 10-byte frame (mirrors proprietary)
    ("FRAME_HEADLESS",bytes([0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
    ("FRAME_QUERY",   bytes([0x00, 0x10, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
]


async def main(addr: str, gap_s: float):
    print(f"Connecting to {addr}...")
    try:
        client = BleakClient(addr, timeout=15.0)
        await client.connect()
    except Exception:
        device = await BleakScanner.find_device_by_address(addr, timeout=15.0)
        if not device:
            raise SystemExit("trainer not found")
        client = BleakClient(device, timeout=15.0)
        await client.connect()
    print("Connected.")

    # Confirm service and char exist — match by suffix "e005" since the full
    # 128-bit UUID may be vendor-specific
    found_uuid = None
    print("\n  Discovering all chars with short-form e005...")
    for s in client.services:
        for c in s.characteristics:
            short = c.uuid.split("-")[0][-4:].lower()
            if short == "e005":
                print(f"    -> service {s.uuid}, char {c.uuid}, props={c.properties}")
                found_uuid = c.uuid
    if not found_uuid:
        print("  !! No char with suffix e005 advertised. Listing all chars for debug:")
        for s in client.services:
            print(f"    service {s.uuid}")
            for c in s.characteristics:
                print(f"      char {c.uuid}  props={c.properties}")
        await client.disconnect()
        return
    print(f"  using full UUID: {found_uuid}")

    # Replace the targeting UUID for writes/notifies
    target_uuid = found_uuid

    responses: list[tuple[float, bytes]] = []
    t0 = time.time()

    def on_ind(_, data: bytearray):
        t = time.time() - t0
        responses.append((t, bytes(data)))
        print(f"    [t={t:5.2f}s] IND: {bytes(data).hex()}")

    try:
        await client.start_notify(target_uuid, on_ind)
        print("  Subscribed to 0xE005 indications.")
    except Exception as e:
        print(f"  !! subscribe failed: {e}")
        await client.disconnect()
        return

    print("\n  --- letting char idle for 3s to see unsolicited traffic ---")
    await asyncio.sleep(3.0)

    novel = []
    for label, payload in PROBES:
        print(f"\n  --- {label}  hex={payload.hex()} ---")
        before = len(responses)
        try:
            await client.write_gatt_char(target_uuid, payload, response=True)
            print(f"    write returned (with-response)")
        except Exception as e:
            print(f"    w/response failed: {e}; trying w/o response")
            try:
                await client.write_gatt_char(target_uuid, payload, response=False)
                print(f"    write returned (without-response)")
            except Exception as e2:
                print(f"    BOTH writes failed: {e2}")
                continue

        await asyncio.sleep(gap_s)
        new = responses[before:]
        if new:
            for t, raw in new:
                print(f"    -> resp @ t={t:.2f}s: {raw.hex()}")
                novel.append((label, raw.hex()))
        else:
            print(f"    -> no response")

    print(f"\n  final wait 3s...")
    await asyncio.sleep(3.0)

    try: await client.stop_notify(target_uuid)
    except Exception: pass
    await client.disconnect()

    print(f"\n=== SUMMARY ===")
    print(f"  total responses: {len(responses)}")
    if novel:
        print(f"  responses by probe:")
        for label, h in novel:
            print(f"    [{label}] {h}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--gap", type=float, default=2.0)
    args = p.parse_args()
    asyncio.run(main(args.addr, args.gap))
