"""Probe Wahoo KICKR control opcodes on the H2's `a026e005` characteristic.

The H2 advertises a Wahoo-derived control point (UUID base `a026...0a7d-4ab3-
97fa-f1500f9feb8b`) inside its CPS service. We've already shown opcodes
0x00-0x10 all return "Op Code not supported" (status 0x02). But Wahoo's KICKR
uses opcodes 0x40+, and has a documented advanced-command unlock at 0x4F with
magic key 0xee 0xfc.

This probe tries the full known Wahoo opcode set, including the unlock, then
the standard control opcodes. Watch for any indication response other than
"02 = not supported."
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

WAHOO_CHAR_UUID = "a026e005-0a7d-4ab3-97fa-f1500f9feb8b"


# Wahoo KICKR control opcodes (publicly reverse-engineered)
PROBES = [
    # Unlock first — many things may require this preface
    ("UNLOCK",                   bytes([0x20, 0xee, 0xfc])),
    ("UNLOCK_alt",               bytes([0x4F, 0xee, 0xfc])),

    # Standard control opcodes
    ("SET_RESISTANCE_0",         bytes([0x40, 0x00, 0x00])),     # 0% resistance
    ("SET_RESISTANCE_min",       bytes([0x40, 0x00, 0x80])),     # min int16 LE
    ("SET_STANDARD_MODE",        bytes([0x41, 0x00, 0x00])),
    ("SET_ERG_0W",               bytes([0x42, 0x00, 0x00])),     # ERG 0W (Wahoo style)
    ("SET_ERG_500W",             bytes([0x42, 0xf4, 0x01])),     # ERG 500W
    ("SET_SIM_MODE",             bytes([0x43, 0x40, 0x1f, 0xa4, 0x00, 0xa4, 0x00])),  # weight 80kg
    ("SET_GRADE_0",              bytes([0x44, 0x00, 0x80])),     # grade 0 (signed mid)
    ("SET_GRADE_neg_max",        bytes([0x44, 0x00, 0x00])),     # grade min
    ("SET_WIND_0",               bytes([0x45, 0x00, 0x80])),
    ("SET_ROLLING_0",            bytes([0x46, 0x00, 0x00])),
    ("SET_WHEEL_DIAM_700",       bytes([0x47, 0xb6, 0x08])),     # 2230 (=2.23m circumf 0.001m units)
    ("INIT_SPINDOWN",            bytes([0x4A])),
    ("READ_MODE",                bytes([0x01])),                 # Wahoo "read mode"
    ("UNKNOWN_4B",               bytes([0x4B])),
    ("UNKNOWN_4C",               bytes([0x4C])),
    ("UNKNOWN_4D",               bytes([0x4D])),
    ("UNKNOWN_4E",               bytes([0x4E])),
]


async def main(addr: str, gap_s: float, do_unlock_first: bool):
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

    responses: list[tuple[float, bytes]] = []
    t0 = time.time()

    def on_ind(_, data: bytearray):
        t = time.time() - t0
        responses.append((t, bytes(data)))
        print(f"    [t={t:5.2f}s] IND: {bytes(data).hex()}")

    await client.start_notify(WAHOO_CHAR_UUID, on_ind)
    print("  Subscribed to Wahoo char.")

    # Always send unlock first if requested
    if do_unlock_first:
        for unlock_label, unlock_payload in [
            ("UNLOCK_4F",   bytes([0x4F, 0xee, 0xfc])),
            ("UNLOCK_20",   bytes([0x20, 0xee, 0xfc])),
        ]:
            print(f"\n  --- {unlock_label}  hex={unlock_payload.hex()} ---")
            try:
                await client.write_gatt_char(WAHOO_CHAR_UUID, unlock_payload, response=True)
                print(f"    write returned")
            except Exception as e:
                print(f"    write failed: {e}")
            await asyncio.sleep(gap_s)

    novel = []
    for label, payload in PROBES:
        if label.startswith("UNLOCK") and do_unlock_first:
            continue  # already sent
        print(f"\n  --- {label}  hex={payload.hex()} ---")
        before = len(responses)
        try:
            await client.write_gatt_char(WAHOO_CHAR_UUID, payload, response=True)
            print(f"    write returned")
        except Exception as e:
            print(f"    w/response failed: {e}; trying w/o")
            try:
                await client.write_gatt_char(WAHOO_CHAR_UUID, payload, response=False)
                print(f"    w/o response returned")
            except Exception as e2:
                print(f"    BOTH failed: {e2}")
                continue

        await asyncio.sleep(gap_s)
        new = responses[before:]
        for t, raw in new:
            print(f"    -> resp @ t={t:.2f}s: {raw.hex()}")
            # Wahoo response format: [01] [op] [status] [...]
            # status 0x00 typically = success
            if len(raw) >= 3 and raw[2] != 0x02:  # not "not supported"
                print(f"       *** non-NOT-SUPPORTED response! status=0x{raw[2]:02X} ***")
                novel.append((label, raw.hex()))

    print(f"\n  final wait 3s...")
    await asyncio.sleep(3.0)

    try: await client.stop_notify(WAHOO_CHAR_UUID)
    except Exception: pass
    await client.disconnect()

    print(f"\n=== SUMMARY ===")
    print(f"  total responses: {len(responses)}")
    print(f"  novel (non-not-supported): {len(novel)}")
    for label, h in novel:
        print(f"    [{label}] {h}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--gap", type=float, default=2.0)
    p.add_argument("--no-unlock", action="store_true",
                   help="Skip the initial unlock writes (default sends unlock first)")
    args = p.parse_args()
    asyncio.run(main(args.addr, args.gap, do_unlock_first=not args.no_unlock))
