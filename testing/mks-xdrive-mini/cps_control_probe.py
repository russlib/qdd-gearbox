"""Probe the standard CPS Control Point (0x2A66) on the H2.

We've been writing to the proprietary CycleOps char (0xCA31A533...) which is
firmware-clamped to apply baseline brake regardless of mode. But the H2 also
exposes the standard Cycling Power Service Control Point (0x2A66) with
indicate+write properties — and we've NEVER written to it.

Per Bluetooth CPS spec, this characteristic accepts opcodes for things like
Set Cumulative Value, Request Calibration, Start Offset Compensation, etc.
Saris's official Utility app likely uses 0x0C (Start Offset Compensation) or
0x10 (Vendor-Specific Start Offset Compensation) for the spindown procedure
that resets the rolling-resistance offset — which we suspect is the floor
that's preventing brake-off.

This probe sends each documented opcode and logs responses. Just BLE — no
motor.

CPS Control Point requires INDICATIONS (0x0200) on its CCCD, not notifications.
Bleak's start_notify auto-picks based on characteristic properties.

Spin the flywheel by hand to wake the trainer first.
"""
from __future__ import annotations
import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from bleak import BleakClient, BleakScanner
from saris_h2.protocol import DEFAULT_ADDR, CPS_MEASUREMENT_UUID

CPS_CONTROL_POINT_UUID = "00002a66-0000-1000-8000-00805f9b34fb"
CPS_FEATURE_UUID       = "00002a65-0000-1000-8000-00805f9b34fb"
SENSOR_LOCATION_UUID   = "00002a5d-0000-1000-8000-00805f9b34fb"


# Standard CPS Control Point opcodes per BT Spec
CPS_OPCODES = [
    ("SET_CUMULATIVE_VAL_0",        bytes([0x01, 0x00, 0x00, 0x00, 0x00])),
    ("UPDATE_SENSOR_LOCATION",      bytes([0x02, 0x00])),  # location 0=other
    ("REQ_SUPPORTED_LOCATIONS",     bytes([0x03])),
    ("SET_CRANK_LENGTH",            bytes([0x04, 0x6c, 0x07])),  # 1900 (0.5mm units = 95mm)
    ("REQ_CRANK_LENGTH",            bytes([0x05])),
    ("SET_CHAIN_LENGTH",            bytes([0x06, 0xff, 0x00])),
    ("REQ_CHAIN_LENGTH",            bytes([0x07])),
    ("SET_CHAIN_WEIGHT",            bytes([0x08, 0xff, 0x00])),
    ("REQ_CHAIN_WEIGHT",            bytes([0x09])),
    ("SET_SPAN_LENGTH",             bytes([0x0a, 0xff, 0x00])),
    ("REQ_SPAN_LENGTH",             bytes([0x0b])),
    ("START_OFFSET_COMPENSATION",   bytes([0x0c])),  # standard spindown trigger
    ("MASK_CP_CONTENT",             bytes([0x0d, 0x00, 0x00])),
    ("REQ_SAMPLING_RATE",           bytes([0x0e])),
    ("REQ_FACTORY_CAL_DATE",        bytes([0x0f])),
    ("START_OFFSET_COMP_VENDOR",    bytes([0x10])),  # vendor-specific spindown — most interesting
    ("REQ_CRC",                     bytes([0x11])),
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

    # Read CPS Feature & Sensor Location for context
    try:
        feat = await client.read_gatt_char(CPS_FEATURE_UUID)
        print(f"  CPS Feature (0x2A65): {feat.hex()}  ({len(feat)} bytes)")
        # Bit 9 = "Offset Compensation Indicator Supported"
        # Bit 12 = "Offset Compensation Supported"
        if len(feat) >= 4:
            f32 = int.from_bytes(feat[:4], "little")
            print(f"    flags as uint32 LE: 0x{f32:08X}")
            for bit, desc in [
                (0,  "Pedal Power Balance Supported"),
                (1,  "Accumulated Torque Supported"),
                (2,  "Wheel Revolution Data Supported"),
                (3,  "Crank Revolution Data Supported"),
                (4,  "Extreme Magnitudes Supported"),
                (5,  "Extreme Angles Supported"),
                (6,  "Top/Bottom Dead Spot Angles Supported"),
                (7,  "Accumulated Energy Supported"),
                (8,  "Offset Compensation Indicator Supported"),
                (9,  "Offset Compensation Supported"),  # YES means trainer accepts opcode 0x0C
                (10, "CP Measurement Char Content Masking Supported"),
                (11, "Multiple Sensor Locations Supported"),
                (12, "Crank Length Adj Supported"),
                (13, "Chain Length Adj Supported"),
                (14, "Chain Weight Adj Supported"),
                (15, "Span Length Adj Supported"),
                (16, "Sensor Measurement Context"),
                (17, "Instantaneous Measurement Direction Supported"),
                (18, "Factory Calibration Date Supported"),  # YES = opcode 0x0F supported
                (19, "Enhanced Offset Compensation Supported"),  # = vendor-specific
            ]:
                if f32 & (1 << bit):
                    print(f"      bit {bit:2d}  {desc}")
    except Exception as e:
        print(f"  CPS Feature read failed: {e}")

    try:
        loc = await client.read_gatt_char(SENSOR_LOCATION_UUID)
        print(f"  Sensor Location (0x2A5D): {loc.hex()}  ({loc[0] if len(loc) else '?'})")
    except Exception as e:
        print(f"  Sensor Location read failed: {e}")

    # Subscribe to indications on Control Point and notifications on Measurement
    responses: list[tuple[float, str]] = []
    cps_count = [0]
    t0 = time.time()

    def on_cp(_, data: bytearray):
        t = time.time() - t0
        responses.append((t, bytes(data).hex()))
        print(f"    [t={t:5.2f}s] CP-CTRL ind: {bytes(data).hex()}")

    def on_meas(_, data: bytearray):
        cps_count[0] += 1

    await client.start_notify(CPS_MEASUREMENT_UUID, on_meas)
    await client.start_notify(CPS_CONTROL_POINT_UUID, on_cp)
    print(f"\n  Subscribed to CPS measurement + control point.")

    novel: list[tuple[str, str]] = []

    for label, payload in CPS_OPCODES:
        print(f"\n  --- {label}  hex={payload.hex()}  ---")
        before = len(responses)
        try:
            # Try with response first (CP CCCDs typically need indication-acked writes)
            await client.write_gatt_char(CPS_CONTROL_POINT_UUID, payload, response=True)
            print(f"    write returned (with-response).")
        except Exception as e:
            print(f"    write w/response failed: {e}; trying w/o response")
            try:
                await client.write_gatt_char(CPS_CONTROL_POINT_UUID, payload, response=False)
                print(f"    write returned (without-response).")
            except Exception as e2:
                print(f"    write w/o response also failed: {e2}")

        # Wait for indication
        await asyncio.sleep(gap_s)
        after = len(responses)
        new = responses[before:after]
        if not new:
            print(f"    -> no indication response (probably not supported)")
        for t, h in new:
            print(f"    -> response @ t={t:.2f}s: {h}")
            # CPS response format: 0x20 <op_request> <result_code> [params...]
            try:
                raw = bytes.fromhex(h)
                if len(raw) >= 3:
                    if raw[0] == 0x20:
                        result_code = raw[2]
                        result_names = {
                            0x01: "Success",
                            0x02: "Op Code not supported",
                            0x03: "Invalid Parameter",
                            0x04: "Operation Failed",
                        }
                        print(f"       parsed: opcode_response_for=0x{raw[1]:02X} "
                              f"result=0x{raw[2]:02X} ({result_names.get(result_code, '?')})")
                        if result_code == 0x01:
                            novel.append((label, h))
                            print(f"       ** {label} SUCCEEDED **")
            except Exception as e:
                print(f"       parse err: {e}")

    print(f"\n  Final wait 5s for stragglers...")
    await asyncio.sleep(5.0)

    try: await client.stop_notify(CPS_MEASUREMENT_UUID)
    except Exception: pass
    try: await client.stop_notify(CPS_CONTROL_POINT_UUID)
    except Exception: pass
    await client.disconnect()

    print(f"\n=== SUMMARY ===")
    print(f"  total opcodes sent      : {len(CPS_OPCODES)}")
    print(f"  total CP indications    : {len(responses)}")
    print(f"  total CPS notifications : {cps_count[0]}")
    if novel:
        print(f"\n  *** SUPPORTED OPCODES ***")
        for label, h in novel:
            print(f"    {label}: response {h}")
    else:
        print(f"\n  No opcodes returned Success. CP may be unimplemented on this firmware.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--gap", type=float, default=3.0)
    args = p.parse_args()
    asyncio.run(main(args.addr, args.gap))
