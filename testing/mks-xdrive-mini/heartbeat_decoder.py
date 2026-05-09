"""Passive heartbeat listener for the Saris H2 proprietary control char.

Subscribing to the proprietary char engages the brake — known. But once we're
subscribed, the trainer streams ~1 Hz status frames (we observed cmdId 0x1005).
Decoding them gives us direct ground-truth on the trainer's *current* mode and
brake status, instead of inferring from motor RPM.

Reference: Kinetic-published Swift driver `CycleOpsSerializer.readResponse()`.
Layout (best-effort, may need adjustment after seeing real data):

  byte 0..1 : header  (likely 0x00 0x10 echoing the request header)
  byte 2..3 : cmdId   little-endian (0x1000 = ack, 0x1005 = unsolicited status)
  byte 4    : current ControlMode (0x00=Headless, 0x01=ManualPower, ...)
  byte 5..6 : param1
  byte 7..8 : param2
  byte 9    : status code (per Swift ControlStatus: 0x01=SpeedUp,
              0x03=RollDownInitializing, 0x04=RollDownInProcess,
              0x05=RollDownPassed, 0x06=RollDownFailed, ...)

Run during T7. Subscribe → no writes → listen → log raw + parsed bytes for
~30 s. Manually inspect to confirm field offsets, then iterate.

Usage:
  & $script:MksPython heartbeat_decoder.py --duration 30
"""
from __future__ import annotations
import argparse
import asyncio
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from bleak import BleakClient, BleakScanner
from saris_h2.protocol import (
    DEFAULT_ADDR,
    SARIS_RESISTANCE_UUID,
    CPS_MEASUREMENT_UUID,
    parse_cps,
)

OUT_DIR = Path(__file__).parent / "thermal_data"
OUT_DIR.mkdir(exist_ok=True)
ts = time.strftime("%Y%m%d-%H%M%S")
CSV_PATH = OUT_DIR / f"heartbeat_decode_{ts}.csv"


CONTROL_MODE_NAMES = {
    0x00: "HEADLESS",
    0x01: "MANUAL_POWER",
    0x02: "MANUAL_SLOPE",
    0x03: "POWER_RANGE",
    0x04: "WARM_UP",
    0x05: "ROLL_DOWN",
}

CONTROL_STATUS_NAMES = {
    0x00: "Idle",
    0x01: "SpeedUp",
    0x02: "SpeedUpDone",
    0x03: "RollDownInitializing",
    0x04: "RollDownInProcess",
    0x05: "RollDownPassed",
    0x06: "RollDownFailed",
}


def decode(raw: bytes) -> dict:
    out = {"raw_hex": raw.hex(), "len": len(raw)}
    if len(raw) >= 4:
        out["cmd_id_le"] = f"0x{int.from_bytes(raw[2:4], 'little'):04X}"
    if len(raw) >= 5:
        m = raw[4]
        out["mode_byte"] = f"0x{m:02X}"
        out["mode_name"] = CONTROL_MODE_NAMES.get(m, "?")
    if len(raw) >= 9:
        out["p1"] = int.from_bytes(raw[5:7], "little", signed=True)
        out["p2"] = int.from_bytes(raw[7:9], "little", signed=True)
    if len(raw) >= 10:
        s = raw[9]
        out["status_byte"] = f"0x{s:02X}"
        out["status_name"] = CONTROL_STATUS_NAMES.get(s, "?")
    return out


async def main(addr: str, duration: float):
    csv_file = open(CSV_PATH, "w", newline="")
    fieldnames = ["t_s", "kind", "raw_hex", "len", "cmd_id_le", "mode_byte", "mode_name",
                  "p1", "p2", "status_byte", "status_name", "cps_power_w", "cps_acc_torque"]
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()

    print(f"Connecting to {addr}...")
    try:
        client = BleakClient(addr, timeout=15.0)
        await client.connect()
    except Exception:
        device = await BleakScanner.find_device_by_address(addr, timeout=20.0)
        if not device:
            raise SystemExit("Trainer not found. Spin flywheel + retry.")
        client = BleakClient(device, timeout=15.0)
        await client.connect()
    print("Connected.")

    t0 = time.time()
    n_resp = 0
    n_cps = 0

    def on_resp(_, data: bytearray):
        nonlocal n_resp
        n_resp += 1
        t = time.time() - t0
        d = decode(bytes(data))
        row = {"t_s": f"{t:.3f}", "kind": "resp"}
        row.update(d)
        writer.writerow(row)
        print(f"  [t={t:5.2f}s] RESP  cmd={d.get('cmd_id_le','-'):>8}  "
              f"mode={d.get('mode_name','?'):>13} ({d.get('mode_byte','-')})  "
              f"p1={d.get('p1','-')} p2={d.get('p2','-')}  "
              f"status={d.get('status_name','?'):>20} ({d.get('status_byte','-')})  "
              f"raw={d['raw_hex']}")

    def on_cps(_, data: bytearray):
        nonlocal n_cps
        n_cps += 1
        t = time.time() - t0
        try:
            m = parse_cps(bytes(data))
            writer.writerow({"t_s": f"{t:.3f}", "kind": "cps",
                             "raw_hex": bytes(data).hex(),
                             "cps_power_w": m.power_w,
                             "cps_acc_torque": f"{m.acc_torque_nm:.4f}"})
            if n_cps <= 3 or n_cps % 5 == 0:
                print(f"  [t={t:5.2f}s] CPS   pwr={m.power_w}W  acc_torque={m.acc_torque_nm:.2f} Nm")
        except Exception as e:
            print(f"  CPS parse err: {e}")

    try:
        # Subscribe in this order: CPS first (brake-safe), then proprietary
        # (engages brake). The user has been warned by the test matrix that
        # T7 will engage the brake — we accept that to read the heartbeat.
        await client.start_notify(CPS_MEASUREMENT_UUID, on_cps)
        await client.start_notify(SARIS_RESISTANCE_UUID, on_resp)
        print(f"Subscribed. Listening {duration:.0f}s. (No writes will be sent.)")
        await asyncio.sleep(duration)
    finally:
        try: await client.stop_notify(SARIS_RESISTANCE_UUID)
        except Exception: pass
        try: await client.stop_notify(CPS_MEASUREMENT_UUID)
        except Exception: pass
        await client.disconnect()
        csv_file.close()

    print(f"\n=== Done ===")
    print(f"  responses: {n_resp}   cps: {n_cps}")
    print(f"  CSV: {CSV_PATH}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--duration", type=float, default=30.0)
    args = p.parse_args()
    asyncio.run(main(args.addr, args.duration))
