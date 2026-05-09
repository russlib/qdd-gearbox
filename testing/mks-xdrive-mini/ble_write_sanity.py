"""BLE-only sanity: prove writes fire and trainer responds.

Connects, subscribes, sends a sequence of mode writes (HEADLESS, WARM_UP,
ROLL_DOWN), waits 3 s between each, then reports event counts and dumps
the timeline. No motor.

Use this between rebuilds of the BLE side to confirm the channel works
before burning thermal cycles on the motor.
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


SEQUENCE = [
    ("HEADLESS",   ResistanceMode.HEADLESS,   0,    0),
    ("WARM_UP",    ResistanceMode.WARM_UP,    0,    0),
    ("ROLL_DOWN",  ResistanceMode.ROLL_DOWN,  0,    0),
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--gap", type=float, default=3.0,
                   help="seconds between writes (and after last write before disconnect)")
    args = p.parse_args()

    sess = SarisH2Session(args.addr, subscribe_proprietary=True)
    print("[main] starting session...")
    sess.start()
    print("[main] session ready.")

    for label, mode, p1, p2 in SEQUENCE:
        cmd = build_resistance_cmd(mode, p1, p2)
        print(f"[main] writing {label}  hex={cmd.hex()}")
        ok = sess.write_proprietary(label, cmd, timeout_s=3.0)
        print(f"[main]   write_ok={ok}")
        time.sleep(args.gap)

    print(f"[main] gap done, stopping session...")
    sess.stop()

    print(f"\n=== Summary ===")
    counts = sess.event_count_by_kind()
    for k in ("connected", "subscribed_cps", "subscribed_proprietary",
              "write_submit", "write_ok", "write_timeout", "write_err",
              "resp", "cps", "cps_err", "disconnected"):
        if k in counts:
            print(f"  {k:30s}: {counts[k]}")
    print(f"  total events: {len(sess.events)}")
    print(f"  cps count   : {sess.cps_count}")
    print(f"  last_power  : {sess.last_power}")

    print(f"\n=== Timeline (write/resp only) ===")
    for t, kind, info in sess.events:
        if kind in ("write_submit", "write_ok", "write_timeout", "write_err", "resp"):
            print(f"  [t={t:6.2f}s] {kind:14s}  {info}")


if __name__ == "__main__":
    main()
