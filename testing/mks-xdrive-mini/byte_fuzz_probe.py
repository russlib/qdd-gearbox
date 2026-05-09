"""Byte-level fuzz probe for the Saris H2 proprietary control char.

The reverse-engineered protocol surface has been mapped: 6 modes (0x00-0x05),
2 signed int16 params, 3 reserved bytes [7..9]. But:

  - No tool sends NEGATIVE param1 (qdomyos clamps to 0, Swift declares Int16
    but never tries negatives).
  - Mode bytes outside 0x00..0x05 are unexplored.
  - Byte [1] is always 0x10; other values may map to different command IDs.
  - Reserved bytes [7..9] always 0x00; flags may exist there.

This probe sends each candidate sequence, waits for and logs responses, and
reports anything novel (new cmdId, new status code, error code).

Brake is engaged the moment we subscribe — that's accepted overhead. We're
looking for response patterns that suggest a hidden mode or privileged path.

Spin the flywheel by hand to wake the trainer first.
"""
from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dyno" / "ble-capture"))

from saris_h2.protocol import DEFAULT_ADDR
from saris_h2.ble_threaded import SarisH2Session


# (label, raw 10-byte hex) — keep all 10 bytes explicit since we're probing
# bytes other tools never touch
SEQUENCES = [
    # --- baseline known-good for reference responses ---
    ("KNOWN_HEADLESS",        "00100000000000000000"),
    ("KNOWN_ERG_100W",        "00100164000000000000"),

    # --- untried region 1: negative ERG (qdomyos clamps to 0; wire is signed) ---
    ("ERG_NEG_1W",            "001001ffff0000000000"),
    ("ERG_NEG_100W",          "0010019cff0000000000"),
    ("ERG_NEG_1000W",         "00100118fc0000000000"),
    ("ERG_NEG_32768W",        "00100100800000000000"),

    # --- untried region 2: negative SLOPE percentages we haven't tried ---
    ("SLOPE_NEG_100PCT",      "001002102710fc000000"),  # weight 100kg, grade -1000 (-100%)
    ("SLOPE_NEG_999PCT",      "001002102711cc000000"),  # not actually -99.9% but exercise

    # --- untried region 3: out-of-enum mode bytes ---
    ("MODE_06",               "00100600000000000000"),
    ("MODE_07",               "00100700000000000000"),
    ("MODE_FF",               "0010ff00000000000000"),
    ("MODE_80",               "00108000000000000000"),
    ("MODE_AA",               "0010aa00000000000000"),
    ("MODE_55",               "00105500000000000000"),

    # --- untried region 4: different command-id second byte [1] ---
    ("CMDB1_11_ERG_100W",     "00110164000000000000"),
    ("CMDB1_20_ERG_100W",     "00200164000000000000"),
    ("CMDB1_FF_ERG_100W",     "00ff0164000000000000"),
    ("CMDB1_00_ERG_100W",     "00000164000000000000"),

    # --- untried region 5: reserved bytes [7..9] flags ---
    ("HEADLESS_FLAG7",        "00100000000000010000"),
    ("HEADLESS_FLAG8",        "00100000000000000100"),
    ("HEADLESS_FLAG9",        "00100000000000000001"),
    ("HEADLESS_ALL_FLAGS",    "00100000000000ffffff"),

    # --- untried region 6: byte [0] non-zero ---
    ("PFX_01_ERG_100W",       "01100164000000000000"),
    ("PFX_FF_ERG_100W",       "ff100164000000000000"),
]


def normalize_seq(label: str, hex_str: str) -> tuple[str, bytes] | None:
    """Validate hex string is exactly 10 bytes (20 hex chars)."""
    hex_str = hex_str.strip().lower()
    if len(hex_str) != 20:
        print(f"  [skip] {label}: hex length {len(hex_str)} != 20")
        return None
    try:
        b = bytes.fromhex(hex_str)
    except ValueError as e:
        print(f"  [skip] {label}: invalid hex ({e})")
        return None
    return (label, b)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--gap", type=float, default=3.5,
                   help="seconds between writes (Swift driver requires >=3.0s)")
    p.add_argument("--final-watch", type=float, default=10.0,
                   help="seconds to keep listening after final write")
    p.add_argument("--only", default="",
                   help="comma-separated subset of labels to send")
    args = p.parse_args()

    seqs = []
    for label, hex_str in SEQUENCES:
        norm = normalize_seq(label, hex_str)
        if norm: seqs.append(norm)

    if args.only:
        wanted = {s.strip() for s in args.only.split(",") if s.strip()}
        seqs = [(l, b) for (l, b) in seqs if l in wanted]
        if not seqs:
            print(f"!! no labels matched --only filter")
            return

    print(f"========== BYTE FUZZ PROBE ==========")
    print(f"  sequences: {len(seqs)}")
    print(f"  gap: {args.gap}s   final_watch: {args.final_watch}s")
    print(f"  total runtime: ~{len(seqs) * args.gap + args.final_watch:.0f}s")

    sess = SarisH2Session(args.addr, subscribe_proprietary=True)
    print("[BLE] starting session...")
    sess.start()
    print("[BLE] connected.")

    # Track baseline known response patterns
    seen_cmdids: set[int] = set()
    seen_status_bytes: set[int] = set()
    seen_modes_in_heartbeat: set[int] = set()
    novel_responses: list[tuple[str, str]] = []   # (label, hex)

    def snapshot_resp_hexes() -> list[str]:
        return [info.split("hex=", 1)[-1]
                for _, kind, info in sess.events
                if kind == "resp"]

    seen_count_before = 0

    for label, cmd in seqs:
        print(f"\n  --- {label}  hex={cmd.hex()}  ---")
        ok = sess.write_proprietary(label, cmd, timeout_s=3.0)
        print(f"    write_ok={ok}")
        time.sleep(args.gap)

        # Check what new responses arrived
        all_resp = snapshot_resp_hexes()
        new = all_resp[seen_count_before:]
        seen_count_before = len(all_resp)
        print(f"    responses since last write: {len(new)}")
        for r in new:
            print(f"      {r}")
            try:
                raw = bytes.fromhex(r)
            except ValueError:
                continue
            if len(raw) >= 4:
                cid = int.from_bytes(raw[2:4], "little")
                if cid not in seen_cmdids:
                    print(f"        *** new cmdId 0x{cid:04X} ***")
                    novel_responses.append((label, r))
                    seen_cmdids.add(cid)
            if len(raw) >= 5:
                m = raw[4]
                if cid == 0x1005 and m not in seen_modes_in_heartbeat:
                    print(f"        *** heartbeat new mode byte 0x{m:02X} ***")
                    seen_modes_in_heartbeat.add(m)
            if len(raw) >= 10:
                s = raw[9]
                if s not in seen_status_bytes:
                    seen_status_bytes.add(s)
                    if s not in (0x00, 0x01, 0x03, 0x04, 0x05, 0x06):
                        print(f"        *** new status byte 0x{s:02X} ***")
                        novel_responses.append((label, r))

    print(f"\n  --- final watch {args.final_watch}s ---")
    time.sleep(args.final_watch)
    final_resp = snapshot_resp_hexes()
    new_final = final_resp[seen_count_before:]
    print(f"    final responses: {len(new_final)}")
    for r in new_final:
        print(f"      {r}")

    sess.stop()

    print(f"\n========== SUMMARY ==========")
    print(f"  total sequences sent: {len(seqs)}")
    counts = sess.event_count_by_kind()
    print(f"  ble event counts: {counts}")
    print(f"  unique cmdIds seen   : {sorted(f'0x{c:04X}' for c in seen_cmdids)}")
    print(f"  unique status bytes  : {sorted(f'0x{s:02X}' for s in seen_status_bytes)}")
    print(f"  hb mode bytes seen   : {sorted(f'0x{m:02X}' for m in seen_modes_in_heartbeat)}")

    if novel_responses:
        print(f"\n  NOVEL responses ({len(novel_responses)}):")
        for label, r in novel_responses:
            print(f"    [{label}] {r}")
    else:
        print(f"\n  No novel responses. Trainer treats all sequences as known modes.")


if __name__ == "__main__":
    main()
