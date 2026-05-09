"""Backfill `_meta.json` sidecars for tests run on 2026-05-07 / 2026-05-08.

Run once. Writes alongside each existing CSV in thermal_data/.
"""
from __future__ import annotations
import json
from pathlib import Path

OUT_DIR = Path(__file__).parent / "thermal_data"

HW = {
    "motor": "D6374-150KV BLDC",
    "controller": "MKS XDrive Mini (ODrive 0.5.1 fw)",
    "encoder": "AMT-102 ABI (no index wired)",
    "trainer": "Saris Hammer H2 (CycleOps H2)",
}

# Common conditions for the 2026-05-07 and 2026-05-08 sessions
COND_BASE = {
    "esc_fan": True,        # default; overridden where it changed
    "motor_fan": True,
    "trainer_ac": "on",
    "ambient_c": 25,
    "bus_v": 34.0,
    "current_lim_a": 60,
    "direction": 1,
}

# Per-test metadata. Key = CSV stem (without .csv).
TESTS = {
    # ------ 2026-05-06 brake-trigger probes (older session, kept for completeness) ------
    "brake_probe_trialA_none_20260506-110648": {
        "purpose": "Trial A — connect-only baseline (-1 dir, motor unloaded by freewheel)",
        "conditions": {**COND_BASE, "direction": -1, "trainer_ble_state": "no client"},
        "parameters": {"target_iq_a": 30, "hold_s": 25},
        "result": {"peak_rpm": 824, "verdict": "free spin, freewheel disengaged"},
    },
    "brake_probe_trialA_none_20260506-110914": {
        "purpose": "Trial A retry (-1 dir freewheel-disengaged baseline)",
        "conditions": {**COND_BASE, "direction": -1, "trainer_ble_state": "no client"},
        "parameters": {"target_iq_a": 30, "hold_s": 25},
        "result": {"peak_rpm": 1244, "verdict": "free spin, freewheel disengaged"},
    },
    "brake_probe_trialA_none_20260506-111007": {
        "purpose": "Trial A first +1 direction test, 30A 1.2Nm — wheel barely moves (16 RPM peak)",
        "conditions": {**COND_BASE, "direction": 1, "trainer_ble_state": "no client"},
        "parameters": {"target_iq_a": 30, "hold_s": 25},
        "result": {"peak_rpm": 16.5, "verdict": "brake locked, freewheel engaged in +1"},
    },
    "brake_probe_trialB_connect_20260506-110737": {
        "purpose": "Trial B — BLE connected, no subscriptions, no writes (-1 dir)",
        "conditions": {**COND_BASE, "direction": -1, "trainer_ble_state": "connected only"},
        "parameters": {"target_iq_a": 15, "hold_s": 15},
        "result": {"peak_rpm": 1244, "verdict": "free spin (freewheel disengaged in -1)"},
    },
    "brake_probe_trialC_cps_20260506-110811": {
        "purpose": "Trial C — BLE + CPS measurement subscribe (-1 dir)",
        "conditions": {**COND_BASE, "direction": -1, "trainer_ble_state": "CPS subscribed"},
        "parameters": {"target_iq_a": 15, "hold_s": 15},
        "result": {"peak_rpm": 1236, "verdict": "free spin (CPS subscribe brake-safe)"},
    },

    # ------ 2026-05-07 brake-disable matrix (all FAIL, +1 direction) ------
    "brake_disable_none_20260507-175721": {
        "purpose": "T0 — fresh-boot baseline, no BLE, +1 direction with brake on",
        "conditions": {**COND_BASE, "trainer_ble_state": "no client"},
        "parameters": {"target_iq_a": 30, "hold_s": 15},
        "result": {"peak_rpm": 33, "verdict": "FAIL (braked baseline)"},
    },
    "brake_disable_warmup_20260507-180209": {
        "purpose": "T1 — write WARM_UP (0x04) on proprietary char, +1 30A",
        "conditions": {**COND_BASE, "trainer_ble_state": "proprietary subscribed + WARM_UP write"},
        "parameters": {"target_iq_a": 30, "hold_s": 15},
        "result": {"peak_rpm": 24.7, "verdict": "FAIL — heartbeat shows trainer in MANUAL_POWER 0W"},
    },
    "brake_disable_headless_20260507-180308": {
        "purpose": "T3 — write HEADLESS (0x00), +1 30A",
        "conditions": {**COND_BASE, "trainer_ble_state": "proprietary subscribed + HEADLESS write"},
        "parameters": {"target_iq_a": 30, "hold_s": 15},
        "result": {"peak_rpm": 33, "verdict": "FAIL — ack only, no brake change"},
    },
    "brake_disable_power-range-0_20260507-180350": {
        "purpose": "T4 — write POWER_RANGE 0,0 (0x03), +1 30A",
        "conditions": {**COND_BASE, "trainer_ble_state": "proprietary subscribed + POWER_RANGE write"},
        "parameters": {"target_iq_a": 30, "hold_s": 15},
        "result": {"peak_rpm": 24.7, "verdict": "FAIL"},
    },
    "brake_disable_rolldown_mr_20260507-180432": {
        "purpose": "T2 — write ROLL_DOWN (0x05), multi-ramp motor, +1 30A",
        "conditions": {**COND_BASE, "trainer_ble_state": "proprietary subscribed + ROLL_DOWN write"},
        "parameters": {"target_iq_a": 30, "hold_s": 30, "multi_ramp": True},
        "result": {"peak_rpm": 24.7, "verdict": "FAIL — status 0x03 RollDownInitializing, no brake change"},
    },
    "brake_disable_warmup_disc_20260507-180607": {
        "purpose": "T5 — WARM_UP then GATT disconnect before motor spin",
        "conditions": {**COND_BASE, "trainer_ble_state": "wrote then disconnected"},
        "parameters": {"target_iq_a": 30, "hold_s": 15},
        "result": {"peak_rpm": 24.7, "verdict": "FAIL — disconnect didn't break the lock"},
    },
    "brake_disable_sim-flat_20260507-180747": {
        "purpose": "SIM_FLAT — MANUAL_SLOPE grade 0%, +1 30A",
        "conditions": {**COND_BASE, "trainer_ble_state": "BLE may have dropped"},
        "parameters": {"target_iq_a": 30, "hold_s": 15},
        "result": {"peak_rpm": 24.7, "verdict": "FAIL"},
    },
    "rapid_brake_cycle_20260507-183055": {
        "purpose": "Rapid cycle through 9 modes at 45A — looking for brake release at speed",
        "conditions": {**COND_BASE, "trainer_ble_state": "queue-write architecture (broken; 0 events fired)"},
        "parameters": {"target_iq_a": 45, "settle_s": 6, "per_cmd_s": 3},
        "result": {"peak_rpm": 49.4, "verdict": "FAIL — also revealed BLE write queue bug fixed in ble_threaded.py"},
    },
    "brake_disable_none_20260507-185535": {
        "purpose": "Post-rolldown 'is brake off after disconnect?' check, no BLE",
        "conditions": {**COND_BASE, "trainer_ble_state": "no client (after ROLL_DOWN+disc earlier)"},
        "parameters": {"target_iq_a": 45, "hold_s": 30},
        "result": {"peak_rpm": 41.2, "verdict": "FAIL — trainer reverts to MANUAL_POWER 0W on disconnect"},
    },
    "rolldown_then_spin_20260507-185818": {
        "purpose": "ROLL_DOWN then spin (v1: only ROLL_DOWN as pre-seq)",
        "conditions": {**COND_BASE, "trainer_ble_state": "ROLL_DOWN, ROLL_DOWN keepalive"},
        "parameters": {"target_iq_a": 45, "hold_s": 25, "keepalive": "ROLL_DOWN"},
        "result": {"peak_rpm": 41.2, "verdict": "FAIL — RollDownFailed (cmdId 0x1009 status 0x06), no HEADLESS transition"},
    },
    "rolldown_then_spin_20260507-190017": {
        "purpose": "ROLL_DOWN with HEADLESS+WARM_UP pre-seq + HEADLESS keepalive — heartbeat CONFIRMED HEADLESS at t=6.45s",
        "conditions": {**COND_BASE, "trainer_ble_state": "HEADLESS confirmed via heartbeat, brake still on"},
        "parameters": {"target_iq_a": 45, "hold_s": 15, "keepalive": "HEADLESS"},
        "result": {"peak_rpm": 41.2, "verdict": "DEFINITIVE FAIL — HEADLESS = Fluid2 curve, NOT free spin"},
    },
    "brake_disable_none_20260507-190857": {
        "purpose": "Trainer AC UNPLUGGED — testing if eddy brake actually needs power",
        "conditions": {**COND_BASE, "trainer_ac": "UNPLUGGED", "trainer_ble_state": "no BLE possible"},
        "parameters": {"target_iq_a": 30, "hold_s": 30},
        "result": {"peak_rpm": 41.2, "verdict": "FAIL — brake still on with AC removed (likely permanent-magnet bias)"},
    },
    "brake_disable_mode-07_20260507-194828": {
        "purpose": "Test undocumented MODE 0x07 (found via byte fuzzer)",
        "conditions": {**COND_BASE, "trainer_ble_state": "wrote 00 10 07 ..."},
        "parameters": {"target_iq_a": 30, "hold_s": 20},
        "result": {"peak_rpm": 24.7, "verdict": "FAIL — 0x07 acked but inert"},
    },
    "brake_disable_slope-down-1000_20260507-194923": {
        "purpose": "MANUAL_SLOPE with grade -100% (param2 = -1000)",
        "conditions": {**COND_BASE, "trainer_ble_state": "SIM grade -100%"},
        "parameters": {"target_iq_a": 30, "hold_s": 20},
        "result": {"peak_rpm": 33, "verdict": "FAIL — accepted but firmware clamps internally"},
    },
    "offset_comp_then_spin_20260507-200935": {
        "purpose": "Standard CPS opcode 0x0C (Start Offset Compensation) on char 0x2A66",
        "conditions": {**COND_BASE, "trainer_ble_state": "CPS Control Point 0x0C accepted"},
        "parameters": {"target_iq_a": 30, "hold_s": 25},
        "result": {"peak_rpm": 41.2, "verdict": "FAIL — response 20 0c 01 ff ff (Success) but no brake change"},
    },
    "wahoo_erg_0w_then_spin_20260507-201604": {
        "purpose": "Wahoo opcode 0x42 (Set ERG, 0W) on a026e005 — Wahoo-derived char",
        "conditions": {**COND_BASE, "trainer_ble_state": "Wahoo ERG 0W accepted (status 0x01)"},
        "parameters": {"target_iq_a": 30, "hold_s": 25},
        "result": {"peak_rpm": 33, "verdict": "FAIL — Wahoo opcodes accepted but inert on this firmware"},
    },

    # ------ 2026-05-08 continuous-current characterization ------
    "adaptive_iq_at_temp_20260508-144502": {
        "purpose": "Adaptive search for max continuous IQ at 80°C FET, ESC FAN ON, motor fan on",
        "conditions": {**COND_BASE, "esc_fan": True, "motor_fan": True,
                       "trainer_ble_state": "no client", "current_lim_a": 65},
        "parameters": {"target_temp_c": 78, "temp_high_c": 80, "temp_abort_c": 85,
                       "iq_start_a": 30, "up_rate_a_s": 0.3, "dn_rate_a_s": 2.0,
                       "duration_s": 480},
        "result": {"continuous_iq_a": 45.8, "continuous_torque_nm": 1.84,
                   "steady_fet_c": 79.5, "convergence_quality": "incomplete (drifting)",
                   "abort_reason": "ax=2 ERROR_DC_BUS_UNDER_VOLTAGE (user cut supply)"},
    },
    "adaptive_iq_at_temp_20260508-183838": {
        "purpose": "Adaptive search, ESC FAN OFF (motor fan still on)",
        "conditions": {**COND_BASE, "esc_fan": False, "motor_fan": True,
                       "trainer_ble_state": "no client", "current_lim_a": 65},
        "parameters": {"target_temp_c": 78, "temp_high_c": 80, "temp_abort_c": 85,
                       "iq_start_a": 20, "up_rate_a_s": 0.15, "dn_rate_a_s": 2.0,
                       "duration_s": 600},
        "result": {"continuous_iq_a": 25.2, "continuous_torque_nm": 1.01,
                   "steady_fet_c": 79.1, "convergence_quality": "fully converged (std=0.00 over last 60s)",
                   "abort_reason": "completed normally"},
    },
}


def main():
    written = 0
    skipped_missing = 0
    for stem, meta in TESTS.items():
        csv_path = OUT_DIR / f"{stem}.csv"
        meta_path = OUT_DIR / f"{stem}_meta.json"
        if not csv_path.exists():
            print(f"  skip (csv missing): {stem}")
            skipped_missing += 1
            continue
        full_meta = {
            "test_name": stem.rsplit("_", 1)[0],
            "timestamp": stem.split("_")[-1],
            "csv": csv_path.name,
            "png": f"{stem}.png" if (OUT_DIR / f"{stem}.png").exists() else None,
            "hardware": HW,
            **meta,
        }
        with open(meta_path, "w") as fh:
            json.dump(full_meta, fh, indent=2)
        print(f"  wrote: {meta_path.name}")
        written += 1
    print(f"\nDone. {written} sidecars written, {skipped_missing} CSVs missing.")


if __name__ == "__main__":
    main()
