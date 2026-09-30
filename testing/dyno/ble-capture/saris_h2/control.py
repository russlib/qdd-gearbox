"""Resistance control for the Saris H2 trainer.

The May 5 HEADLESS brake-release hypothesis was rejected by the May 7/8
bench findings. HEADLESS requests the Fluid2 curve and did not remove loaded-
direction drag. Command transport success is not mechanical brake release.
release_brake() fails before any write. Explicit mode requests retain their
protocol meanings; hardware operation needs the existing test procedures.

Subscribe to resistance indications before an explicitly chosen write.

References:
    Service:  c0f4013a-a837-4165-bab9-654ef70747c6
    Char:     ca31a533-a858-4dc7-a650-fdeb6dad4c14
"""
import asyncio

from bleak import BleakClient

from .protocol import (
    SARIS_RESISTANCE_UUID,
    ResistanceMode,
    build_resistance_cmd,
)


async def enable_resistance_notifications(client: BleakClient,
                                          handler=None) -> None:
    """Subscribe to notifications on the resistance characteristic.

    The H2 rejects writes (CCCD improperly configured) until this is done.
    """
    if handler is None:
        async def _ignore(sender, data):
            pass
        handler = _ignore
    await client.start_notify(SARIS_RESISTANCE_UUID, handler)


async def release_brake(client: BleakClient):
    """Unsupported: later bench records reject HEADLESS as brake release.

    Kept as an explicit failing compatibility entry point. No write or success
    message is emitted; choose no substitute hardware operation automatically.
    """
    raise RuntimeError("Brake release is unsupported on the tested H2; HEADLESS did not release drag. See H2_BENCH_FINDINGS.md.")


async def set_erg_power(client: BleakClient, watts: int):
    """ERG mode — trainer adjusts brake to maintain target wattage.

    NOTE: 0 watts ENGAGES the brake at minimum (lock-flywheel behavior).
    No tested BLE brake-release sequence is established; 0W is not a release.
    """
    cmd = build_resistance_cmd(ResistanceMode.MANUAL_POWER, watts, 0)
    await client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd, response=False)
    print(f"Sent ERG target request: {watts} W")


async def set_sim(client: BleakClient, weight_kg: float, grade_pct: float):
    """SIM mode — simulate road grade given rider weight.

    Note: param1 = weight*100, param2 = grade*10 (NOT grade*100).
    """
    weight_param = int(round(weight_kg * 100))
    grade_param  = int(round(grade_pct * 10))
    # grade is signed for descents; convert to two's-complement uint16 if negative
    if grade_param < 0:
        grade_param = (grade_param + (1 << 16)) & 0xFFFF
    cmd = build_resistance_cmd(ResistanceMode.MANUAL_SLOPE, weight_param, grade_param)
    await client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd, response=False)
    print(f"Sent SIM mode request: {weight_kg:.1f} kg @ {grade_pct:+.1f}% grade")


async def set_power_range(client: BleakClient, min_w: int, max_w: int):
    """POWER_RANGE mode — keep brake torque between two wattage limits."""
    cmd = build_resistance_cmd(ResistanceMode.POWER_RANGE, min_w, max_w)
    await client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd, response=False)
    print(f"Sent POWER_RANGE request: {min_w}-{max_w} W")


async def sweep_resistance(
    client: BleakClient,
    mode: ResistanceMode,
    start: int,
    stop: int,
    step: int,
    hold_seconds: float = 5.0,
):
    """Step through resistance levels, holding each.

    Useful for mapping the brake torque curve empirically.
    """
    for value in range(start, stop + 1, step):
        cmd = build_resistance_cmd(mode, value, 0)
        await client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd, response=False)
        if mode == ResistanceMode.MANUAL_POWER:
            label = f"{value}W"
        elif mode == ResistanceMode.MANUAL_SLOPE:
            label = f"slope param={value}"
        else:
            label = f"param={value}"
        print(f"Set {label}, holding {hold_seconds}s...")
        await asyncio.sleep(hold_seconds)
    print("Sweep complete.")
