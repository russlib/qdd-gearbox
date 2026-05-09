"""FTMS (Fitness Machine Service) protocol for the Saris Hammer H2.

Reference: GoldenCheetah src/Train/BT40Device.cpp + src/Train/Ftms.h.

The H2 exposes both a proprietary CycleOps service (engages brake on any GATT
op) and the standard FTMS service. GoldenCheetah ignores the proprietary one
entirely and drives the trainer through FTMS only — that path keeps the brake
released until the host writes a target.

Service:        0x1826 (Fitness Machine)
Indoor Bike:    0x2AD2 (notify)   — telemetry
Control Point:  0x2AD9 (write + indicate)
Status:         0x2ADA (notify)
Feature:        0x2ACC (read)

CCCD on Control Point should use INDICATIONS (0x0200), per FTMS spec.
GoldenCheetah uses notifications (0x0100) and gets away with it on most
trainers; bleak's start_notify auto-picks the right CCCD value based on
the characteristic's properties.
"""

from __future__ import annotations
import struct
from dataclasses import dataclass
from enum import IntEnum

# --- BLE identifiers ---
FTMS_SERVICE_UUID         = "00001826-0000-1000-8000-00805f9b34fb"
FTMS_INDOOR_BIKE_UUID     = "00002ad2-0000-1000-8000-00805f9b34fb"
FTMS_CONTROL_POINT_UUID   = "00002ad9-0000-1000-8000-00805f9b34fb"
FTMS_STATUS_UUID          = "00002ada-0000-1000-8000-00805f9b34fb"
FTMS_FEATURE_UUID         = "00002acc-0000-1000-8000-00805f9b34fb"


class FtmsOp(IntEnum):
    """FTMS Control Point opcodes (BT FTMS spec)."""
    REQUEST_CONTROL                  = 0x00
    RESET                            = 0x01
    SET_TARGET_RESISTANCE_LEVEL      = 0x04
    SET_TARGET_POWER                 = 0x05
    START_RESUME                     = 0x07
    STOP_PAUSE                       = 0x08
    SET_INDOOR_BIKE_SIM_PARAMS       = 0x11
    RESPONSE_CODE                    = 0x80


class FtmsResult(IntEnum):
    """Response codes inside an FTMS indication payload."""
    SUCCESS              = 0x01
    NOT_SUPPORTED        = 0x02
    INVALID_PARAMETER    = 0x03
    OPERATION_FAILED     = 0x04
    CONTROL_NOT_PERMITTED = 0x05


@dataclass
class IndoorBikeMeasurement:
    """Parsed Indoor Bike Data (0x2AD2) notification.

    Flags (uint16 LE) bit definitions per FTMS spec:
      bit 0: more data (instant speed PRESENT if 0, ABSENT if 1)
      bit 1: avg speed
      bit 2: instant cadence
      bit 3: avg cadence
      bit 4: total distance
      bit 5: resistance level
      bit 6: instant power
      bit 7: avg power
      bit 8: expended energy
      bit 9: heart rate
      bit 10: metabolic equivalent
      bit 11: elapsed time
      bit 12: remaining time
    """
    flags: int = 0
    inst_speed_kmh: float | None = None
    avg_speed_kmh: float | None = None
    inst_cadence_rpm: float | None = None
    avg_cadence_rpm: float | None = None
    total_distance_m: int | None = None
    resistance_level: int | None = None
    inst_power_w: int | None = None
    avg_power_w: int | None = None
    expended_energy_kj: int | None = None
    heart_rate_bpm: int | None = None
    elapsed_time_s: int | None = None
    remaining_time_s: int | None = None
    raw: bytes = b""


def parse_indoor_bike(data: bytes) -> IndoorBikeMeasurement:
    """Parse an Indoor Bike Data (0x2AD2) FTMS notification."""
    m = IndoorBikeMeasurement(raw=bytes(data))
    flags = struct.unpack_from("<H", data, 0)[0]
    m.flags = flags
    off = 2

    # bit 0 == 0 -> Instantaneous Speed PRESENT
    if not (flags & 0x0001):
        raw = struct.unpack_from("<H", data, off)[0]
        m.inst_speed_kmh = raw / 100.0
        off += 2
    if flags & 0x0002:
        raw = struct.unpack_from("<H", data, off)[0]
        m.avg_speed_kmh = raw / 100.0
        off += 2
    if flags & 0x0004:
        raw = struct.unpack_from("<H", data, off)[0]
        m.inst_cadence_rpm = raw / 2.0
        off += 2
    if flags & 0x0008:
        raw = struct.unpack_from("<H", data, off)[0]
        m.avg_cadence_rpm = raw / 2.0
        off += 2
    if flags & 0x0010:
        # Total distance is uint24 LE
        b = data[off:off+3]
        m.total_distance_m = b[0] | (b[1] << 8) | (b[2] << 16)
        off += 3
    if flags & 0x0020:
        m.resistance_level = struct.unpack_from("<h", data, off)[0]
        off += 2
    if flags & 0x0040:
        m.inst_power_w = struct.unpack_from("<h", data, off)[0]
        off += 2
    if flags & 0x0080:
        m.avg_power_w = struct.unpack_from("<h", data, off)[0]
        off += 2
    if flags & 0x0100:
        m.expended_energy_kj = struct.unpack_from("<H", data, off)[0]
        off += 2 + 2 + 1  # total + per_hr + per_min
    if flags & 0x0200:
        m.heart_rate_bpm = data[off]
        off += 1
    if flags & 0x0400:
        off += 1
    if flags & 0x0800:
        m.elapsed_time_s = struct.unpack_from("<H", data, off)[0]
        off += 2
    if flags & 0x1000:
        m.remaining_time_s = struct.unpack_from("<H", data, off)[0]
        off += 2
    return m


def parse_control_response(data: bytes) -> dict:
    """Parse an indication on the FTMS Control Point (0x2AD9).

    Format: 0x80 <op_request> <result_code> [params...]
    """
    if len(data) < 3 or data[0] != FtmsOp.RESPONSE_CODE:
        return {"raw_hex": bytes(data).hex(), "valid": False}
    return {
        "valid": True,
        "request_op": data[1],
        "request_op_name": FtmsOp(data[1]).name if data[1] in FtmsOp._value2member_map_ else f"0x{data[1]:02X}",
        "result": data[2],
        "result_name": FtmsResult(data[2]).name if data[2] in FtmsResult._value2member_map_ else f"0x{data[2]:02X}",
        "extra_hex": bytes(data[3:]).hex() if len(data) > 3 else "",
        "raw_hex": bytes(data).hex(),
    }


# --- Command builders ---

def cmd_request_control() -> bytes:
    return bytes([FtmsOp.REQUEST_CONTROL])

def cmd_reset() -> bytes:
    return bytes([FtmsOp.RESET])

def cmd_start_resume() -> bytes:
    return bytes([FtmsOp.START_RESUME])

def cmd_stop_pause(action: int = 0x01) -> bytes:
    """action: 0x01 = stop, 0x02 = pause."""
    return bytes([FtmsOp.STOP_PAUSE, action])

def cmd_set_target_power(watts: int) -> bytes:
    return struct.pack("<Bh", FtmsOp.SET_TARGET_POWER, int(watts))

def cmd_set_target_resistance(level: int) -> bytes:
    return struct.pack("<BB", FtmsOp.SET_TARGET_RESISTANCE_LEVEL, int(level) & 0xFF)

def cmd_set_sim_params(wind_kmh: float, grade_pct: float, crr: float = 0.004, cw: float = 0.51) -> bytes:
    """Indoor Bike Simulation Parameters (opcode 0x11).

    Wind speed:      sint16, 0.001 m/s
    Grade:           sint16, 0.01 %
    Coeff rolling:   uint8,  0.0001
    Wind resistance: uint8,  0.01 kg/m
    """
    wind_int = int(round(wind_kmh / 3.6 / 0.001))
    grade_int = int(round(grade_pct / 0.01))
    crr_int = int(round(crr / 0.0001)) & 0xFF
    cw_int = int(round(cw / 0.01)) & 0xFF
    return struct.pack("<BhhBB", FtmsOp.SET_INDOOR_BIKE_SIM_PARAMS, wind_int, grade_int, crr_int, cw_int)
