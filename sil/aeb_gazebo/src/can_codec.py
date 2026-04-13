"""
CAN Signal Codec
================
Encode/decode CAN signals matching aeb_system.dbc layout.
All signals use Intel byte order (little-endian), unsigned raw + offset.

Used by both perception_node.py (TX) and scenario_controller.py (RX).
No ROS 2 dependency — pure python-can.

DBC formula: physical = raw * factor + offset
Encode:      raw = (physical - offset) / factor
"""

import struct


def _pack(data: bytearray, start_bit: int, length: int, raw: int):
    """Pack unsigned integer into LE CAN payload."""
    for i in range(length):
        abs_bit = start_bit + i
        byte_pos = abs_bit // 8
        bit_pos = abs_bit % 8
        bit_val = (raw >> i) & 1
        data[byte_pos] = (data[byte_pos] & ~(1 << bit_pos)) | (bit_val << bit_pos)


def _unpack(data: bytes, start_bit: int, length: int) -> int:
    """Unpack unsigned integer from LE CAN payload."""
    result = 0
    for i in range(length):
        abs_bit = start_bit + i
        byte_pos = abs_bit // 8
        bit_pos = abs_bit % 8
        result |= ((data[byte_pos] >> bit_pos) & 1) << i
    return result


def _encode(physical: float, factor: float, offset: float) -> int:
    raw = (physical - offset) / factor
    return max(0, int(round(raw)))


def _decode(raw: int, factor: float, offset: float) -> float:
    return raw * factor + offset


# ═══════════════════════════════════════════════════════════════════════════
#  TX: Test Harness → Zephyr ECU
# ═══════════════════════════════════════════════════════════════════════════

def encode_ego_vehicle(speed_ms: float, accel_ms2: float,
                       yaw_dps: float = 0.0, steering_deg: float = 0.0) -> bytes:
    """Encode AEB_EgoVehicle (0x100, 8 bytes)."""
    data = bytearray(8)
    _pack(data, 0,  16, _encode(speed_ms,     0.01,  0.0))
    _pack(data, 16, 16, _encode(accel_ms2,    0.001, -32.0))
    _pack(data, 32, 16, _encode(yaw_dps,      0.01,  -327.68))
    _pack(data, 48, 16, _encode(steering_deg, 0.1,   -3276.8))
    return bytes(data)


def encode_driver_input(brake_pct: int = 0, accel_pct: int = 0,
                        aeb_enable: int = 1, override: int = 0) -> bytes:
    """Encode AEB_DriverInput (0x101, 4 bytes)."""
    data = bytearray(4)
    _pack(data, 0,  8, brake_pct)
    _pack(data, 8,  8, accel_pct)
    _pack(data, 16, 1, aeb_enable)
    _pack(data, 17, 1, override)
    return bytes(data)


def encode_radar_target(distance_m: float, rel_speed_ms: float,
                        ttc_s: float, confidence: int = 15) -> bytes:
    """Encode AEB_RadarTarget (0x120, 8 bytes)."""
    data = bytearray(8)
    _pack(data, 0,  16, _encode(distance_m,   0.01,  0.0))
    _pack(data, 16, 16, _encode(rel_speed_ms, 0.01,  -327.68))
    _pack(data, 32, 16, _encode(ttc_s,        0.001, 0.0))
    _pack(data, 48, 8,  confidence)
    _pack(data, 56, 8,  0)  # Reserved
    return bytes(data)


# ═══════════════════════════════════════════════════════════════════════════
#  RX: Zephyr ECU → Test Harness
# ═══════════════════════════════════════════════════════════════════════════

def decode_brake_cmd(data: bytes) -> dict:
    """Decode AEB_BrakeCmd (0x080, 4 bytes)."""
    return {
        'brake_request':  _unpack(data, 0,  1),
        'brake_pressure': _decode(_unpack(data, 1,  15), 0.1, 0.0),
        'brake_mode':     _unpack(data, 16, 3),
        'alive_counter':  _unpack(data, 24, 4),
        'crc':            _unpack(data, 28, 4),
    }


def decode_fsm_state(data: bytes) -> dict:
    """Decode AEB_FSMState (0x200, 4 bytes)."""
    FSM_NAMES = {0: 'OFF', 1: 'STANDBY', 2: 'WARNING',
                 3: 'BRAKE_L1', 4: 'BRAKE_L2', 5: 'BRAKE_L3', 6: 'POST_BRAKE'}
    state = _unpack(data, 0, 8)
    return {
        'fsm_state':     state,
        'fsm_name':      FSM_NAMES.get(state, f'UNKNOWN({state})'),
        'alert_level':   _unpack(data, 8, 8),
        'brake_active':  _unpack(data, 16, 8),
        'ttc_threshold': _decode(_unpack(data, 24, 8), 0.1, 0.0),
    }


def decode_alert(data: bytes) -> dict:
    """Decode AEB_Alert (0x300, 2 bytes)."""
    return {
        'alert_type':   _unpack(data, 0, 8),
        'alert_active': _unpack(data, 8, 1),
        'buzzer_cmd':   _unpack(data, 9, 3),
    }
