#!/usr/bin/env python3
"""Shared MSP helpers for local Betaflight SITL tools."""

from __future__ import annotations

import socket
import struct


MSP_RC = 105
MSP_ATTITUDE = 108
MSP_MOTOR = 104
MSP_ADVANCED_CONFIG = 90
MSP_SET_ADVANCED_CONFIG = 91
MSP_BOXNAMES = 116
MSP_BOXIDS = 119
MSP_STATUS_EX = 150
MSP_MIXER_CONFIG = 42
MSP_SET_MIXER_CONFIG = 43
MSP_EEPROM_WRITE = 250
MSP_PID = 112
MSP_SET_PID = 202
MSP_RC_TUNING = 111
MSP_SET_RC_TUNING = 204
MSP_DEBUG = 254

ARMING_DISABLE_NAMES = [
    "NOGYRO",
    "FAILSAFE",
    "RXLOSS",
    "NOT_DISARMED",
    "BOXFAILSAFE",
    "RUNAWAY",
    "CRASH",
    "THROTTLE",
    "ANGLE",
    "BOOTGRACE",
    "NOPREARM",
    "LOAD",
    "CALIB",
    "CLI",
    "CMS",
    "BST",
    "MSP",
    "PARALYZE",
    "GPS",
    "RESCUE_SW",
    "DSHOT_TELEM",
    "REBOOT_REQD",
    "DSHOT_BBANG",
    "NO_ACC_CAL",
    "MOTOR_PROTO",
    "FLIP_SWITCH",
    "ALT_HOLD_SW",
    "POS_HOLD_SW",
    "ARM_SWITCH",
]


def msp_checksum(code: int, payload: bytes = b"") -> int:
    checksum = len(payload) ^ code
    for byte in payload:
        checksum ^= byte
    return checksum


def msp_encode(code: int, payload: bytes = b"") -> bytes:
    return b"$M<" + bytes([len(payload), code]) + payload + bytes([msp_checksum(code, payload)])


def read_msp_frame(sock: socket.socket) -> tuple[int, bytes]:
    while True:
        byte = sock.recv(1)
        if not byte:
            raise RuntimeError("MSP connection closed")
        if byte != b"$":
            continue
        if sock.recv(1) != b"M":
            continue
        direction = sock.recv(1)
        if direction not in (b">", b"!"):
            continue
        header = sock.recv(2)
        if len(header) != 2:
            raise RuntimeError("short MSP header")
        size, code = header[0], header[1]
        payload = b""
        while len(payload) < size + 1:
            chunk = sock.recv(size + 1 - len(payload))
            if not chunk:
                raise RuntimeError("short MSP payload")
            payload += chunk
        body = payload[:-1]
        checksum = payload[-1]
        if checksum != msp_checksum(code, body):
            raise RuntimeError("bad MSP checksum for code %d" % code)
        if direction == b"!":
            raise RuntimeError("MSP returned error for code %d" % code)
        return code, body


def msp_request(host: str, port: int, code: int, timeout: float) -> bytes:
    return msp_command(host, port, code, b"", timeout)


def msp_command(host: str, port: int, code: int, payload: bytes, timeout: float) -> bytes:
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(msp_encode(code, payload))
        for _ in range(10):
            response_code, payload = read_msp_frame(sock)
            if response_code == code:
                return payload
    raise RuntimeError("MSP response %d not received" % code)


def msp_request_many(host: str, port: int, codes: list[int], timeout: float) -> dict[int, bytes]:
    responses: dict[int, bytes] = {}
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        for code in codes:
            sock.sendall(msp_encode(code))
            for _ in range(10):
                response_code, payload = read_msp_frame(sock)
                if response_code == code:
                    responses[code] = payload
                    break
            else:
                raise RuntimeError("MSP response %d not received" % code)
    return responses


def parse_u16_list(payload: bytes) -> list[int]:
    if len(payload) < 2:
        return []
    count = len(payload) // 2
    return list(struct.unpack("<%dH" % count, payload[:count * 2]))


def parse_mixer_config(payload: bytes) -> dict:
    if len(payload) < 2:
        return {"valid": False, "mixer_mode": None, "yaw_motors_reversed": None}
    return {
        "valid": True,
        "mixer_mode": payload[0],
        "yaw_motors_reversed": bool(payload[1]),
    }


def encode_mixer_config(mixer_mode: int, yaw_motors_reversed: bool) -> bytes:
    if not 0 <= int(mixer_mode) <= 255:
        raise ValueError("mixer_mode must fit in one byte")
    return bytes([int(mixer_mode), 1 if yaw_motors_reversed else 0])


def parse_advanced_config(payload: bytes) -> dict:
    if len(payload) < 14:
        return {
            "valid": False,
            "debug_mode": None,
            "debug_mode_count": None,
        }
    return {
        "valid": True,
        "gyro_sync_denom": payload[0],
        "pid_process_denom": payload[1],
        "motor_continuous_update": bool(payload[2]),
        "motor_protocol": payload[3],
        "motor_pwm_rate": struct.unpack_from("<H", payload, 4)[0],
        "motor_idle": struct.unpack_from("<H", payload, 6)[0],
        "motor_inversion": bool(payload[9]),
        "gyro_high_fsr": payload[11],
        "gyro_movement_calibration_threshold": payload[12],
        "gyro_calibration_duration": struct.unpack_from("<H", payload, 13)[0] if len(payload) >= 15 else None,
        "gyro_offset_yaw": struct.unpack_from("<H", payload, 15)[0] if len(payload) >= 17 else None,
        "gyro_check_overflow": payload[17] if len(payload) >= 18 else None,
        "debug_mode": payload[18] if len(payload) >= 19 else None,
        "debug_mode_count": payload[19] if len(payload) >= 20 else None,
    }


def _byte_value(config: dict, key: str, default: int | None = None) -> int:
    value = config.get(key, default)
    if value is None:
        raise ValueError("advanced config missing %s" % key)
    value = int(value)
    if not 0 <= value <= 255:
        raise ValueError("%s must fit in one byte" % key)
    return value


def _u16_value(config: dict, key: str) -> int:
    value = config.get(key)
    if value is None:
        raise ValueError("advanced config missing %s" % key)
    value = int(value)
    if not 0 <= value <= 65535:
        raise ValueError("%s must fit in uint16" % key)
    return value


def _bool_byte(config: dict, key: str) -> int:
    value = config.get(key)
    if value is None:
        raise ValueError("advanced config missing %s" % key)
    return 1 if bool(value) else 0


def encode_advanced_config(config: dict, debug_mode: int | None = None) -> bytes:
    """Encode MSP_SET_ADVANCED_CONFIG while preserving the current config fields."""
    selected_debug_mode = config.get("debug_mode") if debug_mode is None else debug_mode
    if selected_debug_mode is None:
        raise ValueError("advanced config missing debug_mode")
    selected_debug_mode = int(selected_debug_mode)
    if not 0 <= selected_debug_mode <= 255:
        raise ValueError("debug_mode must fit in one byte")

    payload = bytearray()
    payload.append(_byte_value(config, "gyro_sync_denom", 1))
    payload.append(_byte_value(config, "pid_process_denom"))
    payload.append(_bool_byte(config, "motor_continuous_update"))
    payload.append(_byte_value(config, "motor_protocol"))
    payload += struct.pack("<H", _u16_value(config, "motor_pwm_rate"))
    payload += struct.pack("<H", _u16_value(config, "motor_idle"))
    payload.append(0)  # deprecated gyro_use_32khz, still consumed by Betaflight
    payload.append(_bool_byte(config, "motor_inversion"))
    payload.append(0)  # deprecated gyro_to_use, still consumed by Betaflight
    payload.append(_byte_value(config, "gyro_high_fsr"))
    payload.append(_byte_value(config, "gyro_movement_calibration_threshold"))
    payload += struct.pack("<H", _u16_value(config, "gyro_calibration_duration"))
    payload += struct.pack("<H", _u16_value(config, "gyro_offset_yaw"))
    payload.append(_byte_value(config, "gyro_check_overflow"))
    payload.append(selected_debug_mode)
    return bytes(payload)


def parse_debug(payload: bytes) -> list[int]:
    if len(payload) < 2:
        return []
    count = len(payload) // 2
    return list(struct.unpack("<%dh" % count, payload[:count * 2]))


def parse_attitude(payload: bytes) -> dict | None:
    if len(payload) < 6:
        return None
    roll, pitch, yaw = struct.unpack("<hhH", payload[:6])
    return {
        "roll": roll / 10.0,
        "pitch": pitch / 10.0,
        "yaw": yaw,
    }


def parse_box_names(payload: bytes) -> list[str]:
    text = payload.decode("ascii", errors="replace")
    return [name for name in text.split(";") if name]


def parse_box_ids(payload: bytes) -> list[int]:
    return list(payload)


def _mode_flag_set(first_flags: int, extra_flags: bytes, bit: int) -> bool:
    if bit < 32:
        return bool(first_flags & (1 << bit))
    extra_index = bit - 32
    byte_index = extra_index // 8
    bit_index = extra_index % 8
    if byte_index >= len(extra_flags):
        return False
    return bool(extra_flags[byte_index] & (1 << bit_index))


def parse_status_ex(
    payload: bytes,
    box_names: list[str] | None = None,
    box_ids: list[int] | None = None,
) -> dict:
    if len(payload) < 16:
        return {
            "valid": False,
            "armed": None,
            "flight_mode_flags": 0,
            "flight_mode_extra_flags": [],
            "active_modes_valid": False,
            "active_modes": [],
            "box_names": box_names or [],
            "box_ids": box_ids or [],
            "arming_disable_flags": 0,
            "arming_disable_names": [],
        }

    flight_mode_flags = struct.unpack_from("<I", payload, 6)[0]
    extra_count = payload[15] & 0x0F
    extra_start = 16
    extra_end = min(len(payload), extra_start + extra_count)
    extra_flags = payload[extra_start:extra_end]

    arming_offset = extra_start + extra_count
    arming_count = None
    arming_flags = 0
    if len(payload) >= arming_offset + 5:
        arming_count = payload[arming_offset]
        arming_flags = struct.unpack_from("<I", payload, arming_offset + 1)[0]

    active_arming = [
        name
        for bit, name in enumerate(ARMING_DISABLE_NAMES)
        if arming_flags & (1 << bit)
    ]

    names = box_names or []
    ids = box_ids or []
    active_modes = [
        name
        for index, name in enumerate(names)
        if _mode_flag_set(flight_mode_flags, extra_flags, index)
    ]
    active_mode_ids = [
        ids[index]
        for index in range(min(len(names), len(ids)))
        if _mode_flag_set(flight_mode_flags, extra_flags, index)
    ]
    active_modes_valid = bool(names)

    return {
        "valid": True,
        "armed": ("ARM" in active_modes) if active_modes_valid else None,
        "flight_mode_flags": flight_mode_flags,
        "flight_mode_extra_flags": list(extra_flags),
        "active_modes_valid": active_modes_valid,
        "active_modes": active_modes,
        "active_mode_ids": active_mode_ids,
        "box_names": names,
        "box_ids": ids,
        "arming_disable_count": arming_count,
        "arming_disable_flags": arming_flags,
        "arming_disable_names": active_arming,
    }
