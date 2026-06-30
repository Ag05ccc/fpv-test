#!/usr/bin/env python3
"""Read or set Betaflight SITL debug_mode over MSP/TCP."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_msp import (  # noqa: E402
    MSP_ADVANCED_CONFIG,
    MSP_EEPROM_WRITE,
    MSP_SET_ADVANCED_CONFIG,
    encode_advanced_config,
    msp_command,
    msp_request,
    parse_advanced_config,
)


DEBUG_MODE_NAMES = [
    "NONE",
    "CYCLETIME",
    "BATTERY",
    "GYRO_FILTERED",
    "ACCELEROMETER",
    "PIDLOOP",
    "RC_INTERPOLATION",
    "ANGLERATE",
    "ESC_SENSOR",
    "SCHEDULER",
    "STACK",
    "ESC_SENSOR_RPM",
    "ESC_SENSOR_TMP",
    "ALTITUDE",
    "FFT",
    "FFT_TIME",
    "FFT_FREQ",
    "RX_FRSKY_SPI",
    "RX_SFHSS_SPI",
    "GYRO_RAW",
    "MULTI_GYRO_RAW",
    "MULTI_GYRO_DIFF",
    "MAX7456_SIGNAL",
    "MAX7456_SPICLOCK",
    "SBUS",
    "FPORT",
    "RANGEFINDER",
    "RANGEFINDER_QUALITY",
    "OPTICALFLOW",
    "LIDAR_TF",
    "ADC_INTERNAL",
    "RUNAWAY_TAKEOFF",
    "SDIO",
    "CURRENT_SENSOR",
    "USB",
    "SMARTAUDIO",
    "RTH",
    "ITERM_RELAX",
    "ACRO_TRAINER",
    "RC_SMOOTHING",
    "RX_SIGNAL_LOSS",
    "RC_SMOOTHING_RATE",
    "ANTI_GRAVITY",
    "DYN_LPF",
    "RX_SPEKTRUM_SPI",
    "DSHOT_RPM_TELEMETRY",
    "RPM_FILTER",
    "D_MAX",
    "AC_CORRECTION",
    "AC_ERROR",
    "MULTI_GYRO_SCALED",
    "DSHOT_RPM_ERRORS",
    "CRSF_LINK_STATISTICS_UPLINK",
    "CRSF_LINK_STATISTICS_PWR",
    "CRSF_LINK_STATISTICS_DOWN",
    "BARO",
    "AUTOPILOT_ALTITUDE",
    "DYN_IDLE",
    "FEEDFORWARD_LIMIT",
    "FEEDFORWARD",
    "BLACKBOX_OUTPUT",
    "GYRO_SAMPLE",
    "RX_TIMING",
    "D_LPF",
    "VTX_TRAMP",
    "GHST",
    "GHST_MSP",
    "SCHEDULER_DETERMINISM",
    "TIMING_ACCURACY",
    "RX_EXPRESSLRS_SPI",
    "RX_EXPRESSLRS_PHASELOCK",
    "RX_STATE_TIME",
    "GPS_RESCUE_VELOCITY",
    "GPS_RESCUE_HEADING",
    "GPS_RESCUE_TRACKING",
    "GPS_CONNECTION",
    "ATTITUDE",
    "VTX_MSP",
    "GPS_DOP",
    "FAILSAFE",
    "GYRO_CALIBRATION",
    "ANGLE_MODE",
    "ANGLE_TARGET",
    "CURRENT_ANGLE",
    "DSHOT_TELEMETRY_COUNTS",
    "RPM_LIMIT",
    "RC_STATS",
    "MAG_CALIB",
    "MAG_TASK_RATE",
    "EZLANDING",
    "TPA",
    "S_TERM",
    "SPA",
    "TASK",
    "GIMBAL",
    "WING_SETPOINT",
    "AUTOPILOT_POSITION",
    "CHIRP",
    "FLASH_TEST_PRBS",
    "MAVLINK_TELEMETRY",
    "OPTICALFLOW_POS",
    "POSITION_SOURCE",
    "AUTOPILOT_PID",
]

DEBUG_MODE_BY_NAME = {name: index for index, name in enumerate(DEBUG_MODE_NAMES)}


def debug_mode_name(value: int | None) -> str:
    if value is None:
        return "unknown"
    if 0 <= value < len(DEBUG_MODE_NAMES):
        return DEBUG_MODE_NAMES[value]
    return "mode%d" % value


def parse_debug_mode_value(value: str) -> int:
    normalized = value.strip().upper().replace("-", "_")
    if normalized.startswith("DEBUG_"):
        normalized = normalized[len("DEBUG_"):]
    if normalized in DEBUG_MODE_BY_NAME:
        return DEBUG_MODE_BY_NAME[normalized]
    try:
        parsed = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("debug mode must be 0..255 or a known debug name") from exc
    if not 0 <= parsed <= 255:
        raise argparse.ArgumentTypeError("debug mode must be 0..255")
    return parsed


def read_advanced_config(host: str, port: int, timeout: float) -> dict:
    config = parse_advanced_config(msp_request(host, port, MSP_ADVANCED_CONFIG, timeout))
    if not config.get("valid") or config.get("debug_mode") is None:
        raise RuntimeError("invalid MSP_ADVANCED_CONFIG payload")
    return config


def format_config(label: str, config: dict) -> str:
    mode = config.get("debug_mode")
    return "%s debug_mode=%s/%s debug_mode_count=%s" % (
        label,
        mode,
        debug_mode_name(mode),
        config.get("debug_mode_count"),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--debug-mode", type=parse_debug_mode_value, default=None,
                        help="Set debug_mode by number or name, e.g. PIDLOOP, ANGLERATE, ANGLE_TARGET")
    parser.add_argument("--save", action="store_true", help="Persist with MSP_EEPROM_WRITE")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    before = read_advanced_config(args.host, args.port, args.timeout)
    print(format_config("before", before))

    if args.debug_mode is not None:
        count = before.get("debug_mode_count")
        if count is not None and args.debug_mode >= int(count):
            print(
                "result=FAIL debug_mode %d is outside debug_mode_count %d" % (args.debug_mode, count),
                file=sys.stderr,
            )
            return 1
        msp_command(
            args.host,
            args.port,
            MSP_SET_ADVANCED_CONFIG,
            encode_advanced_config(before, debug_mode=args.debug_mode),
            args.timeout,
        )
        after = read_advanced_config(args.host, args.port, args.timeout)
        print(format_config("after", after))
        if after["debug_mode"] != args.debug_mode:
            print("result=FAIL debug_mode did not update", file=sys.stderr)
            return 1
        if args.save:
            msp_command(args.host, args.port, MSP_EEPROM_WRITE, b"", args.timeout)
            print("saved with MSP_EEPROM_WRITE")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
