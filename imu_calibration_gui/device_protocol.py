"""Serial command protocol for pushing calibration to the MCU."""

from __future__ import annotations

import json
from typing import Any

from calibration import CalibrationData


def format_set_cal_command(cal: CalibrationData) -> str:
    payload = cal.to_dict()
    payload.pop("saved_at", None)
    return f"SET_CAL {json.dumps(payload, separators=(',', ':'))}"


def format_bno_write_profile_command(profile: dict[str, Any]) -> str:
    return f"BNO_WRITE_PROFILE {json.dumps(profile, separators=(',', ':'))}"


def parse_cal_response(line: str) -> tuple[str, dict[str, Any] | None]:
    line = line.strip()
    if line in (
        "PONG",
        "CAL_OK",
        "CAL_ERR",
        "CAL_MODE_ON",
        "CAL_MODE_OFF",
        "CAL_MODE:ON",
        "CAL_MODE:OFF",
        "BNO_WRITE_OK",
        "BNO_WRITE_ERR",
    ):
        return line, None
    if line.startswith("CAL "):
        try:
            return "CAL", json.loads(line[4:])
        except json.JSONDecodeError:
            return "CAL_ERR", None
    for prefix, kind in (
        ("BNO_BEGIN ", "BNO_BEGIN"),
        ("BNO_STATUS ", "BNO_STATUS"),
        ("BNO_PROFILE ", "BNO_PROFILE"),
        ("BNO_EULER ", "BNO_EULER"),
        ("BNO_QUAT ", "BNO_QUAT"),
        ("BNO_ERR ", "BNO_ERR"),
    ):
        if line.startswith(prefix):
            try:
                return kind, json.loads(line[len(prefix):])
            except json.JSONDecodeError:
                return "BNO_ERR", {"message": "Invalid JSON payload", "raw": line}
    return "", None


def is_command_response(line: str) -> bool:
    line = line.strip()
    return (
        line in (
            "PONG",
            "CAL_OK",
            "CAL_ERR",
            "CAL_MODE_ON",
            "CAL_MODE_OFF",
            "CAL_MODE:ON",
            "CAL_MODE:OFF",
            "BNO_WRITE_OK",
            "BNO_WRITE_ERR",
        )
        or line.startswith("CAL ")
        or line.startswith("BNO_BEGIN ")
        or line.startswith("BNO_STATUS ")
        or line.startswith("BNO_PROFILE ")
        or line.startswith("BNO_EULER ")
        or line.startswith("BNO_QUAT ")
        or line.startswith("BNO_ERR ")
    )
