"""Serial command protocol for pushing calibration to the MCU."""

from __future__ import annotations

import json
from typing import Any

from calibration import CalibrationData


def format_set_cal_command(cal: CalibrationData) -> str:
    payload = cal.to_dict()
    payload.pop("saved_at", None)
    return f"SET_CAL {json.dumps(payload, separators=(',', ':'))}"


def parse_cal_response(line: str) -> tuple[str, dict[str, Any] | None]:
    line = line.strip()
    if line in ("PONG", "CAL_OK", "CAL_ERR", "CAL_MODE_ON", "CAL_MODE_OFF", "CAL_MODE:ON", "CAL_MODE:OFF"):
        return line, None
    if line.startswith("CAL "):
        try:
            return "CAL", json.loads(line[4:])
        except json.JSONDecodeError:
            return "CAL_ERR", None
    return "", None


def is_command_response(line: str) -> bool:
    line = line.strip()
    return (
        line in ("PONG", "CAL_OK", "CAL_ERR", "CAL_MODE_ON", "CAL_MODE_OFF", "CAL_MODE:ON", "CAL_MODE:OFF")
        or line.startswith("CAL ")
    )
