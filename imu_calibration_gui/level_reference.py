"""Runtime level / zero reference for live visualization (not saved to calibration JSON)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from axis_conventions import tilt_from_accel, yaw_rate_from_gyro
from calibration import CALIBRATION_CAPTURE_SECONDS, CALIBRATION_MIN_SAMPLES
from orientation_block import relative_rotation_from_level

LEVEL_CAPTURE_SECONDS = CALIBRATION_CAPTURE_SECONDS
LEVEL_MIN_SAMPLES = CALIBRATION_MIN_SAMPLES


@dataclass
class LevelReference:
    """Zero pose captured while the IMU is held level and still."""

    active: bool = False
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0

    def clear(self) -> None:
        self.active = False
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0

    def capture(self, samples: list[dict[str, float]]) -> bool:
        """Average samples as the level zero. Returns False if too few samples."""
        if len(samples) < LEVEL_MIN_SAMPLES:
            return False

        n = len(samples)
        self.ax = sum(s["ax"] for s in samples) / n
        self.ay = sum(s["ay"] for s in samples) / n
        self.az = sum(s["az"] for s in samples) / n
        self.gx = sum(s["gx"] for s in samples) / n
        self.gy = sum(s["gy"] for s in samples) / n
        self.gz = sum(s["gz"] for s in samples) / n
        self.active = True
        return True

    def relative_rotation(
        self, ax: float, ay: float, az: float,
    ) -> np.ndarray:
        if not self.active:
            return np.eye(3)
        return relative_rotation_from_level(ax, ay, az, self.ax, self.ay, self.az)

    def relative_tilt(self, ax: float, ay: float, az: float) -> tuple[float, float]:
        """Roll/pitch in degrees relative to the captured level pose."""
        if not self.active:
            return 0.0, 0.0
        d_rot = self.relative_rotation(ax, ay, az)
        z_in_ref = d_rot @ np.array([0.0, 0.0, 1.0])
        mag = float(np.linalg.norm([ax, ay, az]))
        if mag < 0.5:
            mag = 9.80665
        return tilt_from_accel(z_in_ref[0] * mag, z_in_ref[1] * mag, z_in_ref[2] * mag)

    def relative(
        self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float,
    ) -> dict[str, float]:
        """Values relative to the captured level pose."""
        if not self.active:
            return {
                "ax": ax, "ay": ay, "az": az,
                "gx": gx, "gy": gy, "gz": gz,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            }

        roll, pitch = self.relative_tilt(ax, ay, az)
        return {
            "ax": ax - self.ax,
            "ay": ay - self.ay,
            "az": az - self.az,
            "gx": gx - self.gx,
            "gy": gy - self.gy,
            "gz": gz - self.gz,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw_rate_from_gyro(gx - self.gx),
        }

    def status_message(self, *, capturing: bool = False, seconds_left: float | None = None) -> str:
        if capturing and seconds_left is not None:
            return f"Hold IMU level and still… capturing zero  ·  {seconds_left:.0f}s left"
        if not self.active:
            return (
                f"Place IMU level, hold still, then click Level Your IMU "
                f"({int(LEVEL_CAPTURE_SECONDS)} s capture)"
            )
        return "Ready — plotting relative to level zero  ·  click Stop level to reset"
