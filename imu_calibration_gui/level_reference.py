"""Runtime level / zero reference for live visualization (not saved to calibration JSON)."""

from __future__ import annotations

from dataclasses import dataclass

from axis_conventions import tilt_from_accel, yaw_rate_from_gyro

LEVEL_AVERAGE_COUNT = 25
LEVEL_MIN_SAMPLES = 8


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
    roll: float = 0.0
    pitch: float = 0.0

    def clear(self) -> None:
        self.active = False
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.roll = self.pitch = 0.0

    def capture(self, samples: list[dict[str, float]]) -> bool:
        """Average recent samples as the level zero. Returns False if too few samples."""
        if len(samples) < LEVEL_MIN_SAMPLES:
            return False

        use = samples[-LEVEL_AVERAGE_COUNT:]
        n = len(use)
        self.ax = sum(s["ax"] for s in use) / n
        self.ay = sum(s["ay"] for s in use) / n
        self.az = sum(s["az"] for s in use) / n
        self.gx = sum(s["gx"] for s in use) / n
        self.gy = sum(s["gy"] for s in use) / n
        self.gz = sum(s["gz"] for s in use) / n
        self.roll, self.pitch = tilt_from_accel(self.ax, self.ay, self.az)
        self.active = True
        return True

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

        rax, ray, raz = ax - self.ax, ay - self.ay, az - self.az
        rgx, rgy, rgz = gx - self.gx, gy - self.gy, gz - self.gz
        roll, pitch = tilt_from_accel(ax, ay, az)
        return {
            "ax": rax, "ay": ray, "az": raz,
            "gx": rgx, "gy": rgy, "gz": rgz,
            "roll": roll - self.roll,
            "pitch": pitch - self.pitch,
            "yaw": yaw_rate_from_gyro(rgx),
        }

    def status_message(self) -> str:
        if not self.active:
            return "Place IMU level, hold still, then click Level Your IMU"
        return (
            f"Leveled — zero set  ·  ref roll {self.roll:+.1f}° pitch {self.pitch:+.1f}°  "
            f"·  click Stop level to reset"
        )
