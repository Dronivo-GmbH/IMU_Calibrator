"""Convert BMI160 serial samples to physical units (m/s², deg/s, µT)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from calibration import GRAVITY

# Bosch BMI160 typical sensitivities (datasheet)
ACCEL_LSB_PER_G: dict[int, float] = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_LSB_PER_DPS: dict[int, float] = {
    125: 262.4, 250: 131.2, 500: 65.6, 1000: 32.8, 2000: 16.4,
}

# DFRobot SEN0250 defaults when firmware streams raw int16 LSB
DEFAULT_ACCEL_RANGE_G = 2
DEFAULT_GYRO_RANGE_DPS = 250


@dataclass
class StreamScale:
    """Detected conversion for a 6-axis LSB stream."""

    is_raw_lsb: bool = False
    accel_range_g: int = DEFAULT_ACCEL_RANGE_G
    gyro_range_dps: int = DEFAULT_GYRO_RANGE_DPS

    @property
    def label(self) -> str:
        if not self.is_raw_lsb:
            return "physical units"
        return f"raw LSB → ±{self.accel_range_g}g / ±{self.gyro_range_dps}°/s"


def _accel_magnitude(data: dict[str, float]) -> float:
    return math.sqrt(data["ax"] ** 2 + data["ay"] ** 2 + data["az"] ** 2)


def looks_like_raw_lsb(data: dict[str, float]) -> bool:
    """Heuristic: firmware LSB magnitudes are thousands; physical accel ≈ 10 m/s²."""
    return _accel_magnitude(data) > 50.0


def infer_accel_range_g(data: dict[str, float]) -> int:
    mag = _accel_magnitude(data)
    if mag < 1e-6:
        return DEFAULT_ACCEL_RANGE_G

    best_g = DEFAULT_ACCEL_RANGE_G
    best_err = float("inf")
    for g, lsb_per_g in ACCEL_LSB_PER_G.items():
        converted = (mag / lsb_per_g) * GRAVITY
        err = abs(converted - GRAVITY)
        if err < best_err:
            best_err = err
            best_g = g
    return best_g


def infer_gyro_range_dps(data: dict[str, float]) -> int:
    """Use DFRobot default ±250 °/s unless readings clearly exceed that range."""
    peak = max(abs(data["gx"]), abs(data["gy"]), abs(data["gz"]))
    if peak / GYRO_LSB_PER_DPS[250] > 240:
        return 2000
    if peak / GYRO_LSB_PER_DPS[500] > 480:
        return 1000
    return DEFAULT_GYRO_RANGE_DPS


def update_stream_scale(scale: StreamScale, data: dict[str, float]) -> StreamScale:
    if not looks_like_raw_lsb(data):
        scale.is_raw_lsb = False
        return scale

    scale.is_raw_lsb = True
    scale.accel_range_g = infer_accel_range_g(data)
    scale.gyro_range_dps = infer_gyro_range_dps(data)
    return scale


def raw_lsb_to_physical(data: dict[str, float], scale: StreamScale) -> dict[str, float]:
    accel_lsb = ACCEL_LSB_PER_G[scale.accel_range_g]
    gyro_lsb = GYRO_LSB_PER_DPS[scale.gyro_range_dps]

    ax = (data["ax"] / accel_lsb) * GRAVITY
    ay = (data["ay"] / accel_lsb) * GRAVITY
    az = (data["az"] / accel_lsb) * GRAVITY
    gx = data["gx"] / gyro_lsb
    gy = data["gy"] / gyro_lsb
    gz = data["gz"] / gyro_lsb

    mx, my, mz = data["mx"], data["my"], data["mz"]
    if max(abs(mx), abs(my), abs(mz)) > 500:
        # Likely magnetometer LSB; BMM150 default ≈ 0.3 µT/LSB
        mx *= 0.3
        my *= 0.3
        mz *= 0.3

    return {
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
        "mx": mx, "my": my, "mz": mz,
    }


def normalize_sample(data: dict[str, float], scale: StreamScale) -> dict[str, float]:
    update_stream_scale(scale, data)
    if scale.is_raw_lsb:
        return raw_lsb_to_physical(data, scale)
    return dict(data)


def normalize_samples(samples: list[dict[str, float]], scale: StreamScale) -> list[dict[str, float]]:
    if not samples:
        return []
    update_stream_scale(scale, samples[0])
    return [normalize_sample(s, scale) for s in samples]
