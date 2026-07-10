"""IMU calibration math for BMI160 (and compatible 9-axis streams)."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any

import numpy as np

GRAVITY = 9.80665

IMU_MODEL_BMI160 = "BMI160"
IMU_MODEL_BNO055 = "BNO055"
CALIBRATION_FORMAT_VERSION = 1

# Static poses (level, six-face positions): average samples over this window.
CALIBRATION_CAPTURE_SECONDS = 30.0
CALIBRATION_MIN_SAMPLES = 50
CALIBRATION_CAPTURE_TIMEOUT_SECONDS = CALIBRATION_CAPTURE_SECONDS + 10.0


def imu_model_store_slug(model_label: str) -> str:
    if "BNO055" in str(model_label).upper():
        return "bno055"
    return "bmi160"


def normalize_imu_model(value: str | None) -> str:
    if value and "BNO055" in str(value).upper():
        return IMU_MODEL_BNO055
    return IMU_MODEL_BMI160


def default_calibration_store_path(model_label: str) -> Path:
    slug = imu_model_store_slug(model_label)
    return Path.home() / ".config" / "dronivo" / f"{slug}_calibration.json"


def default_export_filename(model_label: str) -> str:
    slug = imu_model_store_slug(model_label)
    return f"IMU_Calibration_{slug.upper()}.json"


def infer_imu_model_from_payload(payload: dict[str, Any]) -> str:
    if payload.get("imu_model"):
        return normalize_imu_model(payload["imu_model"])
    if payload.get("bno055_profile") or payload.get("calibration_bytes"):
        return IMU_MODEL_BNO055
    return IMU_MODEL_BMI160


def build_bno055_export_payload(profile: dict[str, Any], model_label: str) -> dict[str, Any]:
    return {
        "imu_model": normalize_imu_model(model_label),
        "format_version": CALIBRATION_FORMAT_VERSION,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "bno055_profile": profile,
    }


def save_bno055_profile(profile: dict[str, Any], model_label: str, path: Path | None = None) -> Path:
    target = path or default_calibration_store_path(model_label)
    target.parent.mkdir(parents=True, exist_ok=True)
    with open(target, "w", encoding="utf-8") as f:
        json.dump(build_bno055_export_payload(profile, model_label), f, indent=2)
    return target


def load_bno055_profile(model_label: str, path: Path | None = None) -> dict[str, Any] | None:
    target = path or default_calibration_store_path(model_label)
    if not target.exists():
        return None
    with open(target, "r", encoding="utf-8") as f:
        payload = json.load(f)
    profile = payload.get("bno055_profile", payload)
    if "calibration_bytes" not in profile:
        return None
    return profile


def build_bmi160_export_payload(cal: CalibrationData, model_label: str) -> dict[str, Any]:
    saved_at = cal.saved_at or time.strftime("%Y-%m-%d %H:%M:%S")
    cal_dict = cal.to_dict()
    return {
        "imu_model": normalize_imu_model(model_label),
        "format_version": CALIBRATION_FORMAT_VERSION,
        "saved_at": saved_at,
        "description": (
            "BMI160 calibration profile for IMU_Calibrator. "
            "Import in the GUI or send calibration fields to firmware with SET_CAL."
        ),
        "units": {
            "gyro_offset": "deg/s",
            "accel_offset": "m/s²",
            "accel_scale": "dimensionless",
            "mag_offset": "µT",
        },
        "capture": {
            "method": "average_while_hold_still",
            "static_pose_seconds": CALIBRATION_CAPTURE_SECONDS,
            "notes": (
                "Level step: place IMU flat (+Z up), hold still for 30 s. "
                "Six-face: hold each orientation still for 30 s."
            ),
        },
        "usage": {
            "import": "IMU Calibration Tool → Calibration tab → Import JSON…",
            "export": "Same tab → Export JSON… to share this file",
            "device": "Connect IMU → Write calibration to device (sends SET_CAL to firmware)",
            "fields_for_code": "Use the calibration object below (gyro_offset, accel_offset, accel_scale, mag_offset, mag_soft_iron)",
        },
        "calibration": cal_dict,
    }


def extract_bmi160_calibration(payload: dict[str, Any]) -> CalibrationData:
    if "calibration" in payload:
        return CalibrationData.from_dict(payload["calibration"])
    return CalibrationData.from_dict(payload)


def _vec3(data: dict[str, float], prefix: str) -> np.ndarray:
    return np.array([data[f"{prefix}x"], data[f"{prefix}y"], data[f"{prefix}z"]], dtype=float)


def _default_vec3() -> dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": 0.0}


def _default_mat3() -> list[list[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


@dataclass
class CalibrationData:
    gyro_offset: dict[str, float] = field(default_factory=_default_vec3)
    accel_offset: dict[str, float] = field(default_factory=_default_vec3)
    accel_scale: dict[str, float] = field(default_factory=lambda: {"x": 1.0, "y": 1.0, "z": 1.0})
    mag_offset: dict[str, float] = field(default_factory=_default_vec3)
    mag_soft_iron: list[list[float]] = field(default_factory=_default_mat3)
    saved_at: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> CalibrationData:
        cal = cls()
        for key in ("gyro_offset", "accel_offset", "accel_scale", "mag_offset"):
            if key in data:
                cal.__dict__[key] = {**cal.__dict__[key], **data[key]}
        if "mag_soft_iron" in data:
            cal.mag_soft_iron = data["mag_soft_iron"]
        cal.saved_at = data.get("saved_at", "")
        return cal

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def average_samples(samples: list[dict[str, float]], prefix: str) -> np.ndarray:
    if not samples:
        raise ValueError("No samples to average")
    values = np.array([_vec3(s, prefix) for s in samples], dtype=float)
    return values.mean(axis=0)


def calibrate_gyro(samples: list[dict[str, float]]) -> dict[str, float]:
    mean = average_samples(samples, "g")
    return {"x": float(mean[0]), "y": float(mean[1]), "z": float(mean[2])}


def calibrate_level_pose(
    samples: list[dict[str, float]], gravity_axis: str = "z",
) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
    """
    One level still capture (e.g. 30 s) → gyro zero-rate + accel flat offsets.

    Same approach as Live View “Level Your IMU”, persisted to calibration JSON.
    """
    gyro_offset = calibrate_gyro(samples)
    accel_offset, accel_scale = calibrate_accel_single_face(samples, gravity_axis=gravity_axis)
    return gyro_offset, accel_offset, accel_scale


def calibrate_accel_single_face(samples: list[dict[str, float]], gravity_axis: str = "z") -> tuple[dict[str, float], dict[str, float]]:
    """Single-position offset calibration with unit scale."""
    mean = average_samples(samples, "a")
    offset = {"x": float(mean[0]), "y": float(mean[1]), "z": float(mean[2])}
    axis_key = gravity_axis.lower()
    if axis_key not in ("x", "y", "z"):
        raise ValueError("gravity_axis must be x, y, or z")
    offset[axis_key] -= GRAVITY
    scale = {"x": 1.0, "y": 1.0, "z": 1.0}
    return offset, scale


ACCEL_FACES = [
    ("+X up", np.array([GRAVITY, 0.0, 0.0])),
    ("-X up", np.array([-GRAVITY, 0.0, 0.0])),
    ("+Y up", np.array([0.0, GRAVITY, 0.0])),
    ("-Y up", np.array([0.0, -GRAVITY, 0.0])),
    ("+Z up", np.array([0.0, 0.0, GRAVITY])),
    ("-Z up", np.array([0.0, 0.0, -GRAVITY])),
]


def calibrate_accel_six_face(face_samples: list[list[dict[str, float]]]) -> tuple[dict[str, float], dict[str, float]]:
    """
    Six-face accelerometer calibration using least-squares bias + diagonal scale.

    Each face_samples[i] corresponds to ACCEL_FACES[i] with known gravity vector.
    """
    if len(face_samples) != 6:
        raise ValueError("Six face sample sets are required")

    measured = []
    expected = []
    for samples, (_, gravity) in zip(face_samples, ACCEL_FACES):
        if not samples:
            raise ValueError("Each face must have samples")
        measured.append(average_samples(samples, "a"))
        expected.append(gravity)

    measured = np.array(measured)
    expected = np.array(expected)

    offset = np.zeros(3)
    scale = np.ones(3)
    for axis in range(3):
        m = measured[:, axis]
        e = expected[:, axis]
        denom = np.dot(e, e)
        if abs(denom) < 1e-9:
            raise ValueError(f"Insufficient excitation on axis {axis}")
        scale[axis] = np.dot(m, e) / denom
        if abs(scale[axis]) < 1e-6:
            raise ValueError(f"Invalid scale on axis {axis}")
        offset[axis] = float(np.mean(m - scale[axis] * e))

    return (
        {"x": float(offset[0]), "y": float(offset[1]), "z": float(offset[2])},
        {"x": float(scale[0]), "y": float(scale[1]), "z": float(scale[2])},
    )


def calibrate_mag_hard_iron(samples: list[dict[str, float]]) -> dict[str, float]:
    mx = [s["mx"] for s in samples]
    my = [s["my"] for s in samples]
    mz = [s["mz"] for s in samples]
    return {
        "x": (max(mx) + min(mx)) / 2,
        "y": (max(my) + min(my)) / 2,
        "z": (max(mz) + min(mz)) / 2,
    }


def calibrate_mag_ellipsoid(samples: list[dict[str, float]]) -> tuple[dict[str, float], list[list[float]]]:
    """
    Fit magnetometer hard-iron offset and diagonal soft-iron correction.

    Returns offset and 3x3 soft-iron matrix (diagonal).
    """
    if len(samples) < 30:
        raise ValueError("Need at least 30 magnetometer samples")

    points = np.array([_vec3(s, "m") for s in samples], dtype=float)
    offset_vec = np.array([
        (points[:, i].max() + points[:, i].min()) / 2 for i in range(3)
    ])

    centered = points - offset_vec
    soft = []
    for i in range(3):
        span = centered[:, i].max() - centered[:, i].min()
        soft.append(2.0 / span if span > 1e-6 else 1.0)

    matrix = np.diag(soft).tolist()
    return (
        {"x": float(offset_vec[0]), "y": float(offset_vec[1]), "z": float(offset_vec[2])},
        matrix,
    )


def accel_calibration_valid(cal: CalibrationData) -> bool:
    """Accel cal should have scale ≈ 1 and small offsets (m/s²). Reject stale LSB-era values."""
    for axis in ("x", "y", "z"):
        scale = cal.accel_scale[axis]
        offset = cal.accel_offset[axis]
        if not (0.5 <= abs(scale) <= 2.0):
            return False
        if abs(offset) > 15.0:
            return False
    return True


def apply_calibration(data: dict[str, float], cal: CalibrationData) -> dict[str, float]:
    if accel_calibration_valid(cal):
        ax = (data["ax"] - cal.accel_offset["x"]) / cal.accel_scale["x"]
        ay = (data["ay"] - cal.accel_offset["y"]) / cal.accel_scale["y"]
        az = (data["az"] - cal.accel_offset["z"]) / cal.accel_scale["z"]
    else:
        ax, ay, az = data["ax"], data["ay"], data["az"]

    gx = data["gx"] - cal.gyro_offset["x"]
    gy = data["gy"] - cal.gyro_offset["y"]
    gz = data["gz"] - cal.gyro_offset["z"]

    mag = _vec3(data, "m") - np.array([
        cal.mag_offset["x"], cal.mag_offset["y"], cal.mag_offset["z"]
    ])
    soft = np.array(cal.mag_soft_iron, dtype=float)
    mag_corrected = soft @ mag

    return {
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
        "mx": float(mag_corrected[0]),
        "my": float(mag_corrected[1]),
        "mz": float(mag_corrected[2]),
    }


def mag_quality_metrics(samples: list[dict[str, float]], cal: CalibrationData) -> dict[str, float]:
    if not samples:
        return {"span_x": 0, "span_y": 0, "span_z": 0, "radius_std": 0}

    corrected = np.array([
        [apply_calibration(s, cal)["mx"], apply_calibration(s, cal)["my"], apply_calibration(s, cal)["mz"]]
        for s in samples
    ])
    radii = np.linalg.norm(corrected, axis=1)
    return {
        "span_x": float(corrected[:, 0].max() - corrected[:, 0].min()),
        "span_y": float(corrected[:, 1].max() - corrected[:, 1].min()),
        "span_z": float(corrected[:, 2].max() - corrected[:, 2].min()),
        "radius_std": float(radii.std()),
        "radius_mean": float(radii.mean()),
    }


class CalibrationStore:
    def __init__(self, path: Path | None = None, model_label: str = IMU_MODEL_BMI160):
        self.model_label = model_label
        self.path = path or default_calibration_store_path(model_label)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.data = CalibrationData()

    def load(self) -> CalibrationData:
        if self.path.exists():
            with open(self.path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            self.data = extract_bmi160_calibration(payload)
        return self.data

    def save(self) -> None:
        self.data.saved_at = time.strftime("%Y-%m-%d %H:%M:%S")
        payload = build_bmi160_export_payload(self.data, self.model_label)
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
