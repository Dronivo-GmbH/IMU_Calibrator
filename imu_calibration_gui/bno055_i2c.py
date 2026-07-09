"""Reusable BNO055 I2C calibration and orientation helper."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Protocol


class I2CBus(Protocol):
    def read_byte_data(self, addr: int, register: int) -> int: ...
    def write_byte_data(self, addr: int, register: int, value: int) -> None: ...
    def read_i2c_block_data(self, addr: int, register: int, length: int) -> list[int]: ...
    def write_i2c_block_data(self, addr: int, register: int, values: list[int]) -> None: ...


class BNO055Error(RuntimeError):
    """Raised when the BNO055 reports an error or behaves unexpectedly."""


@dataclass
class BNO055CalibrationStatus:
    sys: int
    gyr: int
    acc: int
    mag: int
    raw: int

    @property
    def is_fully_calibrated(self) -> bool:
        return self.sys == 3 and self.gyr == 3 and self.acc == 3 and self.mag == 3

    def to_dict(self) -> dict[str, int]:
        return {"sys": self.sys, "gyr": self.gyr, "acc": self.acc, "mag": self.mag, "raw": self.raw}


@dataclass
class BNO055CalibrationProfile:
    calibration_bytes: list[int]
    axis_map_config: int = 0x24
    axis_map_sign: int = 0x00
    address: int = 0x28
    saved_at: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "calibration_bytes": list(self.calibration_bytes),
            "axis_map_config": self.axis_map_config,
            "axis_map_sign": self.axis_map_sign,
            "address": self.address,
            "saved_at": self.saved_at,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> BNO055CalibrationProfile:
        values = [int(v) & 0xFF for v in data["calibration_bytes"]]
        if len(values) != 22:
            raise BNO055Error("BNO055 calibration profile must contain exactly 22 bytes")
        return cls(
            calibration_bytes=values,
            axis_map_config=int(data.get("axis_map_config", 0x24)) & 0xFF,
            axis_map_sign=int(data.get("axis_map_sign", 0x00)) & 0xFF,
            address=int(data.get("address", 0x28)) & 0xFF,
            saved_at=str(data.get("saved_at", "")),
        )


@dataclass
class BNO055Euler:
    heading: float
    roll: float
    pitch: float


@dataclass
class BNO055Quaternion:
    w: float
    x: float
    y: float
    z: float


SYS_ERR_MESSAGES = {
    0: "no error",
    1: "peripheral initialization error",
    2: "system initialization error",
    3: "self-test failed",
    4: "register map value out of range",
    5: "register map address out of range",
    6: "register map write error",
    7: "low power mode not available",
    8: "accelerometer power mode not available",
    9: "fusion algorithm configuration error",
    10: "sensor configuration error",
}


@dataclass
class BNO055:
    """Direct I2C helper for BNO055 setup, calibration, and orientation reads."""

    bus: I2CBus | int
    address: int | None = None
    axis_map_config: int = 0x24
    axis_map_sign: int = 0x00
    unit_sel: int = 0x80
    logger: Callable[[str], None] | None = print
    _owns_bus: bool = field(default=False, init=False, repr=False)
    _current_mode: int = field(default=0x00, init=False, repr=False)

    CHIP_ID_REG = 0x00
    PAGE_ID_REG = 0x07
    ACC_OFFSET_START = 0x55
    CALIB_STAT_REG = 0x35
    UNIT_SEL_REG = 0x3B
    OPR_MODE_REG = 0x3D
    PWR_MODE_REG = 0x3E
    SYS_STATUS_REG = 0x39
    SYS_ERR_REG = 0x3A
    AXIS_MAP_CONFIG_REG = 0x41
    AXIS_MAP_SIGN_REG = 0x42
    EULER_START = 0x1A
    QUAT_START = 0x20
    CHIP_ID = 0xA0
    CONFIG_MODE = 0x00
    NDOF_MODE = 0x0C
    NORMAL_POWER = 0x00
    PROFILE_LENGTH = 22
    VALID_ADDRESSES = (0x28, 0x29)

    def __post_init__(self) -> None:
        if isinstance(self.bus, int):
            try:
                from smbus2 import SMBus
            except ImportError as exc:
                raise BNO055Error("smbus2 is required when bus is provided as an integer") from exc
            self.bus = SMBus(self.bus)
            self._owns_bus = True

    def close(self) -> None:
        if self._owns_bus and hasattr(self.bus, "close"):
            self.bus.close()  # type: ignore[call-arg]

    def _log(self, message: str) -> None:
        if self.logger:
            self.logger(message)

    def begin(self) -> int:
        """Initialize the sensor and return the detected I2C address."""
        time.sleep(0.650)

        if self.address is None:
            detected = None
            for candidate in self.VALID_ADDRESSES:
                try:
                    chip_id = self._read_u8(candidate, self.CHIP_ID_REG)
                except OSError:
                    continue
                if chip_id == self.CHIP_ID:
                    detected = candidate
                    break
            if detected is None:
                raise BNO055Error("BNO055 not found on 0x28 or 0x29")
            self.address = detected
        else:
            chip_id = self._read_u8(self.address, self.CHIP_ID_REG)
            if chip_id != self.CHIP_ID:
                raise BNO055Error(
                    f"BNO055 CHIP_ID mismatch on 0x{self.address:02X}: expected 0xA0, got 0x{chip_id:02X}"
                )

        self._write_u8(self.PAGE_ID_REG, 0x00)
        self.set_mode(self.CONFIG_MODE)
        self._write_u8(self.PWR_MODE_REG, self.NORMAL_POWER)
        self._write_u8(self.UNIT_SEL_REG, self.unit_sel)
        self._write_u8(self.AXIS_MAP_CONFIG_REG, self.axis_map_config)
        self._write_u8(self.AXIS_MAP_SIGN_REG, self.axis_map_sign)
        self.set_mode(self.NDOF_MODE)
        self.check_system_status()
        return self.address

    def set_mode(self, mode: int) -> None:
        self._write_u8(self.PAGE_ID_REG, 0x00)
        self._write_u8(self.OPR_MODE_REG, mode & 0x0F)
        self._current_mode = mode & 0x0F
        time.sleep(0.019 if self._current_mode == self.CONFIG_MODE else 0.007)

    def get_calibration_status(self) -> BNO055CalibrationStatus:
        raw = self._read_u8(self.address, self.CALIB_STAT_REG)
        return BNO055CalibrationStatus(
            sys=(raw >> 6) & 0x03,
            gyr=(raw >> 4) & 0x03,
            acc=(raw >> 2) & 0x03,
            mag=raw & 0x03,
            raw=raw,
        )

    def wait_for_full_calibration(
        self,
        poll_interval: float = 0.5,
        timeout_s: float | None = None,
        status_callback: Callable[[BNO055CalibrationStatus], None] | None = None,
    ) -> BNO055CalibrationStatus:
        """Poll until SYS/GYR/ACC/MAG all reach 3, printing user guidance continuously."""
        start = time.monotonic()
        self._log("Gyroscope: keep device completely still until GYR reaches 3.")
        self._log("Accelerometer: move through 6 stable orientations and hold each for a few seconds.")
        self._log("Magnetometer: move in random 3D / figure-eight motion away from magnetic interference.")

        while True:
            self.check_system_status()
            status = self.get_calibration_status()
            if status_callback:
                status_callback(status)
            else:
                self._log(
                    f"CALIB SYS={status.sys} GYR={status.gyr} ACC={status.acc} MAG={status.mag}"
                )
            if status.is_fully_calibrated:
                return status
            if timeout_s is not None and (time.monotonic() - start) >= timeout_s:
                raise TimeoutError("Timed out waiting for full BNO055 calibration")
            time.sleep(poll_interval)

    def read_calibration_profile(self) -> BNO055CalibrationProfile:
        status = self.get_calibration_status()
        if not status.is_fully_calibrated:
            raise BNO055Error("Calibration profile may only be read after SYS/GYR/ACC/MAG all reach 3")

        previous_mode = self._current_mode
        self.set_mode(self.CONFIG_MODE)
        values = self._read_block(self.ACC_OFFSET_START, self.PROFILE_LENGTH)
        if len(values) != self.PROFILE_LENGTH:
            raise BNO055Error("Failed to read complete 22-byte BNO055 calibration profile")
        self.set_mode(previous_mode if previous_mode != self.CONFIG_MODE else self.NDOF_MODE)
        return BNO055CalibrationProfile(
            calibration_bytes=values,
            axis_map_config=self.axis_map_config,
            axis_map_sign=self.axis_map_sign,
            address=self.address or 0x28,
        )

    def write_calibration_profile(self, profile: BNO055CalibrationProfile) -> None:
        if len(profile.calibration_bytes) != self.PROFILE_LENGTH:
            raise BNO055Error("BNO055 calibration profile must contain 22 bytes")

        previous_mode = self._current_mode
        self.set_mode(self.CONFIG_MODE)
        self._write_u8(self.AXIS_MAP_CONFIG_REG, profile.axis_map_config)
        self._write_u8(self.AXIS_MAP_SIGN_REG, profile.axis_map_sign)
        self._write_block(self.ACC_OFFSET_START, profile.calibration_bytes)
        self.set_mode(self.NDOF_MODE if previous_mode == self.CONFIG_MODE else previous_mode)
        self.check_system_status()

    def save_profile(self, path: str | Path) -> Path:
        profile = self.read_calibration_profile()
        profile.saved_at = time.strftime("%Y-%m-%d %H:%M:%S")
        out_path = Path(path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as handle:
            json.dump(profile.to_dict(), handle, indent=2)
        return out_path

    def load_profile(self, path: str | Path) -> BNO055CalibrationProfile:
        with open(path, "r", encoding="utf-8") as handle:
            profile = BNO055CalibrationProfile.from_dict(json.load(handle))
        self.write_calibration_profile(profile)
        return profile

    def read_euler(self) -> BNO055Euler:
        data = self._read_block(self.EULER_START, 6)
        heading = self._int16(data[0], data[1]) / 16.0
        roll = self._int16(data[2], data[3]) / 16.0
        pitch = self._int16(data[4], data[5]) / 16.0
        return BNO055Euler(heading=heading, roll=roll, pitch=pitch)

    def read_quaternion(self) -> BNO055Quaternion:
        data = self._read_block(self.QUAT_START, 8)
        scale = 1.0 / (1 << 14)
        return BNO055Quaternion(
            w=self._int16(data[0], data[1]) * scale,
            x=self._int16(data[2], data[3]) * scale,
            y=self._int16(data[4], data[5]) * scale,
            z=self._int16(data[6], data[7]) * scale,
        )

    def read_sys_status(self) -> int:
        return self._read_u8(self.address, self.SYS_STATUS_REG)

    def read_sys_err(self) -> int:
        return self._read_u8(self.address, self.SYS_ERR_REG)

    def check_system_status(self) -> None:
        status = self.read_sys_status()
        if status == 1:
            err = self.read_sys_err()
            message = SYS_ERR_MESSAGES.get(err, f"unknown error {err}")
            raise BNO055Error(f"BNO055 system error {err}: {message}")

    def _read_u8(self, addr: int | None, register: int) -> int:
        if addr is None:
            raise BNO055Error("BNO055 address is not set")
        return int(self.bus.read_byte_data(addr, register)) & 0xFF

    def _write_u8(self, register: int, value: int) -> None:
        if self.address is None:
            raise BNO055Error("BNO055 address is not set")
        self.bus.write_byte_data(self.address, register, value & 0xFF)

    def _read_block(self, register: int, length: int) -> list[int]:
        if self.address is None:
            raise BNO055Error("BNO055 address is not set")
        values = self.bus.read_i2c_block_data(self.address, register, length)
        return [int(v) & 0xFF for v in values]

    def _write_block(self, register: int, values: list[int]) -> None:
        if self.address is None:
            raise BNO055Error("BNO055 address is not set")
        self.bus.write_i2c_block_data(self.address, register, [int(v) & 0xFF for v in values])

    @staticmethod
    def _int16(lsb: int, msb: int) -> int:
        value = (msb << 8) | lsb
        return value - 65536 if value & 0x8000 else value
