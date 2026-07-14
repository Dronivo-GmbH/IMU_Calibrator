"""Serial connection and IMU data streaming."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Callable

import serial
import serial.tools.list_ports

from device_protocol import is_command_response

GRAVITY = 9.80665


@dataclass
class IMUSample:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    timestamp: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return {
            "ax": self.ax, "ay": self.ay, "az": self.az,
            "gx": self.gx, "gy": self.gy, "gz": self.gz,
            "mx": self.mx, "my": self.my, "mz": self.mz,
        }


def parse_line(line: str) -> dict[str, float] | None:
    """
    Supported serial formats:
      DFRobot / ESP32:  ax,ay,az,gx,gy,gz          (6 values, raw LSB)
      9-axis stream:    ax,ay,az,gx,gy,gz,mx,my,mz  (9 values)
      BNO055 plotter:   ax_g:... ay_g:... az_g:... gx_dps:... gy_dps:... gz_dps:... mx_uT:... my_uT:... mz_uT:...
    """
    plotter_sample = _parse_bno055_plotter_line(line)
    if plotter_sample:
        return plotter_sample

    try:
        parts = [p.strip() for p in line.split(",")]
        values = [float(p) for p in parts if p]
        if len(values) == 6:
            return {
                "ax": values[0], "ay": values[1], "az": values[2],
                "gx": values[3], "gy": values[4], "gz": values[5],
                "mx": 0.0, "my": 0.0, "mz": 0.0,
            }
        if len(values) == 9:
            return {
                "ax": values[0], "ay": values[1], "az": values[2],
                "gx": values[3], "gy": values[4], "gz": values[5],
                "mx": values[6], "my": values[7], "mz": values[8],
            }
        return None
    except ValueError:
        return None


def _parse_bno055_plotter_line(line: str) -> dict[str, float] | None:
    if ":" not in line:
        return None

    values: dict[str, float] = {}
    for token in line.replace("\t", " ").split():
        if ":" not in token:
            continue
        key, raw_value = token.split(":", 1)
        try:
            values[key.strip()] = float(raw_value.strip())
        except ValueError:
            return None

    required = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "mx_uT", "my_uT", "mz_uT")
    if not all(key in values for key in required):
        return None

    return {
        "ax": values["ax_g"] * GRAVITY,
        "ay": values["ay_g"] * GRAVITY,
        "az": values["az_g"] * GRAVITY,
        "gx": values["gx_dps"],
        "gy": values["gy_dps"],
        "gz": values["gz_dps"],
        "mx": values["mx_uT"],
        "my": values["my_uT"],
        "mz": values["mz_uT"],
    }


def list_serial_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


class SerialIMUClient:
    def __init__(
        self,
        on_sample: Callable[[IMUSample], None] | None = None,
        max_history: int = 5000,
    ):
        self._on_sample = on_sample
        self._ser: serial.Serial | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: IMUSample | None = None
        self._history: deque[IMUSample] = deque(maxlen=max_history)
        self._capture_buffer: list[IMUSample] | None = None
        self._sample_count = 0
        self._last_sample_time: float | None = None
        self._rate_hz = 0.0
        self._write_lock = threading.Lock()
        self._response_queue: Queue[str] = Queue()
        self._has_magnetometer = False
        self._stream_label = "unknown"
        self._reader_alive = False
        self._on_reader_stopped: Callable[[], None] | None = None

    @property
    def has_magnetometer(self) -> bool:
        return self._has_magnetometer

    @property
    def stream_label(self) -> str:
        return self._stream_label

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def latest(self) -> IMUSample | None:
        with self._lock:
            return self._latest

    @property
    def sample_rate_hz(self) -> float:
        return self._rate_hz

    @property
    def total_samples(self) -> int:
        return self._sample_count

    @property
    def reader_alive(self) -> bool:
        return self._reader_alive

    def set_on_reader_stopped(self, callback: Callable[[], None] | None) -> None:
        self._on_reader_stopped = callback

    def connect(self, port: str, baud: int = 115200, boot_delay_s: float = 2.5) -> None:
        if self.is_connected:
            self.disconnect()
        self._ser = serial.Serial(port, baud, timeout=1)
        self._has_magnetometer = False
        self._stream_label = "unknown"
        self._sample_count = 0
        self._rate_hz = 0.0
        self._last_sample_time = None
        self._running = False
        self._reader_alive = False
        # Opening the port resets many Arduino boards — wait for boot text to finish.
        if boot_delay_s > 0:
            time.sleep(boot_delay_s)
        self._ser.reset_input_buffer()
        while not self._response_queue.empty():
            try:
                self._response_queue.get_nowait()
            except Empty:
                break
        self._running = True
        self._reader_alive = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._running = False
        self._reader_alive = False
        if self._thread:
            self._thread.join(timeout=1.5)
            self._thread = None
        if self._ser:
            self._ser.close()
            self._ser = None

    @property
    def capture_count(self) -> int:
        with self._lock:
            return len(self._capture_buffer) if self._capture_buffer is not None else 0

    def start_capture(self) -> None:
        with self._lock:
            self._capture_buffer = []

    def stop_capture(self) -> list[dict[str, float]]:
        with self._lock:
            buffer = self._capture_buffer or []
            self._capture_buffer = None
            return [s.as_dict() for s in buffer]

    def get_recent_history(self, max_samples: int = 500) -> list[dict[str, float]]:
        with self._lock:
            items = list(self._history)[-max_samples:]
            return [s.as_dict() for s in items]

    def send_command(
        self,
        command: str,
        timeout: float = 3.0,
        expect_prefix: str | None = None,
    ) -> str | None:
        if not self.is_connected or not self._ser:
            raise RuntimeError("Serial port is not connected")

        if expect_prefix is None:
            cmd = command.strip().split()[0]
            if cmd == "PING":
                expect_prefix = "PONG"
            else:
                expect_prefix = cmd

        while not self._response_queue.empty():
            try:
                self._response_queue.get_nowait()
            except Empty:
                break

        with self._write_lock:
            self._ser.write(f"{command.strip()}\n".encode("utf-8"))
            self._ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self._response_queue.get(timeout=0.1)
            except Empty:
                continue
            if self._response_matches(line, expect_prefix):
                return line
        return None

    @staticmethod
    def _response_matches(line: str, expect_prefix: str) -> bool:
        if line == expect_prefix:
            return True
        return line.startswith(f"{expect_prefix} ") or line.startswith(f"{expect_prefix}{{")

    def write_command(self, command: str) -> None:
        if not self.is_connected or not self._ser:
            raise RuntimeError("Serial port is not connected")

        with self._write_lock:
            self._ser.write(f"{command.strip()}\n".encode("utf-8"))
            self._ser.flush()

    def ping_device(self) -> bool:
        try:
            return self.send_command("PING") == "PONG"
        except RuntimeError:
            return False

    def _reader_loop(self) -> None:
        try:
            while self._running and self._ser:
                try:
                    line = self._ser.readline().decode(errors="ignore").strip()
                    if not line or line.startswith("#") or line.startswith("Starting") or line.startswith("BMI160"):
                        continue
                    if line.startswith("ax,") or line == "Read error":
                        continue

                    if is_command_response(line):
                        self._response_queue.put(line)
                        continue

                    data = parse_line(line)
                    if not data:
                        continue

                    mag_active = any(abs(data[k]) > 1e-6 for k in ("mx", "my", "mz"))
                    if mag_active:
                        self._has_magnetometer = True
                        self._stream_label = "9-axis"
                    elif self._sample_count == 0:
                        self._stream_label = "6-axis (DFRobot)"

                    now = time.time()
                    sample = IMUSample(timestamp=now, **data)

                    with self._lock:
                        if self._last_sample_time:
                            dt = now - self._last_sample_time
                            if dt > 0:
                                instant_rate = 1.0 / dt
                                self._rate_hz = 0.9 * self._rate_hz + 0.1 * instant_rate if self._rate_hz else instant_rate
                        self._last_sample_time = now
                        self._latest = sample
                        self._history.append(sample)
                        self._sample_count += 1
                        if self._capture_buffer is not None:
                            self._capture_buffer.append(sample)

                    if self._on_sample:
                        self._on_sample(sample)
                except (serial.SerialException, OSError):
                    break
        finally:
            self._reader_alive = False
            if self._on_reader_stopped:
                self._on_reader_stopped()
