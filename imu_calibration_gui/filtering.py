"""Runtime signal filtering — separate from calibration (not persisted to JSON)."""

from __future__ import annotations

import math
from dataclasses import dataclass

FILTERED_AXES = ("ax", "ay", "az", "gx", "gy", "gz")

# Nominal stream rate when sample timestamps are unavailable.
DEFAULT_SAMPLE_DT = 0.01


@dataclass
class FilterConfig:
    """Runtime-only filter settings (never saved to calibration JSON)."""

    enabled: bool = True
    cutoff_hz: float = 5.0


LPF_PRESETS: tuple[tuple[str, float], ...] = (
    ("Heavy (2 Hz)", 2.0),
    ("Normal (5 Hz)", 5.0),
    ("Light (12 Hz)", 12.0),
)


def alpha_from_cutoff(cutoff_hz: float, dt: float) -> float:
    """First-order low-pass coefficient for sample interval dt (seconds)."""
    if cutoff_hz <= 0.0:
        return 1.0
    dt = max(1e-4, min(dt, 0.25))
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (tau + dt)


class LowPassFilter:
    """
    First-order IIR low-pass on calibrated accel + gyro axes.

    y[n] = y[n-1] + α * (x[n] - y[n-1])
    α from cutoff frequency and sample interval.
    """

    def __init__(self, cutoff_hz: float = 5.0, default_dt: float = DEFAULT_SAMPLE_DT) -> None:
        self.cutoff_hz = cutoff_hz
        self.default_dt = default_dt
        self._state: dict[str, float | None] = {k: None for k in FILTERED_AXES}
        self._last_ts: float | None = None

    def reset(self) -> None:
        for key in FILTERED_AXES:
            self._state[key] = None
        self._last_ts = None

    def configure(self, cutoff_hz: float) -> None:
        if cutoff_hz != self.cutoff_hz:
            self.cutoff_hz = cutoff_hz
            self.reset()

    def _sample_dt(self, timestamp: float | None) -> float:
        if timestamp is not None and self._last_ts is not None:
            dt = timestamp - self._last_ts
            if 1e-4 <= dt <= 0.25:
                self._last_ts = timestamp
                return dt
        if timestamp is not None:
            self._last_ts = timestamp
        return self.default_dt

    def apply(self, corrected: dict[str, float], *, timestamp: float | None = None) -> dict[str, float]:
        alpha = alpha_from_cutoff(self.cutoff_hz, self._sample_dt(timestamp))
        out = dict(corrected)
        for key in FILTERED_AXES:
            value = corrected[key]
            prev = self._state[key]
            if prev is None:
                smoothed = value
            else:
                smoothed = prev + alpha * (value - prev)
            self._state[key] = smoothed
            out[key] = smoothed
        return out

    @property
    def initialized(self) -> bool:
        return self._state["ax"] is not None


def apply_runtime_filter(
    corrected: dict[str, float],
    filt: LowPassFilter,
    config: FilterConfig,
    *,
    timestamp: float | None = None,
) -> dict[str, float]:
    if not config.enabled:
        return dict(corrected)
    return filt.apply(corrected, timestamp=timestamp)
