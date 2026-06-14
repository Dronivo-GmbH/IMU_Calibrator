"""Runtime signal filtering — separate from calibration (not persisted to JSON)."""

from __future__ import annotations

from dataclasses import dataclass

FILTERED_AXES = ("ax", "ay", "az", "gx", "gy", "gz")


@dataclass
class FilterConfig:
    """Runtime-only filter settings (never saved to calibration JSON)."""

    enabled: bool = True
    alpha: float = 0.1


EMA_PRESETS: tuple[tuple[str, float], ...] = (
    ("Smooth (0.05)", 0.05),
    ("Balanced (0.1)", 0.1),
    ("Responsive (0.2)", 0.2),
)


class RuntimeEMAFilter:
    """
    Exponential moving average on calibrated accel + gyro axes.

    filtered = alpha * current + (1 - alpha) * previous
    """

    def __init__(self, alpha: float = 0.1) -> None:
        self.alpha = alpha
        self._state: dict[str, float | None] = {k: None for k in FILTERED_AXES}

    def reset(self) -> None:
        for key in FILTERED_AXES:
            self._state[key] = None

    def configure(self, alpha: float) -> None:
        if alpha != self.alpha:
            self.alpha = alpha
            self.reset()

    def apply(self, corrected: dict[str, float]) -> dict[str, float]:
        out = dict(corrected)
        a = self.alpha
        for key in FILTERED_AXES:
            value = corrected[key]
            prev = self._state[key]
            if prev is None:
                smoothed = value
            else:
                smoothed = a * value + (1.0 - a) * prev
            self._state[key] = smoothed
            out[key] = smoothed
        return out

    @property
    def initialized(self) -> bool:
        return self._state["ax"] is not None


# Backward-compatible alias
AccelEMAFilter = RuntimeEMAFilter


def apply_runtime_filter(
    corrected: dict[str, float],
    filt: RuntimeEMAFilter,
    config: FilterConfig,
) -> dict[str, float]:
    if not config.enabled:
        return dict(corrected)
    return filt.apply(corrected)


def apply_accel_filter(corrected: dict[str, float], filt: RuntimeEMAFilter, config: FilterConfig) -> dict[str, float]:
    return apply_runtime_filter(corrected, filt, config)
