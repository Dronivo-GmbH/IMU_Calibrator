"""Live IMU visualizations — calibrated physical units."""

from __future__ import annotations

import math
import tkinter as tk
from collections import deque
from dataclasses import dataclass

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec

import viz_theme as theme
from calibration import GRAVITY
from orientation_block import draw_orientation_block

DEFAULT_ELEV = 28
DEFAULT_AZIM = -58


@dataclass
class _Sample:
    t: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


def _font(size: int, weight: str = "normal") -> dict:
    w = "bold" if weight in ("600", "bold") else weight
    return {"fontfamily": "sans-serif", "fontsize": size, "fontweight": w, "color": theme.TEXT}


class IMUVisualizerWidget(tk.Frame):
    """Rolling accel/gyro charts in calibrated physical units."""

    REFRESH_MS = 80
    IDLE_REFRESH_MS = 1000

    def __init__(self, parent: tk.Misc, window_seconds: float = 10.0, max_samples: int = 1200):
        super().__init__(parent, bg=theme.BG)
        self.window_seconds = window_seconds
        self.max_samples = max_samples
        self._samples: deque[_Sample] = deque(maxlen=max_samples)
        self._t0: float | None = None
        self._refresh_job: str | None = None
        self._root = parent.winfo_toplevel()
        self._initialized = False
        self._default_elev = DEFAULT_ELEV
        self._default_azim = DEFAULT_AZIM
        self._running = False
        self._dirty = False
        self._ref_g_line = None

        self._build_header()

        self.figure = Figure(figsize=(10.5, 6.8), dpi=100, facecolor=theme.BG)
        gs = GridSpec(
            2, 2, figure=self.figure,
            height_ratios=[1, 1],
            hspace=0.34, wspace=0.24,
            left=0.08, right=0.97, top=0.92, bottom=0.09,
        )

        self.ax_accel = self.figure.add_subplot(gs[0, 0])
        self.ax_gyro = self.figure.add_subplot(gs[0, 1])
        self.ax_orientation = self.figure.add_subplot(gs[1, 0], projection="3d")
        self.ax_mag = self.figure.add_subplot(gs[1, 1])

        self.plot_frame = tk.Frame(self, bg=theme.BG)
        self.plot_frame.pack(fill="both", expand=True, padx=4, pady=(0, 4))

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.plot_frame)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.configure(bg=theme.BG, highlightthickness=0)
        canvas_widget.pack(fill="both", expand=True)

        self._reset_3d_btn = tk.Button(
            self.plot_frame,
            text="↺",
            command=self._reset_3d_view,
            font=(theme.FONT, 11),
            width=2,
            relief="flat",
            bg=theme.PANEL,
            fg=theme.TEXT,
            activebackground=theme.GRID,
            highlightthickness=1,
            highlightbackground=theme.BORDER,
            cursor="hand2",
        )
        self._reset_3d_btn.place(relx=0.485, rely=0.505, anchor="ne")
        self._reset_3d_btn.lift()

        self._init_artists()
        self._draw_idle_state()
        self._schedule_refresh()

    def set_running(self, active: bool) -> None:
        self._running = active
        if active:
            self._dirty = True
        else:
            self._dirty = False

    def _build_header(self) -> None:
        header = tk.Frame(self, bg=theme.BG)
        header.pack(fill="x", padx=8, pady=(4, 2))

        tk.Label(
            header, text="Live Sensor Dashboard", bg=theme.BG, fg=theme.TEXT,
            font=(theme.FONT, 13, "bold"),
        ).pack(side="left")

        tk.Label(
            header, text="Calibrated + EMA filtered · m/s² · deg/s", bg=theme.BG, fg=theme.MUTED,
            font=(theme.FONT, 10),
        ).pack(side="left", padx=(12, 0))

        self.stats_var = tk.StringVar(value="Waiting for stream…")
        tk.Label(
            header, textvariable=self.stats_var, bg=theme.BG, fg=theme.MUTED,
            font=(theme.FONT, 10),
        ).pack(side="left", padx=(14, 0))

        btn_frame = tk.Frame(header, bg=theme.BG)
        btn_frame.pack(side="right")

        for label, seconds in (("5s", 5), ("10s", 10), ("20s", 20)):
            tk.Button(
                btn_frame, text=label, relief="flat", bg=theme.PANEL, fg=theme.MUTED,
                activebackground=theme.GRID, font=(theme.FONT, 9),
                padx=8, pady=2, cursor="hand2",
                command=lambda s=seconds: self._set_window(s),
            ).pack(side="left", padx=2)

        tk.Button(
            btn_frame, text="Clear", relief="flat", bg=theme.PANEL, fg=theme.TEXT,
            activebackground=theme.GRID, font=(theme.FONT, 9),
            padx=10, pady=2, cursor="hand2", command=self.clear,
        ).pack(side="left", padx=(6, 0))

    def _set_window(self, seconds: float) -> None:
        self.window_seconds = seconds
        self._dirty = True

    def _schedule_refresh(self) -> None:
        if self._refresh_job:
            self._root.after_cancel(self._refresh_job)
        self._refresh_loop()

    def _refresh_loop(self) -> None:
        if self._running and self._samples:
            self.refresh()

        delay = self.REFRESH_MS if (self._running and self._samples) else self.IDLE_REFRESH_MS
        self._refresh_job = self._root.after(delay, self._refresh_loop)

    def destroy(self) -> None:
        if self._refresh_job:
            self._root.after_cancel(self._refresh_job)
        super().destroy()

    def clear(self) -> None:
        self._samples.clear()
        self._t0 = None
        self._dirty = True

    def add_sample(
        self,
        timestamp: float,
        ax: float, ay: float, az: float,
        gx: float, gy: float, gz: float,
    ) -> None:
        if self._t0 is None:
            self._t0 = timestamp
        self._samples.append(_Sample(timestamp - self._t0, ax, ay, az, gx, gy, gz))
        self._dirty = True

    def _style_2d_axis(self, ax, title: str, ylabel: str) -> None:
        ax.set_facecolor(theme.PANEL)
        ax.set_title(title, pad=8, **_font(10, "bold"))
        ax.set_xlabel("Time (s)", labelpad=4, **_font(8))
        ax.set_ylabel(ylabel, labelpad=6, **_font(8))
        ax.tick_params(colors=theme.MUTED, labelsize=7, length=3, width=0.6)
        ax.grid(True, color=theme.GRID, linewidth=0.8, alpha=0.9)
        for spine in ax.spines.values():
            spine.set_color(theme.BORDER)
            spine.set_linewidth(0.8)

    def _init_artists(self) -> None:
        self._style_2d_axis(self.ax_accel, "Accelerometer", "Acceleration (m/s²)")
        self._style_2d_axis(self.ax_gyro, "Gyroscope", "Angular rate (deg/s)")
        self._style_2d_axis(self.ax_mag, "Signal magnitude", "Magnitude")

        self._accel_lines = [
            self.ax_accel.plot([], [], color=c, linewidth=1.8, solid_capstyle="round", label=l)[0]
            for c, l in zip(theme.CHANNEL_COLORS, ("X", "Y", "Z"))
        ]
        self._gyro_lines = [
            self.ax_gyro.plot([], [], color=c, linewidth=1.8, solid_capstyle="round", label=l)[0]
            for c, l in zip(theme.CHANNEL_COLORS, ("X", "Y", "Z"))
        ]
        self._mag_lines = [
            self.ax_mag.plot([], [], color=theme.ACCEL_MAG, linewidth=2.0, label="|a| (m/s²)", alpha=0.9)[0],
            self.ax_mag.plot([], [], color=theme.GYRO_MAG, linewidth=2.0, label="|ω| (deg/s)", alpha=0.9)[0],
        ]
        self._ref_g_line = self.ax_mag.axhline(
            GRAVITY, color=theme.MUTED, linewidth=1.0, linestyle="--", alpha=0.7, label="1 g",
        )
        self.ax_mag.axhline(0.0, color=theme.GRID, linewidth=0.8, alpha=0.8)

        for ax, lines in ((self.ax_accel, self._accel_lines), (self.ax_gyro, self._gyro_lines)):
            ax.legend(handles=lines, loc="upper right", frameon=True, framealpha=0.92,
                      edgecolor=theme.BORDER, fontsize=7, handlelength=1.6)
        self.ax_mag.legend(loc="upper right", frameon=True, framealpha=0.92,
                           edgecolor=theme.BORDER, fontsize=7)

        draw_orientation_block(
            self.ax_orientation, 0.0, 0.0, GRAVITY,
            elev=self._default_elev, azim=self._default_azim,
        )

        self._idle_text = self.figure.text(
            0.5, 0.5, "Connect serial port to begin streaming",
            ha="center", va="center", fontsize=11, color=theme.MUTED, alpha=0.85,
        )
        self._initialized = True

    def _reset_3d_view(self) -> None:
        if self._samples:
            last = self._samples[-1]
            draw_orientation_block(
                self.ax_orientation, last.ax, last.ay, last.az,
                elev=self._default_elev, azim=self._default_azim,
            )
        else:
            draw_orientation_block(
                self.ax_orientation, 0.0, 0.0, GRAVITY,
                elev=self._default_elev, azim=self._default_azim,
            )
        self._redraw_canvas()

    def _redraw_canvas(self) -> None:
        try:
            self.canvas.draw()
        except tk.TclError:
            pass

    def _update_orientation(self, ax: float, ay: float, az: float) -> None:
        elev = self.ax_orientation.elev
        azim = self.ax_orientation.azim
        draw_orientation_block(self.ax_orientation, ax, ay, az, elev=elev, azim=azim)

    def _draw_idle_state(self) -> None:
        if not self._initialized:
            return
        self._idle_text.set_alpha(0.85 if not self._samples else 0.0)

    @staticmethod
    def _smooth_ylim(data: np.ndarray, pad: float = 0.15, min_span: float = 1.0) -> tuple[float, float]:
        if len(data) == 0:
            return -min_span, min_span
        lo, hi = float(np.min(data)), float(np.max(data))
        if abs(hi - lo) < 1e-6:
            mid = lo
            return mid - min_span, mid + min_span
        margin = max((hi - lo) * pad, min_span * 0.1)
        return lo - margin, hi + margin

    def _update_2d_panel(self, ax, lines: list, t: np.ndarray,
                         series: tuple[np.ndarray, np.ndarray, np.ndarray],
                         min_span: float) -> None:
        for line, y in zip(lines, series):
            line.set_data(t, y)
        if len(t) >= 2:
            ax.set_xlim(float(t[0]), float(t[-1]))
        ymin, ymax = self._smooth_ylim(np.concatenate(series), min_span=min_span)
        ax.set_ylim(ymin, ymax)

    def refresh(self) -> None:
        if not self._initialized or not self._running or not self._samples:
            return

        self._idle_text.set_alpha(0.0)

        arr = np.array([(s.t, s.ax, s.ay, s.az, s.gx, s.gy, s.gz) for s in self._samples])
        t = arr[:, 0]
        t_min = max(0.0, float(t[-1]) - self.window_seconds)
        mask = t >= t_min
        t = t[mask]
        ax, ay, az = arr[mask, 1], arr[mask, 2], arr[mask, 3]
        gx, gy, gz = arr[mask, 4], arr[mask, 5], arr[mask, 6]

        self._update_2d_panel(self.ax_accel, self._accel_lines, t, (ax, ay, az), min_span=2.0)
        self._update_2d_panel(self.ax_gyro, self._gyro_lines, t, (gx, gy, gz), min_span=1.0)

        accel_mag = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
        gyro_mag = np.sqrt(gx ** 2 + gy ** 2 + gz ** 2)
        self._mag_lines[0].set_data(t, accel_mag)
        self._mag_lines[1].set_data(t, gyro_mag)
        if len(t) >= 2:
            self.ax_mag.set_xlim(float(t[0]), float(t[-1]))
        ymin = min(float(np.min(accel_mag)), float(np.min(gyro_mag)), 0.0)
        ymax = max(float(np.max(accel_mag)), float(np.max(gyro_mag)), GRAVITY * 1.2)
        pad = (ymax - ymin) * 0.12 if ymax > ymin else 1.0
        self.ax_mag.set_ylim(ymin - pad, ymax + pad)

        self._update_orientation(float(ax[-1]), float(ay[-1]), float(az[-1]))

        self.stats_var.set(
            f"{len(self._samples)} samples  ·  "
            f"a=[{ax[-1]:+.2f}, {ay[-1]:+.2f}, {az[-1]:+.2f}] m/s²  ·  "
            f"|ω| {gyro_mag[-1]:.2f} °/s  ·  {self.window_seconds:.0f}s"
        )

        self._redraw_canvas()
