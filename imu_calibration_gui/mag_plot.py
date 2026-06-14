"""Live 3D magnetometer scatter plot for calibration quality."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import viz_theme as theme


class MagPlotWidget(ttk.Frame):
    def __init__(self, parent: tk.Misc, max_points: int = 400):
        super().__init__(parent)
        self.max_points = max_points
        self._raw_points: list[tuple[float, float, float]] = []
        self._corr_points: list[tuple[float, float, float]] = []
        self._show_corrected = tk.BooleanVar(value=True)

        controls = ttk.Frame(self)
        controls.pack(fill="x", pady=(0, 6))

        ttk.Label(controls, text="Magnetometer 3D", font=(theme.FONT, 10, "bold")).pack(side="left")
        ttk.Checkbutton(
            controls,
            text="Show corrected",
            variable=self._show_corrected,
            command=self._redraw,
        ).pack(side="left", padx=(12, 0))
        ttk.Button(controls, text="Clear", command=self.clear).pack(side="right")

        self.stats_var = tk.StringVar(value="Points: 0")
        ttk.Label(controls, textvariable=self.stats_var, foreground=theme.MUTED).pack(side="right", padx=(0, 12))

        self.figure = Figure(figsize=(4.2, 3.4), dpi=100, facecolor=theme.BG)
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().configure(bg=theme.BG, highlightthickness=0)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._style_axes()
        self._redraw()

    def _style_axes(self) -> None:
        self.ax.set_facecolor(theme.PANEL)
        self.ax.set_xlabel("X (µT)", fontsize=8, color=theme.TEXT)
        self.ax.set_ylabel("Y (µT)", fontsize=8, color=theme.TEXT)
        self.ax.set_zlabel("Z (µT)", fontsize=8, color=theme.TEXT)
        self.ax.tick_params(labelsize=7, colors=theme.MUTED)
        self.ax.set_title("Mag field cloud", fontsize=9, color=theme.TEXT, pad=8)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.xaxis.pane.set_edgecolor(theme.GRID)
        self.ax.yaxis.pane.set_edgecolor(theme.GRID)
        self.ax.zaxis.pane.set_edgecolor(theme.GRID)
        self.ax.grid(True, color=theme.GRID, alpha=0.45)

    def clear(self) -> None:
        self._raw_points.clear()
        self._corr_points.clear()
        self._redraw()

    def add_sample(self, raw: tuple[float, float, float], corrected: tuple[float, float, float]) -> None:
        self._raw_points.append(raw)
        self._corr_points.append(corrected)
        if len(self._raw_points) > self.max_points:
            self._raw_points = self._raw_points[-self.max_points :]
            self._corr_points = self._corr_points[-self.max_points :]

    def _redraw(self) -> None:
        self.ax.cla()
        self._style_axes()

        points = self._corr_points if self._show_corrected.get() else self._raw_points
        label = "Corrected (µT)" if self._show_corrected.get() else "Raw (µT)"

        if points:
            arr = np.array(points)
            self.ax.scatter(
                arr[:, 0], arr[:, 1], arr[:, 2],
                c=theme.AXIS_Z, s=10, alpha=0.7, depthshade=True, label=label,
            )
            center = arr.mean(axis=0)
            radii = np.linalg.norm(arr - center, axis=1)
            self.stats_var.set(
                f"Points: {len(points)}  ·  radius μ={radii.mean():.1f} σ={radii.std():.2f} µT"
            )
            self._set_equal_axes(arr)
            self.ax.legend(loc="upper left", fontsize=7, framealpha=0.9, edgecolor=theme.BORDER)
        else:
            self.stats_var.set("Points: 0 — rotate IMU in figure-8 to collect data")
            self.ax.text2D(
                0.5, 0.5, "No magnetometer data",
                transform=self.ax.transAxes, ha="center", va="center",
                fontsize=9, color=theme.MUTED,
            )

        self.figure.tight_layout()
        self.canvas.draw_idle()

    def redraw(self) -> None:
        self._redraw()

    def _set_equal_axes(self, arr: np.ndarray) -> None:
        mins = arr.min(axis=0)
        maxs = arr.max(axis=0)
        centers = (mins + maxs) / 2
        span = max((maxs - mins).max() / 2, 5.0)
        self.ax.set_xlim(centers[0] - span, centers[0] + span)
        self.ax.set_ylim(centers[1] - span, centers[1] + span)
        self.ax.set_zlim(centers[2] - span, centers[2] + span)
