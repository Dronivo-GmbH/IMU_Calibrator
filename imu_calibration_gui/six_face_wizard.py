"""Six-face accelerometer wizard — PX4-style 3D block orientation guide."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import art3d

import viz_theme as theme
from calibration import (
    ACCEL_FACES,
    CALIBRATION_CAPTURE_SECONDS,
    CALIBRATION_CAPTURE_TIMEOUT_SECONDS,
    CALIBRATION_MIN_SAMPLES,
    calibrate_accel_six_face,
)

# Rotation: body frame → world frame (columns = body axes in world coords)
_POSE_ROT: dict[str, np.ndarray] = {
    "flat_z_up": np.eye(3),
    "flat_z_down": np.diag([1.0, -1.0, -1.0]),
    "edge_x_up": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float),
    "edge_x_down": np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], dtype=float),
    "edge_y_up": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float),
    "edge_y_down": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float),
}

FACE_GUIDES = [
    {
        "name": "+Z up",
        "step": 1,
        "short": "+Z",
        "title": "Position 1 — Level (TOP side up)",
        "pose": "flat_z_up",
        "up_axis": np.array([0.0, 0.0, 1.0]),
        "steps": [
            "Place the IMU flat on the table — like a phone lying screen-up.",
            "The chip / component side must face the CEILING.",
            "The board must be level — not tilted forward or sideways.",
            "When correct, Z shows the largest positive value in live data.",
        ],
        "hint": "Most IMU breakouts: silkscreen on top = +Z facing up.",
    },
    {
        "name": "-Z up",
        "step": 2,
        "short": "-Z",
        "title": "Position 2 — Upside down (BOTTOM side up)",
        "pose": "flat_z_down",
        "up_axis": np.array([0.0, 0.0, -1.0]),
        "steps": [
            "Flip the board over from Position 1.",
            "The solder / bottom side now faces the CEILING.",
            "Keep the board flat and still on the table.",
            "When correct, Z shows the largest negative value.",
        ],
        "hint": "Simply rotate 180° around the long edge from Position 1.",
    },
    {
        "name": "+X up",
        "step": 3,
        "short": "+X",
        "title": "Position 3 — +X axis pointing up",
        "pose": "edge_x_up",
        "up_axis": np.array([1.0, 0.0, 0.0]),
        "steps": [
            "From level (+Z up), stand the board on one narrow edge.",
            "Only one edge touches the table — support it so it cannot slip.",
            "Watch live X: rotate between the two side edges until X is dominant and POSITIVE.",
            "The red +X arrow on the diagram should point straight UP toward the ceiling.",
        ],
        "hint": "+X up when X reads the largest positive value (~+9.8 m/s²). "
        "Yaw axis: spin flat clockwise → gx positive = +X yaw right.",
    },
    {
        "name": "-X up",
        "step": 4,
        "short": "-X",
        "title": "Position 4 — −X axis pointing up",
        "pose": "edge_x_down",
        "up_axis": np.array([-1.0, 0.0, 0.0]),
        "steps": [
            "Use the opposite edge along the same axis as Position 3.",
            "The −X direction points straight UP toward the ceiling.",
            "Hold steady — do not start capture until stable.",
            "When correct, X reads the largest negative value.",
        ],
        "hint": "Mirror of Position 3 on the same board axis (not the Y or Z edges).",
    },
    {
        "name": "+Y up",
        "step": 5,
        "short": "+Y",
        "title": "Position 5 — +Y axis pointing up (roll axis)",
        "pose": "edge_y_up",
        "up_axis": np.array([0.0, 1.0, 0.0]),
        "steps": [
            "Stand the board on the remaining edge (not used for X in steps 3–4).",
            "The green +Y arrow points straight UP.",
            "Roll right while level → Y (gy) goes positive; this is the roll axis.",
            "When correct, Y reads the largest positive value.",
        ],
        "hint": "Y+ = roll right. Pick the edge where Y is dominant and positive.",
    },
    {
        "name": "-Y up",
        "step": 6,
        "short": "-Y",
        "title": "Position 6 — −Y axis pointing up",
        "pose": "edge_y_down",
        "up_axis": np.array([0.0, -1.0, 0.0]),
        "steps": [
            "Use the opposite edge along the same axis as Position 5.",
            "The −Y direction points straight UP.",
            "This is the final position.",
            "When correct, Y reads the largest negative value.",
        ],
        "hint": "Y− = roll left. After capture, click Finish calibration.",
    },
]


def _cube_corners(hx: float, hy: float, hz: float) -> np.ndarray:
    return np.array([
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ])


def _draw_px4_block(ax, pose: str, *, compact: bool = False) -> None:
    """Draw IMU block on a grid floor — PX4-style 3D orientation view."""
    ax.cla()
    ax.set_facecolor("#ffffff")
    ax.set_axis_off()

    hx, hy, hz = (0.55, 0.38, 0.07) if compact else (1.0, 0.72, 0.12)
    R = _POSE_ROT[pose]

    corners = (_cube_corners(hx, hy, hz) @ R.T)
    corners[:, 2] -= corners[:, 2].min()  # sit on floor z=0

    # Grid floor
    g = 2.0 if compact else 2.8
    grid_n = 5 if compact else 7
    for i in np.linspace(-g, g, grid_n):
        ax.plot([i, i], [-g, g], [0, 0], color="#d1d5db", linewidth=0.4, alpha=0.8)
        ax.plot([-g, g], [i, i], [0, 0], color="#d1d5db", linewidth=0.4, alpha=0.8)

    faces_idx = [
        [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
        [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4],
    ]
    face_colors = ["#e5e7eb"] * 4 + ["#bfdbfe", "#bfdbfe"]
    board = art3d.Poly3DCollection(
        [[corners[i] for i in f] for f in faces_idx],
        facecolors=face_colors,
        edgecolors="#374151",
        linewidths=1.4 if not compact else 0.8,
        alpha=0.95,
    )
    ax.add_collection3d(board)

    center = corners.mean(axis=0)
    scale = 0.55 if compact else 0.95
    for label, col, idx in [("X", theme.AXIS_X, 0), ("Y", theme.AXIS_Y, 1), ("Z", theme.AXIS_Z, 2)]:
        direction = R[:, idx] * scale
        ax.quiver(
            center[0], center[1], center[2],
            direction[0], direction[1], direction[2],
            color=col, linewidth=1.8 if not compact else 1.0,
            arrow_length_ratio=0.18,
        )
        if not compact:
            tip = center + direction * 1.15
            ax.text(tip[0], tip[1], tip[2], label, color=col, fontsize=11, fontweight="bold")

    # Gravity arrow (world -Z)
    if not compact:
        ax.quiver(0, 0, 2.4, 0, 0, -0.7, color="#16a34a", linewidth=2.2, arrow_length_ratio=0.15)
        ax.text(0, 0, 2.55, "Gravity", color="#16a34a", fontsize=10, fontweight="bold", ha="center")

    lim = 1.6 if compact else 2.5
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(0, 2.8 if not compact else 1.8)
    ax.view_init(elev=28, azim=-52)
    try:
        ax.set_box_aspect((1, 1, 0.75))
    except Exception:
        pass


class SixFaceWizard:
    CAPTURE_TIMEOUT_SECONDS = CALIBRATION_CAPTURE_TIMEOUT_SECONDS
    CAPTURE_SECONDS = CALIBRATION_CAPTURE_SECONDS
    MIN_SAMPLES = CALIBRATION_MIN_SAMPLES

    STATUS_PENDING = "#e5e7eb"
    STATUS_ACTIVE = "#3b82f6"
    STATUS_DONE = "#22c55e"

    def __init__(
        self,
        root: tk.Tk,
        client,
        capture,
        on_complete: Callable[[list[list[dict]]], None],
        on_cancel: Callable[[], None] | None = None,
    ):
        self.root = root
        self.client = client
        self.capture = capture
        self.on_complete = on_complete
        self.on_cancel = on_cancel

        self._face_index = 0
        self._face_samples: list[list[dict]] = []
        self._face_done = [False] * 6
        self._capturing = False
        self._fullscreen = False
        self._live_job: str | None = None

        self.win = tk.Toplevel(root)
        self.win.title("Six-Face Accelerometer Calibration")
        screen_w = self.win.winfo_screenwidth()
        screen_h = self.win.winfo_screenheight()
        self.win.geometry(f"{min(1120, screen_w - 80)}x{min(780, screen_h - 80)}")
        self.win.minsize(900, 640)
        self.win.configure(bg="#ffffff")
        self.win.transient(root)
        self.win.grab_set()
        self.win.protocol("WM_DELETE_WINDOW", self._cancel)
        self.win.bind("<F11>", lambda _event: self._toggle_fullscreen())
        self.win.bind("<Escape>", lambda _event: self._exit_fullscreen())

        self._build_ui()
        self._show_face(0)
        self._start_live_update()
        self.win.after(0, self._maximize_window)

    def _build_ui(self) -> None:
        # Header
        header = tk.Frame(self.win, bg="#ffffff")
        header.pack(fill="x", padx=16, pady=(10, 6))

        title_row = tk.Frame(header, bg="#ffffff")
        title_row.pack(fill="x")
        tk.Label(title_row, text="Accelerometer Calibration", bg="#ffffff", fg=theme.TEXT,
                 font=(theme.FONT, 16, "bold")).pack(side="left")
        ttk.Button(title_row, text="Full screen", command=self._toggle_fullscreen).pack(side="right")
        self.subtitle_var = tk.StringVar()
        tk.Label(header, textvariable=self.subtitle_var, bg="#ffffff", fg=theme.MUTED,
                 font=(theme.FONT, 10)).pack(anchor="w", pady=(2, 0))

        tk.Label(
            header,
            text="Hold the IMU in each orientation shown below, then capture for 30 seconds. "
                 "Rotate through all 6 positions like PX4 sensor calibration.",
            bg="#ffffff", fg=theme.MUTED, font=(theme.FONT, 9), wraplength=920, justify="left",
        ).pack(anchor="w", pady=(4, 0))

        # Overview grid — all 6 positions (PX4-style)
        overview = tk.LabelFrame(self.win, text="  All 6 positions  ", bg="#ffffff", fg=theme.TEXT,
                                 font=(theme.FONT, 10, "bold"), padx=6, pady=6)
        overview.pack(fill="x", padx=16, pady=(4, 2))

        self._overview_fig = Figure(figsize=(7.0, 2.0), dpi=80, facecolor="#ffffff")
        self._overview_axes = []
        for i, guide in enumerate(FACE_GUIDES):
            ax = self._overview_fig.add_subplot(2, 3, i + 1, projection="3d")
            _draw_px4_block(ax, guide["pose"], compact=True)
            self._overview_axes.append(ax)

        self._overview_canvas = FigureCanvasTkAgg(self._overview_fig, master=overview)
        self._overview_canvas.get_tk_widget().pack()
        self._indicator_frames: list = []  # unused; kept for compat

        # Body: diagram + instructions
        body = tk.Frame(self.win, bg="#ffffff")
        body.pack(fill="both", expand=True, padx=16, pady=6)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        left = tk.LabelFrame(body, text="  Current orientation  ", bg="#ffffff", fg=theme.TEXT,
                             font=(theme.FONT, 10, "bold"))
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))

        self.main_fig = Figure(figsize=(5.5, 4.0), dpi=100, facecolor="#ffffff")
        self.main_ax = self.main_fig.add_subplot(111, projection="3d")
        self.main_canvas = FigureCanvasTkAgg(self.main_fig, master=left)
        self.main_canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)

        right = tk.Frame(body, bg="#ffffff")
        right.grid(row=0, column=1, sticky="nsew")

        tk.Label(right, text="What to do", bg="#ffffff", fg=theme.TEXT,
                 font=(theme.FONT, 12, "bold")).pack(anchor="w")

        steps_frame = tk.Frame(right, bg="#ffffff", highlightbackground=theme.BORDER,
                               highlightthickness=1)
        steps_frame.pack(fill="x", pady=(8, 10))

        self.steps_text = tk.Text(
            steps_frame, height=7, wrap="word", font=(theme.FONT, 10),
            bg="#ffffff", fg=theme.TEXT, relief="flat", padx=10, pady=8,
            highlightthickness=0, spacing1=4, spacing3=6,
        )
        self.steps_text.pack(fill="both", expand=True)
        self.steps_text.config(state="disabled")

        self.hint_var = tk.StringVar()
        tk.Label(right, textvariable=self.hint_var, bg="#f0fdf4", fg="#166534",
                 font=(theme.FONT, 10), wraplength=320, justify="left",
                 padx=10, pady=6).pack(fill="x", pady=(0, 8))

        tk.Label(right, text="Live accelerometer", bg="#ffffff", fg=theme.MUTED,
                 font=(theme.FONT, 9)).pack(anchor="w")
        self.live_var = tk.StringVar(value="Waiting for data…")
        tk.Label(right, textvariable=self.live_var, bg="#ffffff", fg=theme.TEXT,
                 font=(theme.FONT, 13, "bold"), padx=4, pady=6).pack(anchor="w")

        self.stability_var = tk.StringVar()
        tk.Label(right, textvariable=self.stability_var, bg="#ffffff", fg=theme.MUTED,
                 font=(theme.FONT, 10), wraplength=320, justify="left").pack(anchor="w", pady=(0, 8))

        self.progress_label = tk.StringVar()
        tk.Label(right, textvariable=self.progress_label, bg="#ffffff", fg=theme.TEXT,
                 font=(theme.FONT, 9)).pack(anchor="w")
        self.progress = ttk.Progressbar(
            right,
            mode="determinate",
            maximum=100,
            style="CalibrationIdle.Horizontal.TProgressbar",
        )
        self.progress.pack(fill="x", pady=4)

        self.status_var = tk.StringVar(value="Align the board, verify live values, then capture.")
        tk.Label(right, textvariable=self.status_var, bg="#ffffff", fg=theme.MUTED,
                 font=(theme.FONT, 9), wraplength=320, justify="left").pack(anchor="w", pady=(6, 0))

        # Footer
        footer = tk.Frame(self.win, bg="#ffffff")
        footer.pack(fill="x", padx=16, pady=(2, 10))
        ttk.Button(footer, text="Cancel", command=self._cancel).pack(side="left")
        self.capture_btn = ttk.Button(
            footer,
            text=f"▶  Start capture ({int(self.CAPTURE_SECONDS)} s)",
            command=self._start_capture,
        )
        self.capture_btn.pack(side="right", padx=(8, 0))
        self.next_btn = ttk.Button(footer, text="Next position →", command=self._next_face, state="disabled")
        self.next_btn.pack(side="right")

    def _update_overview_highlights(self) -> None:
        for i, ax in enumerate(self._overview_axes):
            if self._face_done[i]:
                color, suffix = "#16a34a", "✓ Done"
            elif i == self._face_index:
                color, suffix = "#2563eb", "● Current"
            else:
                color, suffix = "#9ca3af", "Pending"
            ax.set_title(
                f"{FACE_GUIDES[i]['short']}  —  {suffix}",
                color=color, fontsize=9, fontweight="bold", pad=3,
            )
        self._overview_fig.subplots_adjust(wspace=0.05, hspace=0.05)
        self._overview_canvas.draw_idle()

    def _set_steps(self, guide: dict) -> None:
        lines = "\n".join(f"{i + 1}. {s}" for i, s in enumerate(guide["steps"]))
        self.steps_text.config(state="normal")
        self.steps_text.delete("1.0", "end")
        self.steps_text.insert("1.0", lines)
        self.steps_text.config(state="disabled")
        self.hint_var.set(f"Tip: {guide['hint']}")

    def _show_face(self, index: int) -> None:
        guide = FACE_GUIDES[index]
        self._face_index = index
        self.subtitle_var.set(f"Step {guide['step']} of 6  ·  {guide['title']}")
        self._set_steps(guide)
        self._update_overview_highlights()

        _draw_px4_block(self.main_ax, guide["pose"], compact=False)
        self.main_canvas.draw()

        self.progress["value"] = 0
        self.progress.config(style="CalibrationIdle.Horizontal.TProgressbar")
        self.progress_label.set("")
        self.capture_btn.config(state="normal")
        self.next_btn.config(state="disabled", text="Next position →")
        self.status_var.set("Position the board as shown. Check live values, then capture.")

    def _maximize_window(self) -> None:
        try:
            self.win.state("zoomed")
        except tk.TclError:
            self.win.attributes("-zoomed", True)

    def _toggle_fullscreen(self) -> None:
        self._fullscreen = not self._fullscreen
        self.win.attributes("-fullscreen", self._fullscreen)

    def _exit_fullscreen(self) -> None:
        if not self._fullscreen:
            return
        self._fullscreen = False
        self.win.attributes("-fullscreen", False)

    def _start_live_update(self) -> None:
        self._update_live()
        self._live_job = self.win.after(200, self._start_live_update)

    def _update_live(self) -> None:
        sample = self.client.latest
        if not sample or self._capturing:
            return

        d = sample.as_dict()
        ax, ay, az = d["ax"], d["ay"], d["az"]
        self.live_var.set(f"X = {ax:,.0f}     Y = {ay:,.0f}     Z = {az:,.0f}")

        guide = FACE_GUIDES[self._face_index]
        up = guide["up_axis"]
        vals = np.array([ax, ay, az])
        dominant = int(np.argmax(np.abs(vals)))
        expected = int(np.argmax(np.abs(up)))
        axes = "XYZ"
        sign = "+" if up[expected] > 0 else "-"
        if dominant == expected and np.sign(vals[dominant]) == np.sign(up[expected]):
            self.stability_var.set(f"✓  Correct — {sign}{axes[expected]} is dominant. Ready to capture.")
        else:
            self.stability_var.set(
                f"⚠  Not aligned yet — rotate until {sign}{axes[expected]} "
                f"reads the largest value (see diagram)."
            )

    def _start_capture(self) -> None:
        if self._capturing:
            return
        self._capturing = True
        self.capture_btn.config(state="disabled")
        self.next_btn.config(state="disabled")
        self.progress.config(style="CalibrationRunning.Horizontal.TProgressbar")
        self.status_var.set(f"Hold completely still for {self.CAPTURE_SECONDS:.0f} s…")
        self.capture.start(
            self.CAPTURE_TIMEOUT_SECONDS,
            on_tick=self._on_tick,
            on_complete=self._on_capture_done,
            on_error=self._on_capture_error,
            stop_after_seconds=self.CAPTURE_SECONDS,
        )

    def _on_tick(
        self, remaining_s: float, count: int, capture_remaining_s: float | None = None,
    ) -> None:
        if capture_remaining_s is not None:
            elapsed = self.CAPTURE_SECONDS - capture_remaining_s
            pct = min(100.0, max(0.0, (elapsed / self.CAPTURE_SECONDS) * 100.0))
            time_left = capture_remaining_s
        else:
            pct = (1.0 - remaining_s / self.CAPTURE_TIMEOUT_SECONDS) * 100.0
            time_left = remaining_s
        self.progress["value"] = pct
        self.progress_label.set(
            f"Capturing… {count} samples  ·  {time_left:.1f}s left  ·  hold still"
        )

    def _on_capture_done(self, samples: list[dict]) -> None:
        self._capturing = False
        if len(samples) < self.MIN_SAMPLES:
            self.status_var.set(
                f"Too few samples ({len(samples)}, need ≥ {self.MIN_SAMPLES}) — "
                "check USB connection and try again."
            )
            self.capture_btn.config(state="normal")
            self.progress["value"] = 0
            self.progress.config(style="CalibrationIdle.Horizontal.TProgressbar")
            return

        self._face_samples.append(samples)
        self._face_done[self._face_index] = True
        self._update_overview_highlights()
        self.progress_label.set(
            f"✓ Position {self._face_index + 1} saved "
            f"({len(samples)} samples, {self.CAPTURE_SECONDS:.0f} s avg)"
        )

        if self._face_index >= 5:
            self.next_btn.config(text="✓  Finish calibration", state="normal")
            self.status_var.set("All positions captured! Click Finish calibration.")
        else:
            self.next_btn.config(state="normal")
            self.status_var.set("Position saved. Reposition for the next face, then click Next.")
        self.capture_btn.config(state="disabled")

    def _on_capture_error(self, msg: str) -> None:
        self._capturing = False
        self.status_var.set(msg)
        self.capture_btn.config(state="normal")
        self.progress.config(style="CalibrationIdle.Horizontal.TProgressbar")

    def _next_face(self) -> None:
        if self._face_index >= 5:
            self._finish()
            return
        self._show_face(self._face_index + 1)

    def _finish(self) -> None:
        name_to_idx = {name: i for i, (name, _) in enumerate(ACCEL_FACES)}
        ordered: list[list[dict] | None] = [None] * 6
        for samples, guide in zip(self._face_samples, FACE_GUIDES):
            ordered[name_to_idx[guide["name"]]] = samples
        if any(s is None for s in ordered):
            self.status_var.set("Missing position data.")
            return
        try:
            calibrate_accel_six_face(ordered)  # type: ignore[arg-type]
        except ValueError as exc:
            self.status_var.set(f"Calibration failed: {exc}")
            self.capture_btn.config(state="normal")
            return
        self._cleanup()
        self.on_complete(ordered)  # type: ignore[arg-type]
        self.win.destroy()

    def _cancel(self) -> None:
        self.capture.cancel()
        self._cleanup()
        if self.on_cancel:
            self.on_cancel()
        self.win.destroy()

    def _cleanup(self) -> None:
        if self._live_job:
            self.win.after_cancel(self._live_job)
            self._live_job = None


def open_six_face_wizard(root, client, capture, on_complete, on_cancel=None) -> SixFaceWizard:
    return SixFaceWizard(root, client, capture, on_complete, on_cancel)
