"""3D IMU block orientation from accelerometer tilt (gravity vector)."""

from __future__ import annotations

import numpy as np
from matplotlib.axes import Axes
from mpl_toolkits.mplot3d import art3d

import viz_theme as theme
from axis_conventions import tilt_from_accel

__all__ = ["draw_orientation_block", "rotation_from_accel", "tilt_from_accel"]


def _cube_corners(hx: float, hy: float, hz: float) -> np.ndarray:
    return np.array([
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ])


def rotation_from_accel(ax: float, ay: float, az: float) -> np.ndarray:
    """
    Build rotation matrix (body → world) from gravity direction.

    At rest, body +Z aligns with the measured acceleration vector.
    Yaw is not observable from accel alone — stable but arbitrary around vertical.
    """
    mag = float(np.hypot(ax, np.hypot(ay, az)))
    if mag < 0.5:
        return np.eye(3)

    z = np.array([ax, ay, az], dtype=float) / mag
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z, ref))) > 0.92:
        ref = np.array([0.0, 1.0, 0.0])

    x = np.cross(ref, z)
    x_norm = float(np.linalg.norm(x))
    if x_norm < 1e-9:
        return np.eye(3)
    x /= x_norm
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def draw_orientation_block(
    ax: Axes,
    ax_m: float,
    ay_m: float,
    az_m: float,
    *,
    elev: float = 28.0,
    azim: float = -58.0,
    roll_deg: float | None = None,
    pitch_deg: float | None = None,
) -> None:
    """Draw a PX4-style IMU block oriented by the accelerometer reading."""
    ax.cla()
    ax.set_facecolor(theme.PANEL)

    R = rotation_from_accel(ax_m, ay_m, az_m)
    hx, hy, hz = 0.85, 0.62, 0.14
    corners = _cube_corners(hx, hy, hz) @ R.T

    # Reference floor grid (world horizontal plane)
    g = 1.5
    for i in np.linspace(-g, g, 7):
        ax.plot([i, i], [-g, g], [0, 0], color=theme.GRID, linewidth=0.5, alpha=0.7)
        ax.plot([-g, g], [i, i], [0, 0], color=theme.GRID, linewidth=0.5, alpha=0.7)

    faces_idx = [
        [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
        [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4],
    ]
    face_colors = ["#e5e7eb", "#e5e7eb", "#e5e7eb", "#e5e7eb", "#bfdbfe", "#bfdbfe"]
    board = art3d.Poly3DCollection(
        [[corners[i] for i in face] for face in faces_idx],
        facecolors=face_colors,
        edgecolors="#374151",
        linewidths=1.2,
        alpha=0.96,
    )
    ax.add_collection3d(board)

    center = corners.mean(axis=0)
    arrow_len = 0.75
    for label, col, idx in (
        ("X", theme.AXIS_X, 0),
        ("Y", theme.AXIS_Y, 1),
        ("Z", theme.AXIS_Z, 2),
    ):
        direction = R[:, idx] * arrow_len
        ax.quiver(
            center[0], center[1], center[2],
            direction[0], direction[1], direction[2],
            color=col, linewidth=1.8, arrow_length_ratio=0.2,
        )
        tip = center + direction * 1.12
        ax.text(tip[0], tip[1], tip[2], label, color=col, fontsize=9, fontweight="bold")

    # World gravity (down)
    ax.quiver(0, 0, 1.35, 0, 0, -0.55, color="#16a34a", linewidth=2.0, arrow_length_ratio=0.18)
    ax.text(0, 0, 1.48, "g", color="#16a34a", fontsize=9, fontweight="bold", ha="center")

    lim = 1.55
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-0.2, 1.65)
    if roll_deg is not None and pitch_deg is not None:
        title = f"Body orientation  ·  Roll {roll_deg:+.1f}°  ·  Pitch {pitch_deg:+.1f}°"
    else:
        title = "Body orientation (from accel)"
    ax.set_title(title, pad=8, fontsize=10, fontweight="bold", color=theme.TEXT)
    ax.set_xlabel("World X", fontsize=7, color=theme.MUTED, labelpad=2)
    ax.set_ylabel("World Y", fontsize=7, color=theme.MUTED, labelpad=2)
    ax.set_zlabel("World Z (up)", fontsize=7, color=theme.MUTED, labelpad=2)
    ax.tick_params(colors=theme.MUTED, labelsize=6)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor(theme.GRID)
    ax.yaxis.pane.set_edgecolor(theme.GRID)
    ax.zaxis.pane.set_edgecolor(theme.GRID)
    ax.grid(True, color=theme.GRID, alpha=0.35)
    ax.view_init(elev=elev, azim=azim)
    try:
        ax.set_box_aspect((1, 1, 0.85))
    except Exception:
        pass
