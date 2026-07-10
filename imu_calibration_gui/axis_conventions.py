"""Body-axis sign conventions used by the calibration GUI.

Board level (reference pose)
----------------------------
Place the IMU flat on the table with the component / silkscreen side up.
In this pose +Z should read about +9.8 m/s² on the accelerometer.

Rotation ↔ gyro (deg/s)
-------------------------
| Axis | Positive (+) motion      | Gyro channel |
|------|--------------------------|--------------|
| Y    | Roll right (right down)  | gy           |
| Z    | Pitch up (nose / front up) | gz       |
| X    | Yaw right (clockwise from above) | gx |

Rotation ↔ tilt (accel-derived angles, degrees)
-----------------------------------------------
| Angle | Positive (+) direction | Primary accel cue        |
|-------|------------------------|--------------------------|
| Roll  | Roll right             | ay increases             |
| Pitch | Pitch up               | ax decreases (nose lifts)|
| Yaw   | Not observable from accel alone — use gx while spinning flat |

Finding +X vs −X (six-face positions 3 & 4)
---------------------------------------------
Do not rely on silkscreen left/right until you have confirmed it once:

1. Level the board (+Z up) and confirm Y/Z signs with small roll-right and pitch-up moves.
2. Stand the board on one narrow edge until **X** is the dominant accelerometer axis.
3. If X reads **positive** (~+9.8 m/s²), that edge is **+X up** (position 3).
4. Flip to the opposite edge along the same axis for **−X up** (position 4).

Quick yaw check (board flat): rotate clockwise on the table; **gx > 0** means +X yaw right.
"""

from __future__ import annotations

import math


def tilt_from_accel(ax_m: float, ay_m: float, az_m: float) -> tuple[float, float]:
    """
    Roll and pitch in degrees from accelerometer tilt (gravity vector).

  Convention (level, +Z up):
    - Roll right  → positive roll  (Y+ / ay increases)
    - Pitch up    → positive pitch (Z+ pitch / nose lifts, ax decreases)
    """
    mag = float(math.hypot(ax_m, math.hypot(ay_m, az_m)))
    if mag < 0.5:
        return 0.0, 0.0
    roll = -math.degrees(math.atan2(ay_m, az_m))
    pitch = math.degrees(math.atan2(-ax_m, math.hypot(ay_m, az_m)))
    return roll, pitch


def yaw_rate_from_gyro(gx_dps: float) -> float:
    """Yaw rate in deg/s; +X = yaw right (clockwise from above)."""
    return gx_dps
