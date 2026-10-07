# SPDX-License-Identifier: Apache-2.0
"""Planar rigid-body kinematics. Wheel order is FL, FR, RL, RR."""

import math

import numpy as np


class SwerveKinematics:
    def __init__(
        self, wheelbase, track_width, wheel_radius, steering_limit=math.pi / 2
    ):
        values = (wheelbase, track_width, wheel_radius, steering_limit)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Geometry and steering limit must be finite and positive")
        if not math.isclose(steering_limit, math.pi / 2, abs_tol=1e-9):
            raise ValueError("This release supports steering limits of +/- pi/2")
        self.radius = wheel_radius
        self.limit = steering_limit
        self.positions = (
            (wheelbase / 2, track_width / 2),
            (wheelbase / 2, -track_width / 2),
            (-wheelbase / 2, track_width / 2),
            (-wheelbase / 2, -track_width / 2),
        )
        matrix = []
        for x, y in self.positions:
            matrix.extend(((1.0, 0.0, -y), (0.0, 1.0, x)))
        self._inverse = np.linalg.pinv(np.array(matrix))

    def inverse(self, vx, vy, wz, current_angles):
        if len(current_angles) != 4 or not all(
            math.isfinite(v) for v in (vx, vy, wz, *current_angles)
        ):
            raise ValueError("Expected a finite twist and four steering angles")
        speeds, angles = [], []
        for (x, y), current in zip(self.positions, current_angles):
            ux, uy = vx - wz * y, vy + wz * x
            speed = math.hypot(ux, uy) / self.radius
            if speed < 1e-9:
                angles.append(max(-self.limit, min(self.limit, current)))
                speeds.append(0.0)
                continue
            angle = math.atan2(uy, ux)
            candidates = [
                (angle + k * math.pi, speed * (-1 if k % 2 else 1))
                for k in (-1, 0, 1)
                if -self.limit - 1e-9 <= angle + k * math.pi <= self.limit + 1e-9
            ]
            angle, speed = min(candidates, key=lambda pair: abs(pair[0] - current))
            angles.append(max(-self.limit, min(self.limit, angle)))
            speeds.append(speed)
        return speeds, angles

    def forward(self, wheel_speeds, steering_angles):
        if (
            len(wheel_speeds) != 4
            or len(steering_angles) != 4
            or not all(math.isfinite(v) for v in (*wheel_speeds, *steering_angles))
        ):
            raise ValueError("Expected four finite wheel speeds and steering angles")
        velocities = []
        for speed, angle in zip(wheel_speeds, steering_angles):
            velocities.extend(
                (
                    speed * self.radius * math.cos(angle),
                    speed * self.radius * math.sin(angle),
                )
            )
        return tuple(float(v) for v in self._inverse @ np.array(velocities))


def integrate_pose(pose, twist, dt):
    """Exact SE(2) integration for a constant body-frame twist."""
    x, y, yaw = pose
    vx, vy, wz = twist
    a = wz * dt
    if abs(wz) < 1e-9:
        dx, dy = vx * dt, vy * dt
    else:
        s, c = math.sin(a) / wz, (1.0 - math.cos(a)) / wz
        dx, dy = s * vx - c * vy, c * vx + s * vy
    return (
        x + math.cos(yaw) * dx - math.sin(yaw) * dy,
        y + math.sin(yaw) * dx + math.cos(yaw) * dy,
        math.atan2(math.sin(yaw + a), math.cos(yaw + a)),
    )
