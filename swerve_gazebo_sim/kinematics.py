# SPDX-License-Identifier: Apache-2.0
"""Planar rigid-body kinematics. Wheel order is FL, FR, RL, RR."""

import math


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
        vx = vy = wz = 0.0
        denominator = sum(x * x + y * y for x, y in self.positions)
        for (x, y), speed, angle in zip(self.positions, wheel_speeds, steering_angles):
            ux = speed * self.radius * math.cos(angle)
            uy = speed * self.radius * math.sin(angle)
            vx += ux / 4
            vy += uy / 4
            wz += (-y * ux + x * uy) / denominator
        return vx, vy, wz


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
