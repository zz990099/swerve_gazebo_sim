# SPDX-License-Identifier: Apache-2.0
"""Source-time planar transforms for optional ideal simulation localization."""

import math
from collections import deque


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (
        a[0] + c * b[0] - s * b[1],
        a[1] + s * b[0] + c * b[1],
        math.remainder(a[2] + b[2], 2 * math.pi),
    )


def inverse(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return (-c * p[0] - s * p[1], s * p[0] - c * p[1], -p[2])


class PoseHistory:
    """Interpolate bracketed wheel poses; never extrapolate across missing data."""

    def __init__(self, max_gap=0.1):
        self.samples = deque(maxlen=256)
        self.max_gap = max_gap

    def add(self, stamp, pose):
        if not all(math.isfinite(v) for v in (stamp, *pose)):
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        self.samples.append((stamp, tuple(pose)))
        return True

    def at(self, stamp):
        for t, p in self.samples:
            if abs(stamp - t) < 1e-8:
                return p
        for (ta, a), (tb, b) in zip(self.samples, list(self.samples)[1:]):
            if ta < stamp < tb and tb - ta <= self.max_gap:
                u = (stamp - ta) / (tb - ta)
                return (
                    a[0] + u * (b[0] - a[0]),
                    a[1] + u * (b[1] - a[1]),
                    a[2] + u * math.remainder(b[2] - a[2], 2 * math.pi),
                )
        return None


def map_to_odom(truth, wheel):
    return compose(truth, inverse(wheel))
