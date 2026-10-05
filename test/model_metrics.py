"""Compare installed-profile motion with independent, time-aligned plant truth."""

import bisect
import math


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def interpolate(history, stamp_ns, max_gap_ns=40_000_000):
    """Interpolate bounded truth intervals; never extrapolate or renew a stamp."""
    index = bisect.bisect_left([row[0] for row in history], stamp_ns)
    if index < len(history) and history[index][0] == stamp_ns:
        return history[index][1:]
    if index == 0 or index == len(history):
        return None
    before, after = history[index - 1], history[index]
    gap = after[0] - before[0]
    if gap <= 0 or gap > max_gap_ns:
        return None
    fraction = (stamp_ns - before[0]) / gap
    return (
        before[1] + fraction * (after[1] - before[1]),
        before[2] + fraction * (after[2] - before[2]),
        angle(before[3] + fraction * angle(after[3] - before[3])),
    )


def relative(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return c * dx + s * dy, -s * dx + c * dy, angle(end[2] - start[2])


class ModelMetrics:
    def __init__(self):
        self.pending = {}
        self.last_key = None
        self.errors = []
        self.moving = []
        self.unmatched = 0
        self.interrupted = 0

    def forecast(self, key, source_ns, end_ns, source_pose, predicted_pose):
        if self.last_key is not None and key <= self.last_key:
            return
        if not 0 < end_ns - source_ns <= 150_000_000:
            raise ValueError("invalid installed prediction interval")
        if not all(math.isfinite(v) for v in (*source_pose, *predicted_pose)):
            raise ValueError("nonfinite installed prediction")
        if self.last_key is not None and key[0] != self.last_key[0]:
            self.unmatched += len(self.pending)
            self.pending.clear()
        self.last_key = key
        self.pending[key] = (source_ns, end_ns, source_pose, predicted_pose)
        # A 300 s physical probe installs at most 3000 ten-Hz intervals.
        # Retain them until statistics run at a stopped boundary.
        if len(self.pending) > 4096:
            self.pending.pop(next(iter(self.pending)))
            self.unmatched += 1

    def interrupt(self, session, stamp_ns):
        """A latched stop truncates a profile; it is not model error over its full span."""
        for key, (start, end, _, _) in list(self.pending.items()):
            if key[0] == session and start <= stamp_ns < end:
                del self.pending[key]
                self.interrupted += 1

    def collect(self, history):
        if not history:
            return
        for key, (start_ns, end_ns, source, predicted) in list(self.pending.items()):
            if history[-1][0] < end_ns:
                continue
            start, end = interpolate(history, start_ns), interpolate(history, end_ns)
            del self.pending[key]
            if start is None or end is None:
                self.unmatched += 1
                continue
            # Each displacement uses its own source body frame, removing accumulated
            # encoder/world origin and yaw drift from the one-interval comparison.
            model, actual = relative(source, predicted), relative(start, end)
            error = (
                math.hypot(model[0] - actual[0], model[1] - actual[1]),
                abs(angle(model[2] - actual[2])),
            )
            self.errors.append(error)
            if (
                math.hypot(model[0], model[1]) > 0.001
                or abs(model[2]) > 0.001
                or math.hypot(actual[0], actual[1]) > 0.001
                or abs(actual[2]) > 0.001
            ):
                self.moving.append(error)

    def summary(self):
        def statistics(values):
            if not values:
                return {"rms": None, "p95": None, "max": None}
            ordered = sorted(values)
            return {
                "rms": math.sqrt(sum(v * v for v in values) / len(values)),
                "p95": ordered[math.ceil(0.95 * len(values)) - 1],
                "max": ordered[-1],
            }

        return {
            "matched_intervals": len(self.errors),
            "moving_intervals": len(self.moving),
            "unmatched_intervals": self.unmatched,
            "interrupted_intervals": self.interrupted,
            "pending_intervals": len(self.pending),
            "moving_translation_error_m": statistics([e[0] for e in self.moving]),
            "moving_yaw_error_rad": statistics([e[1] for e in self.moving]),
        }
