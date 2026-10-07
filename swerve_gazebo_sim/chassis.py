# SPDX-License-Identifier: Apache-2.0
"""Small ROS-independent chassis mechanics; no planner or transport sessions."""

import math
from dataclasses import dataclass
from enum import IntEnum

from .kinematics import SwerveKinematics


class Mode(IntEnum):
    DUAL_ACKERMANN = 0
    SPIN = 1
    CRAB = 2


class Phase(IntEnum):
    FAULT = 0
    BRAKING = 1
    ALIGNING = 2
    READY = 3


class Fault(IntEnum):
    NONE = 0
    CLOCK = 1
    FEEDBACK = 2
    COMMAND = 3
    TIMEOUT = 4
    TRANSITION = 5


@dataclass(frozen=True)
class Command:
    stamp: float
    mode: int
    velocity: tuple
    request_id: int = 0
    entry: tuple = (0.0, 0.0, 0.0)


def permitted(mode, velocity):
    if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
        return False
    vx, vy, wz = velocity
    if mode == Mode.DUAL_ACKERMANN:
        return vy == 0 and (vx != 0 or wz == 0)
    if mode == Mode.SPIN:
        return vx == 0 and vy == 0
    return mode == Mode.CRAB and wz == 0


def approach(current, target, delta):
    return [a + max(-delta, min(delta, b - a)) for a, b in zip(current, target)]


class Chassis:
    def __init__(self, configuration, geometry):
        self.c = configuration
        self.kinematics = SwerveKinematics(**geometry)
        self.actual_mode = self.requested_mode = Mode.DUAL_ACKERMANN
        self.phase, self.fault = Phase.READY, Fault.NONE
        self.request_id = 0
        self.entry = self.limited = (0.0, 0.0, 0.0)
        self.accepted_steering = self.alignment = [0.0] * 4
        self.angles, self.speeds = [0.0] * 4, [0.0] * 4
        self.measured_angles, self.measured_speeds = [0.0] * 4, [0.0] * 4
        self.feedback_stamp = self.feedback_wall = None
        self.command = None
        self.command_wall = self.last_time = None
        self.last_stamp = -1.0
        self.transition_start = self.aligned_since = None
        self.explicit_transition = False
        self.initialized = False

    @property
    def confirmed(self):
        return self.phase == Phase.READY and self.fault == Fault.NONE

    def stop(self, fault):
        self.phase, self.fault = Phase.FAULT, fault
        self.command = None
        self.limited = (0.0, 0.0, 0.0)
        self.speeds = [0.0] * 4
        self.angles = [
            max(-self.kinematics.limit, min(self.kinematics.limit, a))
            for a in self.measured_angles
        ]

    def clock_reset(self):
        self.stop(Fault.CLOCK)
        self.feedback_stamp = self.feedback_wall = self.last_time = None
        self.last_stamp = -1.0

    def observe(self, angles, speeds, stamp, wall):
        if (
            len(angles) != 4
            or len(speeds) != 4
            or not all(math.isfinite(v) for v in (*angles, *speeds, stamp, wall))
            or stamp < 0
            or any(abs(a) > self.kinematics.limit + 0.05 for a in angles)
        ):
            self.stop(Fault.FEEDBACK)
            return False
        if self.feedback_stamp is not None and stamp <= self.feedback_stamp:
            return False
        self.measured_angles, self.measured_speeds = list(angles), list(speeds)
        self.feedback_stamp, self.feedback_wall = stamp, wall
        if not self.initialized:
            self.angles = list(angles)
            self.phase, self.fault = Phase.READY, Fault.NONE
            self.initialized = True
        return True

    def bounded(self, velocity):
        vx, vy, wz = velocity
        scale = max(
            1.0,
            math.hypot(vx, vy) / self.c["max_linear_speed"],
            abs(wz) / self.c["max_angular_speed"],
        )
        return vx / scale, vy / scale, wz / scale

    def targets(self, velocity):
        speeds, angles = self.kinematics.inverse(*velocity, self.measured_angles)
        scale = max(1.0, max(map(abs, speeds)) / self.c["max_wheel_speed"])
        return [v / scale for v in speeds], angles

    def receive(self, command, now, wall):
        if not self.initialized:
            self.stop(Fault.FEEDBACK)
            return False
        if (
            not permitted(command.mode, command.velocity)
            or not permitted(command.mode, command.entry)
            or not math.isfinite(command.stamp)
            or command.stamp < self.last_stamp
            or command.stamp < 0
            or command.stamp > now + 0.05
            or now - command.stamp > self.c["cmd_timeout"]
            or command.request_id < self.request_id
        ):
            self.stop(Fault.COMMAND)
            return False
        if command.request_id > self.request_id:
            if any(command.velocity) or self.phase not in (Phase.READY, Phase.FAULT):
                self.stop(Fault.COMMAND)
                return False
            self.request_id, self.requested_mode = command.request_id, command.mode
            self.entry = command.entry
            magnitude = max(map(abs, command.entry))
            intent = (
                tuple(v / magnitude for v in command.entry)
                if magnitude
                else ((0.0, 0.0, 1.0) if command.mode == Mode.SPIN else (1.0, 0.0, 0.0))
            )
            self.alignment = self.targets(intent)[1]
            self.accepted_steering = list(self.alignment)
            self.explicit_transition = True
            self.begin(now)
            self.fault = Fault.NONE
        elif (
            command.mode != self.requested_mode
            or command.entry != self.entry
            or (
                self.explicit_transition
                and not self.confirmed
                and any(command.velocity)
            )
        ):
            self.stop(Fault.COMMAND)
            return False
        elif self.phase == Phase.FAULT:
            return False
        self.command, self.command_wall = command, wall
        self.last_stamp = command.stamp
        return True

    def begin(self, now):
        self.phase = Phase.BRAKING
        self.transition_start, self.aligned_since = now, None

    def stopped(self):
        return max(map(abs, self.measured_speeds)) <= self.c["stopped_wheel_speed"]

    def step(self, now, wall):
        dt = 0.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        if dt < 0 or dt > self.c["feedback_timeout"]:
            self.clock_reset()
        elif (
            self.feedback_stamp is None
            or now - self.feedback_stamp > self.c["feedback_timeout"]
            or self.feedback_stamp > now + 0.1
            or wall - self.feedback_wall > self.c["feedback_timeout"]
        ):
            self.stop(Fault.FEEDBACK)
        elif self.command is not None and (
            now - self.command.stamp > self.c["cmd_timeout"]
            or wall - self.command_wall > self.c["wall_timeout"]
        ):
            self.stop(Fault.TIMEOUT)
        if self.phase == Phase.FAULT or dt <= 0:
            return self.speeds, self.angles
        # Timer jitter must not produce a large actuator target jump.
        dt = min(dt, 2.0 / self.c["update_rate"])
        wheel_delta = self.c["max_wheel_acceleration"] * dt
        steering_delta = self.c["max_steering_rate"] * dt
        velocity = (
            self.bounded(self.command.velocity) if self.command else (0.0, 0.0, 0.0)
        )
        if self.phase == Phase.READY:
            if not any(velocity):
                self.speeds = approach(self.speeds, [0.0] * 4, wheel_delta)
                self.limited = (0.0, 0.0, 0.0)
                return self.speeds, self.angles
            difference = [b - a for a, b in zip(self.limited, velocity)]
            linear, angular = math.hypot(*difference[:2]), abs(difference[2])
            scale = min(
                1.0,
                self.c["max_linear_acceleration"] * dt / linear if linear else 1.0,
                self.c["max_angular_acceleration"] * dt / angular if angular else 1.0,
            )
            next_velocity = tuple(
                a + scale * d for a, d in zip(self.limited, difference)
            )
            speeds, angles = self.targets(next_velocity)
            if (
                max(abs(a - b) for a, b in zip(angles, self.measured_angles))
                > self.c["drive_steering_limit"]
            ):
                self.alignment = self.targets(velocity)[1]
                self.begin(now)
            else:
                self.speeds = approach(self.speeds, speeds, wheel_delta)
                self.angles = approach(self.angles, angles, steering_delta)
                self.limited = next_velocity
                return self.speeds, self.angles
        if now - self.transition_start > self.c["mode_switch_timeout"]:
            self.stop(Fault.TRANSITION)
            return self.speeds, self.angles
        self.speeds = approach(self.speeds, [0.0] * 4, wheel_delta)
        self.limited = (0.0, 0.0, 0.0)
        if not self.stopped() or any(self.speeds):
            self.phase, self.aligned_since = Phase.BRAKING, None
            return self.speeds, self.angles
        self.phase = Phase.ALIGNING
        self.angles = approach(self.angles, self.alignment, steering_delta)
        if (
            max(abs(a - b) for a, b in zip(self.alignment, self.measured_angles))
            <= self.c["steering_alignment_tolerance"]
        ):
            if self.aligned_since is None:
                self.aligned_since = now
            if now - self.aligned_since >= self.c["steering_alignment_duration"]:
                self.actual_mode, self.phase = self.requested_mode, Phase.READY
                self.explicit_transition = False
        else:
            self.aligned_since = None
        return self.speeds, self.angles
