# SPDX-License-Identifier: Apache-2.0
"""ROS-independent three-mode transition supervisor."""

from dataclasses import dataclass
from enum import Enum

from .kinematics import DriveMode, select_drive_mode


class TransitionPhase(Enum):
    IDLE = "idle"
    BRAKING = "braking"
    ALIGNING = "aligning"
    ACTIVE = "active"
    FAULT = "fault"


@dataclass
class MotionDecision:
    wheel_targets: list
    steering_targets: list
    drive_enabled: bool = False


class MotionSupervisor:
    """Gate mode entry with fresh measured wheel and steering feedback.

    A retargeted transition never preserves the old active mode. The latest mode
    must remain stable for a dwell interval, and all wheels must be measured stopped
    before steering is moved. A timeout latches a fault until a stop command.
    """

    def __init__(self, kinematics, configuration):
        self.kinematics = kinematics
        self.cfg = configuration
        self.reset()

    def reset(self):
        self.phase = TransitionPhase.IDLE
        self.requested_mode = None
        self.active_mode = None
        self.reason = "stopped"
        self.filtered_command = (0.0, 0.0, 0.0)
        self._component_active = [False] * 3
        self._candidate_since = None
        self._transition_since = None
        self._aligned_since = None
        self.max_steering_error = 0.0
        self.wheels_stopped = False

    def _filter_command(self, command):
        filtered = []
        for index, value in enumerate(command):
            kind = "angular" if index == 2 else "linear"
            boundary = "exit" if self._component_active[index] else "enter"
            threshold = self.cfg[f"mode_{kind}_{boundary}_threshold"]
            self._component_active[index] = abs(value) > threshold
            filtered.append(value if self._component_active[index] else 0.0)
        return tuple(filtered)

    def _begin_transition(self, now):
        self.active_mode = None
        self.phase = TransitionPhase.BRAKING
        self._aligned_since = None
        if self._transition_since is None:
            self._transition_since = now

    def _stop(self, reason):
        if self.phase is not TransitionPhase.FAULT:
            self.phase = TransitionPhase.IDLE
            self.reason = reason
            self._transition_since = None
        self.active_mode = None
        self._aligned_since = None
        return MotionDecision([0.0] * 4, [0.0] * 4)

    def update(
        self,
        command,
        angles,
        wheel_speeds,
        sent_angles,
        now,
        command_current,
        feedback_current,
    ):
        if not command_current:
            self.requested_mode = None
            self.filtered_command = (0.0, 0.0, 0.0)
            self._component_active = [False] * 3
            self._candidate_since = None
            return self._stop("command_timeout_or_invalid_header")

        self.filtered_command = self._filter_command(command)
        mode = select_drive_mode(*self.filtered_command)
        if mode is None:
            if not any(self.filtered_command):
                self.reset()
                return self._stop("stopped")
            self.requested_mode = None
            self._candidate_since = None
            return self._stop("unsupported_lateral_yaw")

        if mode is not self.requested_mode:
            self.requested_mode = mode
            self._candidate_since = now
            if self.phase is not TransitionPhase.FAULT:
                self._begin_transition(now)

        if not feedback_current:
            self.wheels_stopped = False
            self._candidate_since = now
            return self._stop("feedback_timeout")
        if self.phase is TransitionPhase.FAULT:
            return MotionDecision([0.0] * 4, list(sent_angles))

        _, wheel_targets, steering_targets = self.kinematics.commands_for_motion(
            *self.filtered_command, angles
        )
        self.wheels_stopped = all(
            abs(speed) <= self.cfg["stopped_wheel_speed"] for speed in wheel_speeds
        )

        if self.phase is TransitionPhase.ACTIVE:
            error = max(abs(a - b) for a, b in zip(steering_targets, angles))
            if (
                mode is DriveMode.DIFFERENTIAL
                or error <= self.cfg["steering_alignment_tolerance"]
            ):
                self.reason = ""
                self.max_steering_error = error
                return MotionDecision(wheel_targets, steering_targets, True)
            # Crab direction changes and lost spin/crab alignment stop drive too.
            self._begin_transition(now)

        if self.phase is TransitionPhase.IDLE:
            self._begin_transition(now)
        if now - self._transition_since >= self.cfg["mode_switch_timeout"]:
            self.phase = TransitionPhase.FAULT
            self.active_mode = None
            self.reason = "mode_switch_timeout_send_zero_to_reset"
            return MotionDecision([0.0] * 4, list(sent_angles))

        if self.phase is TransitionPhase.BRAKING:
            if not self.wheels_stopped:
                self.reason = "waiting_for_wheels_to_stop"
                return MotionDecision([0.0] * 4, list(sent_angles))
            if now - self._candidate_since < self.cfg["mode_dwell_time"]:
                self.reason = "waiting_for_stable_mode_request"
                return MotionDecision([0.0] * 4, list(sent_angles))
            self.phase = TransitionPhase.ALIGNING

        entry_targets = (
            [0.0] * 4 if mode is DriveMode.DIFFERENTIAL else steering_targets
        )
        self.max_steering_error = max(abs(a - b) for a, b in zip(entry_targets, angles))
        if self.max_steering_error <= self.cfg["steering_alignment_tolerance"]:
            if self._aligned_since is None:
                self._aligned_since = now
            if now - self._aligned_since >= self.cfg["steering_alignment_duration"]:
                self.phase = TransitionPhase.ACTIVE
                self.active_mode = mode
                self.reason = ""
                self._transition_since = None
                return MotionDecision(wheel_targets, entry_targets, True)
            self.reason = "waiting_for_stable_steering_alignment"
        else:
            self._aligned_since = None
            self.reason = "waiting_for_steering_alignment"
        return MotionDecision([0.0] * 4, entry_targets)
