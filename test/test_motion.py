"""Deterministic transition tests without ROS or wall-clock timing."""

from pathlib import Path

import pytest
from swerve_gazebo_sim.bringup import load_config, validate_control
from swerve_gazebo_sim.kinematics import DriveMode, SwerveKinematics
from swerve_gazebo_sim.motion import MotionSupervisor, TransitionPhase

ROOT = Path(__file__).resolve().parents[1]
DIFFERENTIAL = (0.3, 0.0, 0.0)
CRAB = (0.3, 0.3, 0.0)
SPIN = (0.0, 0.0, 0.5)


@pytest.fixture
def supervisor():
    cfg = load_config(ROOT / "config/swerve.yaml")["control"]
    return MotionSupervisor(SwerveKinematics(0.6, 0.5, 0.1), cfg)


def step(supervisor, command, now, angles=None, speeds=None, **kwargs):
    return supervisor.update(
        command,
        [0.0] * 4 if angles is None else angles,
        [0.0] * 4 if speeds is None else speeds,
        kwargs.pop("sent_angles", [0.0] * 4 if angles is None else angles),
        now,
        kwargs.pop("command_current", True),
        kwargs.pop("feedback_current", True),
    )


def activate(supervisor, command, now=0.0):
    step(supervisor, command, now)
    decision = step(supervisor, command, now + 0.11)
    angles = decision.steering_targets
    step(supervisor, command, now + 0.12, angles=angles)
    decision = step(supervisor, command, now + 0.18, angles=angles)
    assert decision.drive_enabled
    return angles


def test_startup_requires_dwell_and_stable_measured_alignment(supervisor):
    assert not step(supervisor, DIFFERENTIAL, 0.0).drive_enabled
    assert supervisor.phase is TransitionPhase.BRAKING
    assert not step(supervisor, DIFFERENTIAL, 0.11).drive_enabled
    assert supervisor.phase is TransitionPhase.ALIGNING
    assert step(supervisor, DIFFERENTIAL, 0.17).drive_enabled
    assert supervisor.active_mode is DriveMode.DIFFERENTIAL


def test_steering_is_held_until_measured_wheels_stop(supervisor):
    held = [0.2] * 4
    decision = step(supervisor, CRAB, 0.0, speeds=[1.0] * 4, sent_angles=held)
    assert decision.steering_targets == held
    decision = step(
        supervisor, CRAB, 0.2, speeds=[0.0, 0.0, 0.0, 0.1], sent_angles=held
    )
    assert decision.steering_targets == held
    assert supervisor.reason == "waiting_for_wheels_to_stop"
    decision = step(supervisor, CRAB, 0.21)
    assert decision.steering_targets != held
    assert not decision.drive_enabled


@pytest.mark.parametrize("command", [CRAB, SPIN])
def test_interrupted_return_requires_new_dwell_and_home_alignment(supervisor, command):
    activate(supervisor, DIFFERENTIAL)
    step(supervisor, command, 0.2)
    step(supervisor, command, 0.32)
    assert not step(supervisor, DIFFERENTIAL, 0.33, angles=[0.25] * 4).drive_enabled
    assert supervisor.active_mode is None
    assert not step(supervisor, DIFFERENTIAL, 0.44, angles=[0.25] * 4).drive_enabled
    assert not step(supervisor, DIFFERENTIAL, 0.45).drive_enabled
    assert step(supervisor, DIFFERENTIAL, 0.51).drive_enabled


@pytest.mark.parametrize("first", [DIFFERENTIAL, CRAB, SPIN])
@pytest.mark.parametrize("second", [DIFFERENTIAL, CRAB, SPIN])
def test_every_direct_mode_change(supervisor, first, second):
    angles = activate(supervisor, first)
    decision = step(supervisor, second, 0.2, angles=angles)
    if first == second:
        assert decision.drive_enabled
        return
    assert not decision.drive_enabled
    assert supervisor.active_mode is None
    decision = step(supervisor, second, 0.32, angles=angles)
    step(supervisor, second, 0.33, angles=decision.steering_targets)
    assert step(
        supervisor, second, 0.39, angles=decision.steering_targets
    ).drive_enabled


def test_third_mode_retargets_without_unlocking_drive(supervisor):
    activate(supervisor, DIFFERENTIAL)
    step(supervisor, CRAB, 0.2)
    decision = step(supervisor, SPIN, 0.25)
    assert not decision.drive_enabled
    assert supervisor.requested_mode is DriveMode.SPIN
    assert not step(supervisor, SPIN, 0.31).drive_enabled
    decision = step(supervisor, SPIN, 0.36)
    step(supervisor, SPIN, 0.37, angles=decision.steering_targets)
    assert step(supervisor, SPIN, 0.43, angles=decision.steering_targets).drive_enabled


def test_deadband_and_hysteresis(supervisor):
    step(supervisor, (0.009, 0.0, 0.0), 0.0)
    assert supervisor.requested_mode is None
    step(supervisor, (0.011, 0.0, 0.0), 0.1)
    assert supervisor.requested_mode is DriveMode.DIFFERENTIAL
    step(supervisor, (0.007, 0.0, 0.0), 0.2)
    assert supervisor.requested_mode is DriveMode.DIFFERENTIAL
    step(supervisor, (0.004, 0.0, 0.0), 0.3)
    assert supervisor.requested_mode is None
    step(supervisor, (0.007, 0.0, 0.0), 0.4)
    assert supervisor.requested_mode is None


def test_yaw_noise_does_not_turn_crab_into_invalid_command(supervisor):
    activate(supervisor, CRAB)
    decision = step(supervisor, (0.3, 0.3, 0.009), 0.2, angles=[0.785398163] * 4)
    assert decision.drive_enabled
    assert supervisor.requested_mode is DriveMode.CRAB


def test_active_differential_keeps_driving_with_steering_tracking_error(supervisor):
    activate(supervisor, DIFFERENTIAL)
    decision = step(supervisor, (0.3, 0.0, 0.2), 0.2, angles=[0.4] * 4)
    assert decision.drive_enabled
    assert supervisor.active_mode is DriveMode.DIFFERENTIAL


def test_active_crab_direction_change_stops_before_steering(supervisor):
    angles = activate(supervisor, CRAB)
    decision = step(supervisor, (0.3, -0.3, 0.0), 0.2, angles=angles, speeds=[3.0] * 4)
    assert not decision.drive_enabled
    assert decision.steering_targets == angles
    assert supervisor.phase is TransitionPhase.BRAKING


@pytest.mark.parametrize("condition", ["command_current", "feedback_current"])
def test_watchdogs_stop_and_invalidate_active_mode(supervisor, condition):
    activate(supervisor, DIFFERENTIAL)
    decision = step(supervisor, DIFFERENTIAL, 0.2, **{condition: False})
    assert not decision.drive_enabled
    assert decision.wheel_targets == [0.0] * 4
    assert supervisor.active_mode is None
    assert not step(supervisor, DIFFERENTIAL, 0.21).drive_enabled


def test_timeout_latches_until_zero_command(supervisor):
    step(supervisor, SPIN, 0.0)
    assert not step(supervisor, SPIN, 5.1).drive_enabled
    assert supervisor.phase is TransitionPhase.FAULT
    assert not step(supervisor, DIFFERENTIAL, 5.2).drive_enabled
    assert supervisor.phase is TransitionPhase.FAULT
    step(supervisor, (0.0, 0.0, 0.0), 5.3)
    assert supervisor.phase is TransitionPhase.IDLE
    activate(supervisor, DIFFERENTIAL, 5.4)


def test_retargeting_does_not_restart_overall_timeout(supervisor):
    step(supervisor, SPIN, 0.0)
    step(supervisor, CRAB, 2.0)
    step(supervisor, DIFFERENTIAL, 4.0, angles=[0.4] * 4)
    step(supervisor, SPIN, 5.1)
    assert supervisor.phase is TransitionPhase.FAULT


def test_alignment_must_remain_stable(supervisor):
    step(supervisor, DIFFERENTIAL, 0.0)
    step(supervisor, DIFFERENTIAL, 0.11)
    step(supervisor, DIFFERENTIAL, 0.14, angles=[0.1] * 4)
    assert not step(supervisor, DIFFERENTIAL, 0.17).drive_enabled
    assert step(supervisor, DIFFERENTIAL, 0.23).drive_enabled


def test_invalid_lateral_yaw_stops(supervisor):
    activate(supervisor, DIFFERENTIAL)
    assert not step(supervisor, (0.3, 0.2, 0.2), 0.2).drive_enabled
    assert supervisor.reason == "unsupported_lateral_yaw"


def test_reset_discards_old_transition_timestamps(supervisor):
    step(supervisor, CRAB, 100.0)
    supervisor.reset()
    activate(supervisor, DIFFERENTIAL, 0.0)


@pytest.mark.parametrize("kind", ["linear", "angular"])
def test_invalid_hysteresis_configuration(supervisor, kind):
    supervisor.cfg[f"mode_{kind}_exit_threshold"] = supervisor.cfg[
        f"mode_{kind}_enter_threshold"
    ]
    with pytest.raises(ValueError, match="exit threshold"):
        validate_control(supervisor.cfg)
