"""Exercise the production runtime with measured actuator feedback."""

import math
from pathlib import Path

import pytest

from swerve_gazebo_sim.bringup import load_config
from swerve_gazebo_sim.chassis import Chassis, Command, Fault, Phase

ROOT = Path(__file__).resolve().parents[1]
VELOCITY = ((0.25, 0.0, 0.05), (0.0, 0.0, 0.3), (0.0, 0.2, 0.0))
ZERO = (0.0, 0.0, 0.0)


class Rig:
    def __init__(self):
        cfg = load_config(ROOT / "config/swerve.yaml")
        geometry = {
            key: cfg["geometry"][key]
            for key in ("wheelbase", "track_width", "wheel_radius")
        }
        self.chassis = Chassis(cfg["control"], geometry)
        self.now = 0.0
        self.chassis.observe([0.0] * 4, [0.0] * 4, self.now, self.now)
        self.chassis.step(self.now, self.now)

    def tick(self, velocity=ZERO, feedback=True, publish=True):
        self.now += 0.01
        c = self.chassis
        if feedback:
            c.observe(c.angles, c.speeds, self.now, self.now)
        if publish:
            assert c.receive(
                Command(self.now, c.requested_mode, velocity, c.request_id, c.entry),
                self.now,
                self.now,
            )
        c.step(self.now, self.now)
        return c

    def switch(self, mode):
        c = self.chassis
        assert c.receive(
            Command(self.now, mode, ZERO, c.request_id + 1, VELOCITY[mode]),
            self.now,
            self.now,
        )
        receipt = (c.entry, tuple(c.accepted_steering))
        for _ in range(450):
            self.tick()
            assert c.fault == Fault.NONE
            assert (c.entry, tuple(c.accepted_steering)) == receipt
            if c.confirmed:
                assert c.actual_mode == mode
                return
        pytest.fail("Alignment did not complete")


@pytest.mark.parametrize(
    ("source", "target"), [(a, b) for a in range(3) for b in range(3) if a != b]
)
def test_all_six_mode_transitions_and_zero_hold(source, target):
    rig = Rig()
    rig.switch(source)
    for _ in range(150):
        rig.tick(VELOCITY[source])
    assert max(map(abs, rig.chassis.measured_speeds)) > 0.1
    rig.switch(target)
    for _ in range(150):
        rig.tick(VELOCITY[target])
    assert rig.chassis.kinematics.forward(
        rig.chassis.speeds, rig.chassis.angles
    ) == pytest.approx(VELOCITY[target])
    angles = tuple(rig.chassis.angles)
    for _ in range(80):
        rig.tick()
    assert rig.chassis.speeds == [0.0] * 4
    assert tuple(rig.chassis.angles) == angles
    assert rig.chassis.actual_mode == target


def test_target_limits_reverse_and_automatic_alignment():
    rig = Rig()
    rig.switch(2)
    for _ in range(200):
        rig.tick((0.0, -2.0, 0.0))
    c = rig.chassis
    assert c.confirmed
    assert c.kinematics.forward(c.speeds, c.angles) == pytest.approx((0.0, -0.8, 0.0))
    assert max(map(abs, c.speeds)) <= c.c["max_wheel_speed"]
    assert max(map(abs, c.angles)) <= math.pi / 2
    for _ in range(200):
        rig.tick((0.8, 0.0, 0.0))
    assert c.confirmed
    assert c.kinematics.forward(c.speeds, c.angles) == pytest.approx((0.8, 0.0, 0.0))


def test_braking_holds_steering_until_encoders_stop():
    rig = Rig()
    for _ in range(50):
        rig.tick(VELOCITY[0])
    c = rig.chassis
    angles = tuple(c.angles)
    assert c.receive(Command(rig.now, 1, ZERO, 1, VELOCITY[1]), rig.now, rig.now)
    rig.tick()
    assert c.phase == Phase.BRAKING
    assert tuple(c.angles) == angles
    assert max(map(abs, c.speeds)) < max(map(abs, c.measured_speeds))


def test_command_loss_latches_fault_and_new_zero_request_recovers():
    rig = Rig()
    for _ in range(50):
        rig.tick(VELOCITY[0])
    for _ in range(60):
        rig.tick(publish=False)
    c = rig.chassis
    assert c.fault == Fault.TIMEOUT
    assert c.speeds == [0.0] * 4
    assert not c.receive(Command(rig.now, 0, ZERO), rig.now, rig.now)
    rig.switch(0)
    assert c.confirmed


def test_paused_clock_does_not_disable_wall_watchdog():
    rig = Rig()
    rig.tick(VELOCITY[0])
    rig.chassis.step(rig.now, rig.now + 0.3)
    assert rig.chassis.fault == Fault.FEEDBACK
    assert rig.chassis.speeds == [0.0] * 4


@pytest.mark.parametrize(
    "command",
    [
        Command(0.0, 3, ZERO),
        Command(0.0, 0, (0.0, 0.0, 0.1)),
        Command(0.0, 1, (0.1, 0.0, 0.0)),
        Command(0.0, 2, (0.0, 0.0, 0.1)),
        Command(0.0, 0, (math.nan, 0.0, 0.0)),
        Command(1.0, 0, ZERO),
        Command(-1.0, 0, ZERO),
        Command(0.0, 1, VELOCITY[1], 1),
    ],
)
def test_invalid_commands_stop(command):
    rig = Rig()
    assert not rig.chassis.receive(command, 0.0, 0.0)
    assert rig.chassis.fault == Fault.COMMAND
    assert rig.chassis.speeds == [0.0] * 4


def test_transition_receipt_cannot_change_and_motion_requires_confirmation():
    rig = Rig()
    c = rig.chassis
    assert c.receive(Command(0.0, 1, ZERO, 1, VELOCITY[1]), 0.0, 0.0)
    assert not c.receive(Command(0.0, 1, VELOCITY[1], 1, VELOCITY[1]), 0.0, 0.0)
    rig.switch(1)
    assert not c.receive(
        Command(rig.now, 1, ZERO, c.request_id, ZERO), rig.now, rig.now
    )
    assert c.fault == Fault.COMMAND


def test_alignment_deadline_is_not_extended_by_retries():
    rig = Rig()
    c = rig.chassis
    assert c.receive(Command(0.0, 2, ZERO, 1, VELOCITY[2]), 0.0, 0.0)
    for _ in range(501):
        rig.now += 0.01
        c.observe([0.0] * 4, [0.0] * 4, rig.now, rig.now)
        assert c.receive(Command(rig.now, 2, ZERO, 1, VELOCITY[2]), rig.now, rig.now)
        c.step(rig.now, rig.now)
    assert c.fault == Fault.TRANSITION


def test_clock_rollback_requires_fresh_feedback_and_new_request():
    rig = Rig()
    rig.switch(1)
    highwater = rig.chassis.request_id
    rig.chassis.step(0.0, rig.now)
    assert rig.chassis.fault == Fault.CLOCK
    assert rig.chassis.feedback_stamp is None
    rig.chassis.observe([0.0] * 4, [0.0] * 4, 0.0, rig.now)
    assert not rig.chassis.receive(
        Command(0.0, 1, ZERO, highwater, VELOCITY[1]), 0.0, rig.now
    )
    assert rig.chassis.receive(Command(0.0, 0, ZERO, highwater + 1), 0.0, rig.now)


def test_missing_feedback_stops_and_invalid_observation_is_rejected():
    rig = Rig()
    c = rig.chassis
    for _ in range(26):
        rig.tick(feedback=False, publish=False)
    assert c.fault == Fault.FEEDBACK
    assert not c.observe([math.nan] * 4, [0.0] * 4, rig.now, rig.now)
    assert c.feedback_stamp == 0.0
