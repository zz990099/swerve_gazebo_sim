import math

import pytest
import rclpy
import swerve_gazebo_sim.controller as controller_module
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from swerve_gazebo_sim.controller import SwerveController
from swerve_gazebo_sim.kinematics import DriveMode
from swerve_gazebo_sim.motion import TransitionPhase


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    result = SwerveController(
        parameter_overrides=[
            Parameter("publish_odom_tf", value=False),
            Parameter("mode_dwell_time", value=0.0),
            Parameter("steering_alignment_duration", value=0.0),
        ]
    )
    result.wheel_pub = Capture()
    result.steer_pub = Capture()
    result.odom_pub = Capture()
    result.status_pub = Capture()
    yield result
    result.destroy_node()
    rclpy.shutdown()


def feedback(node, steering_angles=None, wheel_speeds=None):
    if steering_angles is None:
        steering_angles = [0.0] * 4
    if wheel_speeds is None:
        wheel_speeds = [0.0] * 4

    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = node.wheel_joint_names + node.steering_joint_names
    msg.position = [0.0] * 4 + list(steering_angles)
    msg.velocity = list(wheel_speeds) + [0.0] * 4
    node.on_feedback(msg)
    return msg


@pytest.fixture(params=[False, True], ids=["internal", "external"])
def clocked_node(request, monkeypatch):
    class Clock:
        nanoseconds = 1_800_000_001_200_000_000

        def now(self):
            return Time(nanoseconds=self.nanoseconds)

    transforms = []

    class Broadcaster:
        def __init__(self, node):
            del node

        def sendTransform(self, transform):
            transforms.append(transform)

    monkeypatch.setattr(controller_module, "TransformBroadcaster", Broadcaster)
    rclpy.init()
    node = SwerveController(
        parameter_overrides=[
            Parameter("external_joint_control", value=request.param),
        ]
    )
    clock = Clock()
    monkeypatch.setattr(node, "get_clock", lambda: clock)
    node._last_clock_ns = clock.nanoseconds
    node.last_tick = clock.nanoseconds * 1e-9
    node.wheel_pub = Capture()
    node.steer_pub = Capture()
    node.odom_pub = Capture()
    node.status_pub = Capture()
    try:
        yield node, clock, transforms
    finally:
        node.destroy_node()
        rclpy.shutdown()


def stamped_feedback(node, stamp_ns, wheel_speed=2.0, angle=0.0):
    msg = JointState()
    msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    msg.name = node.wheel_joint_names + node.steering_joint_names
    msg.position = [0.0] * 4 + [angle] * 4
    msg.velocity = [wheel_speed] * 4 + [0.0] * 4
    node.on_feedback(msg)


@pytest.mark.parametrize("backward_ns", [0, 1, 10_000_000])
def test_old_feedback_does_not_reset_or_replace_current_state(
    clocked_node, backward_ns
):
    node, clock, transforms = clocked_node
    stamp_ns = clock.nanoseconds - 100_000_000
    stamped_feedback(node, stamp_ns)
    node.pose = (5.0, 0.0, 0.0)
    node.command_stamp = node.feedback_stamp
    previous = (
        node.pose,
        node.command_stamp,
        node.feedback_stamp,
        node.previous_odom_stamp_ns,
        list(node.angles),
        list(node.measured_speeds),
        list(node.sent_angles),
    )
    stamped_feedback(node, stamp_ns - backward_ns, wheel_speed=9.0, angle=0.5)
    assert (
        node.pose,
        node.command_stamp,
        node.feedback_stamp,
        node.previous_odom_stamp_ns,
        node.angles,
        node.measured_speeds,
        node.sent_angles,
    ) == previous
    assert len(node.odom_pub.messages) == len(transforms) == 1
    stamped_feedback(node, clock.nanoseconds)
    assert node.pose == pytest.approx((5.02, 0.0, 0.0))
    assert len(node.odom_pub.messages) == len(transforms) == 2


def test_adjacent_new_nanosecond_is_integrated_at_large_epoch(clocked_node):
    node, clock, transforms = clocked_node
    stamp_ns = clock.nanoseconds - 100_000_000
    stamped_feedback(node, stamp_ns)
    stamped_feedback(node, stamp_ns + 1)
    assert node.previous_odom_stamp_ns == stamp_ns + 1
    assert node.pose[0] == pytest.approx(0.2e-9, abs=1e-15)
    assert len(node.odom_pub.messages) == len(transforms) == 2


@pytest.mark.parametrize("observer", ["feedback", "status"])
def test_real_clock_rollback_starts_new_epoch_in_both_modes(clocked_node, observer):
    node, clock, transforms = clocked_node
    stamped_feedback(node, clock.nanoseconds)
    node.pose = (5.0, 0.0, 0.0)
    node.command_stamp = node.feedback_stamp
    node.sent_speeds = [2.0] * 4
    if node.supervisor is not None:
        node.supervisor.phase = TransitionPhase.ACTIVE
    clock.nanoseconds = 100_000_000
    if observer == "status":
        node.publish_status()
        assert node.feedback_stamp is None
        assert node.previous_odom_stamp_ns is None
        assert node.pose == (0.0, 0.0, 0.0)
    stamped_feedback(node, clock.nanoseconds, wheel_speed=0.0, angle=0.3)
    assert node.command_stamp is None
    assert node.pose == (0.0, 0.0, 0.0)
    assert node.previous_odom_stamp_ns == clock.nanoseconds
    assert node.sent_angles == [0.3] * 4
    assert node.sent_speeds == [0.0] * 4
    if node.supervisor is not None:
        assert node.supervisor.phase is TransitionPhase.IDLE
    node.on_timer()
    assert node.previous_odom_stamp_ns == clock.nanoseconds
    assert node.feedback_stamp is not None
    assert len(node.odom_pub.messages) == len(transforms) == 2


def test_clock_rollback_invalidates_state_even_with_partial_feedback(clocked_node):
    node, clock, transforms = clocked_node
    stamped_feedback(node, clock.nanoseconds)
    node.pose = (5.0, 0.0, 0.0)
    node.command_stamp = node.feedback_stamp
    clock.nanoseconds = 100_000_000
    node.on_feedback(JointState())
    assert node.pose == (0.0, 0.0, 0.0)
    assert node.command_stamp is None
    assert node.feedback_stamp is None
    assert node.previous_odom_stamp_ns is None
    assert len(node.odom_pub.messages) == len(transforms) == 1


def test_drive_timer_can_observe_clock_reset_before_feedback(node, monkeypatch):
    feedback(node)
    node.pose = (5.0, 0.0, 0.0)
    node.command_stamp = node.feedback_stamp

    class Clock:
        nanoseconds = 100_000_000

        def now(self):
            return Time(nanoseconds=self.nanoseconds)

    clock = Clock()
    monkeypatch.setattr(node, "get_clock", lambda: clock)
    node.on_timer()
    assert node.command_stamp is None
    assert node.feedback_stamp is None
    assert node.previous_odom_stamp_ns is None
    assert node.pose == (0.0, 0.0, 0.0)
    assert list(node.wheel_pub.messages[-1].data) == [0.0] * 4
    feedback(node, steering_angles=[0.3] * 4)
    clock.nanoseconds += 10_000_000
    node.on_timer()
    assert node.previous_odom_stamp_ns == 100_000_000
    assert node.feedback_stamp is not None


def test_external_control_preserves_odometry_without_command_ownership():
    rclpy.init()
    node = SwerveController(
        parameter_overrides=[
            Parameter("external_joint_control", value=True),
            Parameter("publish_odom_tf", value=False),
        ]
    )
    try:
        assert node.supervisor is None
        assert node.wheel_pub is None
        assert node.steer_pub is None
        node.wheel_pub = Capture()
        node.steer_pub = Capture()
        node.odom_pub = Capture()
        node.status_pub = Capture()
        feedback(node, wheel_speeds=[2.0] * 4)
        assert node.odom_pub.messages[-1].twist.twist.linear.x == pytest.approx(0.2)
        msg = TwistStamped()
        msg.twist.linear.x = 1.0
        node.on_command(msg)
        node.on_timer()
        assert not node.wheel_pub.messages
        assert not node.steer_pub.messages
        assert node.command_stamp is None
        assert node.requested_mode is None
        assert node.active_mode is None
        node.publish_status()
        status = node.status_pub.messages[-1].status[0]
        assert status.message == "external joint control"
        assert [(item.key, item.value) for item in status.values] == [
            ("command_owner", "external")
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command(node, x=0.5, y=0.0, yaw=0.0):
    msg = TwistStamped()
    msg.twist.linear.x = x
    msg.twist.linear.y = y
    msg.twist.angular.z = yaw
    node.on_command(msg)


def tick(node, period=0.01):
    node.last_tick = node.get_clock().now().nanoseconds * 1e-9 - period
    node.on_timer()
    return list(node.wheel_pub.messages[-1].data)


def settle_steering(node):
    for _ in range(100):
        feedback(node, steering_angles=node.sent_angles)
        wheel_speeds = tick(node, period=0.1)
        if node.steering_is_aligned:
            return wheel_speeds
    raise AssertionError("Steering did not reach its target")


def test_no_motion_without_feedback(node):
    command(node)
    assert tick(node) == [0.0] * 4


def test_command_timeout_stops_wheels_and_returns_steering_home(node):
    feedback(node, steering_angles=[0.4] * 4)
    node.sent_angles = [0.4] * 4
    command(node)
    node.command_stamp -= 1.0
    assert tick(node, period=0.1) == [0.0] * 4
    assert max(abs(angle) for angle in node.sent_angles) < 0.4

    for _ in range(2):
        tick(node, period=0.1)
    assert node.sent_angles == pytest.approx([0.0] * 4)


def test_feedback_timeout_stops(node):
    feedback(node)
    command(node)
    assert min(tick(node)) > 0
    node.feedback_stamp -= 1.0
    assert tick(node) == [0.0] * 4


def test_spin_waits_for_steering_alignment(node):
    feedback(node)
    command(node, x=0.0, yaw=0.5)

    assert tick(node, period=0.1) == [0.0] * 4
    assert node.requested_mode is DriveMode.SPIN
    assert not node.steering_is_aligned
    assert max(abs(angle) for angle in node.sent_angles) > 0.0

    wheel_speeds = settle_steering(node)
    assert max(abs(speed) for speed in wheel_speeds) > 0.0


def test_crab_waits_for_steering_alignment(node):
    feedback(node)
    command(node, x=0.0, y=0.4)

    assert tick(node, period=0.1) == [0.0] * 4
    assert node.requested_mode is DriveMode.CRAB
    assert not node.steering_is_aligned

    wheel_speeds = settle_steering(node)
    assert max(abs(speed) for speed in wheel_speeds) > 0.0


def test_differential_waits_until_steering_returns_home(node):
    feedback(node, steering_angles=[0.7] * 4)
    node.sent_angles = [0.7] * 4
    command(node, x=0.3, yaw=0.2)

    assert tick(node, period=0.1) == [0.0] * 4
    assert node.requested_mode is DriveMode.DIFFERENTIAL
    assert not node.steering_is_aligned

    wheel_speeds = settle_steering(node)
    assert node.sent_angles == pytest.approx([0.0] * 4)
    expected_speeds, _ = node.kinematics.inverse(0.3, 0.0, 0.2, [0.0] * 4)
    assert wheel_speeds == pytest.approx(expected_speeds)


def test_active_differential_mode_does_not_reapply_steering_interlock(node):
    feedback(node)
    command(node, x=0.3, yaw=0.2)
    expected_speeds, expected_angles = node.kinematics.inverse(
        0.3,
        0.0,
        0.2,
        [0.0] * 4,
    )
    assert tick(node, period=0.1) == pytest.approx(expected_speeds)
    assert node.active_mode is DriveMode.DIFFERENTIAL
    assert node.sent_angles == pytest.approx([0.0] * 4)

    feedback(node)
    wheel_speeds = tick(node, period=0.1)

    assert not node.steering_is_aligned
    assert node.sent_angles == pytest.approx(expected_angles)
    assert wheel_speeds == pytest.approx(expected_speeds)


def test_direct_crab_to_spin_switch_stops_drive_until_realigned(node):
    feedback(node)
    command(node, x=0.0, y=0.4)
    assert max(abs(speed) for speed in settle_steering(node)) > 0.0

    feedback(node, steering_angles=node.sent_angles)
    command(node, x=0.0, yaw=0.5)

    assert tick(node, period=0.1) == [0.0] * 4
    assert node.requested_mode is DriveMode.SPIN
    assert not node.steering_is_aligned
    assert max(abs(speed) for speed in settle_steering(node)) > 0.0


def test_explicit_stop_returns_crab_steering_home(node):
    feedback(node)
    command(node, x=0.0, y=0.4)
    settle_steering(node)
    assert max(abs(angle) for angle in node.sent_angles) > 1.0

    command(node, x=0.0)
    assert tick(node, period=0.1) == [0.0] * 4
    assert node.requested_mode is None
    assert node.active_mode is None

    for _ in range(10):
        tick(node, period=0.1)
    assert node.sent_angles == pytest.approx([0.0] * 4)


@pytest.mark.parametrize("intermediate", [(0.3, 0.3, 0.0), (0.0, 0.0, 0.5)])
def test_interrupted_transition_back_to_differential_reapplies_home_gate(
    node, intermediate
):
    feedback(node)
    command(node, x=0.3)
    assert min(tick(node, period=0.1)) > 0.0

    command(node, *intermediate)
    assert tick(node, period=0.1) == [0.0] * 4
    assert node.active_mode is None
    feedback(node, steering_angles=node.sent_angles)
    assert max(abs(angle) for angle in node.angles) > 0.05

    command(node, x=0.3)
    assert tick(node, period=0.1) == [0.0] * 4
    assert node.active_mode is None
    assert min(settle_steering(node)) > 0.0
    assert node.active_mode is DriveMode.DIFFERENTIAL


@pytest.mark.parametrize(
    ("first", "second"),
    [
        ((0.3, 0.0, 0.0), (0.0, 0.3, 0.0)),
        ((0.3, 0.0, 0.0), (0.0, 0.0, 0.5)),
        ((0.0, 0.3, 0.0), (0.3, 0.0, 0.0)),
        ((0.0, 0.3, 0.0), (0.0, 0.0, 0.5)),
        ((0.0, 0.0, 0.5), (0.3, 0.0, 0.0)),
        ((0.0, 0.0, 0.5), (0.0, 0.3, 0.0)),
    ],
)
def test_direct_mode_changes_hold_drive_until_alignment(node, first, second):
    feedback(node)
    command(node, *first)
    assert any(settle_steering(node))
    feedback(node, steering_angles=node.sent_angles)
    command(node, *second)
    assert tick(node, period=0.1) == [0.0] * 4
    assert node.active_mode is None
    assert any(settle_steering(node))


@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_nonfinite_command_stops(node, value):
    feedback(node)
    command(node)
    tick(node)
    command(node, value)
    assert tick(node) == [0.0] * 4


def test_partial_feedback_ignored(node):
    msg = JointState()
    msg.name = node.wheel_joint_names + node.steering_joint_names
    node.on_feedback(msg)
    assert node.feedback_stamp is None


def test_encoder_rotation_is_published(node):
    msg = feedback(node)
    msg.header.stamp = node.get_clock().now().to_msg()
    for i, (x, y) in enumerate(node.kinematics.positions):
        msg.velocity[i] = math.hypot(x, y) / 0.1
        msg.position[i + 4] = math.atan2(x, -y)
    node.on_feedback(msg)
    result = node.odom_pub.messages[-1]
    assert result.twist.twist.angular.z == pytest.approx(1)
    assert result.header.frame_id == "odom"
    assert result.child_frame_id == "base_footprint"


def test_differential_encoder_velocity_is_published(node):
    wheel_speeds, steering_angles = node.kinematics.inverse(
        0.3,
        0.0,
        0.2,
        [0.0] * 4,
    )
    feedback(
        node,
        steering_angles=steering_angles,
        wheel_speeds=wheel_speeds,
    )

    result = node.odom_pub.messages[-1]
    assert result.twist.twist.linear.x == pytest.approx(0.3)
    assert result.twist.twist.linear.y == pytest.approx(0.0)
    assert result.twist.twist.angular.z == pytest.approx(0.2)


@pytest.mark.parametrize("kind", ["expired", "future", "wrong_frame"])
def test_invalid_header_stops(node, kind):
    feedback(node)
    command(node)
    tick(node)
    msg = TwistStamped()
    msg.twist.linear.x = 0.5
    msg.header.stamp = node.get_clock().now().to_msg()
    if kind == "expired":
        msg.header.stamp.sec -= 1
    elif kind == "future":
        msg.header.stamp.sec += 1
    else:
        msg.header.frame_id = "map"
    node.on_command(msg)
    assert tick(node) == [0.0] * 4


def test_wheel_saturation_and_explicit_stop(node):
    feedback(node)
    command(node, 10000.0)
    for _ in range(60):
        node.last_tick = node.get_clock().now().nanoseconds * 1e-9 - 0.1
        node.on_timer()
        assert max(abs(v) for v in node.sent_speeds) <= 20
    assert node.sent_speeds == pytest.approx([20.0] * 4)
    command(node, 0.0)
    assert tick(node) == [0.0] * 4


def test_measured_wheel_motion_holds_steering_during_transition(node):
    feedback(node, wheel_speeds=[1.0] * 4)
    command(node, x=0.0, y=0.3)
    assert tick(node, period=0.1) == [0.0] * 4
    assert node.sent_angles == [0.0] * 4
    assert node.supervisor.phase is TransitionPhase.BRAKING
    feedback(node, wheel_speeds=[0.0] * 4)
    assert tick(node, period=0.1) == [0.0] * 4
    assert any(node.sent_angles)


def test_drive_status_reports_requested_mode_and_blocking_reason(node):
    feedback(node, wheel_speeds=[1.0] * 4)
    command(node, x=0.0, y=0.3)
    tick(node)
    node.publish_status()
    status = node.status_pub.messages[-1].status[0]
    values = {item.key: item.value for item in status.values}
    assert values["requested_mode"] == "crab"
    assert values["active_mode"] == "none"
    assert values["phase"] == "braking"
    assert status.message == "waiting_for_wheels_to_stop"


def test_timeout_fault_requires_zero_command_before_drive(node):
    feedback(node)
    command(node, x=0.0, yaw=0.5)
    tick(node)
    node.supervisor._transition_since -= 6.0
    assert tick(node) == [0.0] * 4
    assert node.supervisor.phase is TransitionPhase.FAULT
    command(node, x=0.3)
    assert tick(node) == [0.0] * 4
    command(node, x=0.0)
    tick(node)
    assert node.supervisor.phase is TransitionPhase.IDLE
    command(node, x=0.3)
    assert any(tick(node))


@pytest.mark.parametrize("publish_odom", [True, False])
@pytest.mark.parametrize("publish_tf", [True, False])
def test_odometry_outputs_are_independent_and_use_configured_names(
    monkeypatch, publish_odom, publish_tf
):
    transforms = []

    class Broadcaster:
        def __init__(self, node):
            del node

        def sendTransform(self, transform):
            transforms.append(transform)

    monkeypatch.setattr(controller_module, "TransformBroadcaster", Broadcaster)
    rclpy.init()
    node = SwerveController(
        namespace="robot1",
        parameter_overrides=[
            Parameter("publish_odom", value=publish_odom),
            Parameter("publish_odom_tf", value=publish_tf),
            Parameter("odom_topic", value="wheel/odometry"),
            Parameter("odom_frame", value="custom_odom"),
            Parameter("odom_child_frame", value="custom_base"),
        ],
    )
    try:
        assert (node.odom_pub is not None) == publish_odom
        assert (node.transform_broadcaster is not None) == publish_tf
        messages = Capture()
        if publish_odom:
            assert node.odom_pub.topic_name == "/robot1/wheel/odometry"
            node.odom_pub = messages
        first = feedback(node, wheel_speeds=[3.0] * 4)
        node.previous_odom_stamp_ns = (
            first.header.stamp.sec * 1_000_000_000
            + first.header.stamp.nanosec
            - 50_000_000
        )
        feedback(node, wheel_speeds=[3.0] * 4)
        assert node.pose[0] > 0
        assert bool(messages.messages) == publish_odom
        assert bool(transforms) == publish_tf
        if publish_odom:
            message = messages.messages[-1]
            assert message.header.frame_id == "custom_odom"
            assert message.child_frame_id == "custom_base"
            assert message.twist.twist.linear.x == pytest.approx(0.3)
        if publish_tf:
            transform = transforms[-1]
            assert transform.header.frame_id == "custom_odom"
            assert transform.child_frame_id == "custom_base"
            assert transform.transform.translation.x == pytest.approx(node.pose[0])
    finally:
        node.destroy_node()
        rclpy.shutdown()
