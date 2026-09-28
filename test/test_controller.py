import math

import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from swerve_gazebo_sim.controller import SwerveController
from swerve_gazebo_sim.kinematics import DriveMode


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    result = SwerveController(
        parameter_overrides=[Parameter("publish_odom_tf", value=False)]
    )
    result.wheel_pub = Capture()
    result.steer_pub = Capture()
    result.odom_pub = Capture()
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
    assert wheel_speeds == pytest.approx([2.5, 3.5, 2.5, 3.5])


def test_active_differential_mode_does_not_reapply_steering_interlock(node):
    feedback(node)
    command(node, x=0.3, yaw=0.2)
    assert tick(node, period=0.1) == pytest.approx([2.5, 3.5, 2.5, 3.5])
    assert node.active_mode is DriveMode.DIFFERENTIAL

    feedback(node, steering_angles=[0.2] * 4)
    wheel_speeds = tick(node, period=0.1)

    assert not node.steering_is_aligned
    assert wheel_speeds == pytest.approx([2.5, 3.5, 2.5, 3.5])


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
    node.active_mode = DriveMode.DIFFERENTIAL
    feedback(node, wheel_speeds=[2.5, 3.5, 2.5, 3.5])

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
