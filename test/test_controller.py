import math

import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from swerve_gazebo_sim.controller import SwerveController


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    result = SwerveController(parameter_overrides=[Parameter('publish_odom_tf', value=False)])
    result.wheel_pub = Capture()
    result.steer_pub = Capture()
    result.odom_pub = Capture()
    yield result
    result.destroy_node()
    rclpy.shutdown()


def feedback(node):
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = node.wheels + node.steers
    msg.position = [0.0] * 8
    msg.velocity = [0.0] * 8
    node.on_feedback(msg)
    return msg


def command(node, x=0.5):
    msg = TwistStamped()
    msg.twist.linear.x = x
    node.on_command(msg)


def tick(node):
    node.last_tick = node.get_clock().now().nanoseconds * 1e-9 - 0.01
    node.on_timer()
    return list(node.wheel_pub.messages[-1].data)


def test_no_motion_without_feedback(node):
    command(node)
    assert tick(node) == [0.0] * 4


def test_command_timeout_stops_wheels_and_holds_steering(node):
    feedback(node)
    command(node)
    assert min(tick(node)) > 0
    angles = list(node.sent_angles)
    node.command_stamp -= 1.0
    assert tick(node) == [0.0] * 4
    assert node.sent_angles == angles


def test_feedback_timeout_stops(node):
    feedback(node)
    command(node)
    assert min(tick(node)) > 0
    node.feedback_stamp -= 1.0
    assert tick(node) == [0.0] * 4


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_nonfinite_command_stops(node, value):
    feedback(node)
    command(node)
    tick(node)
    command(node, value)
    assert tick(node) == [0.0] * 4


def test_partial_feedback_ignored(node):
    msg = JointState()
    msg.name = node.wheels + node.steers
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
    assert result.header.frame_id == 'odom'
    assert result.child_frame_id == 'base_footprint'


@pytest.mark.parametrize('kind', ['expired', 'future', 'wrong_frame'])
def test_invalid_header_stops(node, kind):
    feedback(node)
    command(node)
    tick(node)
    msg = TwistStamped()
    msg.twist.linear.x = 0.5
    msg.header.stamp = node.get_clock().now().to_msg()
    if kind == 'expired':
        msg.header.stamp.sec -= 1
    elif kind == 'future':
        msg.header.stamp.sec += 1
    else:
        msg.header.frame_id = 'map'
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
