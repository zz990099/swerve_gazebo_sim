"""Encoder observer validation; no command authority lives in this node."""

import math

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from swerve_gazebo_sim.controller import OdometryObserver


@pytest.fixture
def observer():
    rclpy.init()
    node = OdometryObserver(parameter_overrides=[Parameter("use_sim_time", value=True)])
    yield node
    node.destroy_node()
    rclpy.shutdown()


def sample(node, stamp=0, speed=1.0):
    msg = JointState()
    msg.name = node.steering_joint_names + node.wheel_joint_names
    msg.position = [0.0] * 8
    msg.velocity = [0.0] * 4 + [speed] * 4
    msg.header.stamp.nanosec = stamp
    return msg


def test_encoder_integration_and_duplicate(observer):
    observer.on_feedback(sample(observer))
    observer.on_feedback(sample(observer, 50_000_000))
    assert observer.pose[0] == pytest.approx(0.005)
    observer.on_feedback(sample(observer, 50_000_000))
    assert observer.pose[0] == pytest.approx(0.005)


def test_invalid_feedback_does_not_advance(observer):
    observer.on_feedback(sample(observer))
    msg = sample(observer, 50_000_000, math.nan)
    observer.on_feedback(msg)
    assert observer.previous_odom_stamp_ns == 0
    msg = sample(observer, 50_000_000)
    msg.name.pop()
    observer.on_feedback(msg)
    assert observer.previous_odom_stamp_ns == 0


def test_clock_reset_clears_pose(observer):
    observer.on_feedback(sample(observer))
    observer.on_feedback(sample(observer, 50_000_000))
    observer._observe_clock(1_000_000_000)
    observer._observe_clock(0)
    assert observer.pose == (0, 0, 0)
    assert observer.previous_odom_stamp_ns is None
