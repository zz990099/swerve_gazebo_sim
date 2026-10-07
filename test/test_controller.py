"""ROS adapter validation for chassis execution and encoder odometry."""

import math

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from swerve_gazebo_sim.controller import SwerveController


@pytest.fixture
def observer():
    rclpy.init()
    node = SwerveController(parameter_overrides=[Parameter("use_sim_time", value=True)])
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


def test_ros_command_validation_and_fault_recovery(observer):
    from swerve_gazebo_sim.msg import ChassisCommand

    from swerve_gazebo_sim.chassis import Fault

    observer.on_feedback(sample(observer, speed=0.0))
    command = ChassisCommand()
    command.header.frame_id = "wrong_frame"
    observer.on_command(command)
    assert observer.chassis.fault == Fault.COMMAND
    command.header.frame_id = observer.body_frame
    command.request_id = 1
    observer.on_command(command)
    assert observer.chassis.fault == Fault.NONE
    command.velocity.linear.z = 0.1
    observer.on_command(command)
    assert observer.chassis.fault == Fault.COMMAND
    assert observer.chassis.speeds == [0.0] * 4


def test_duplicate_joint_names_are_rejected(observer):
    observer.on_feedback(sample(observer))
    msg = sample(observer, 50_000_000)
    msg.name.append(msg.name[0])
    observer.on_feedback(msg)
    assert observer.previous_odom_stamp_ns == 0


@pytest.mark.parametrize(
    ("publish_odom", "publish_tf"), [(False, True), (True, False), (False, False)]
)
def test_odometry_and_tf_switches_are_independent(publish_odom, publish_tf):
    rclpy.init()
    node = SwerveController(
        parameter_overrides=[
            Parameter("use_sim_time", value=True),
            Parameter("publish_odom", value=publish_odom),
            Parameter("publish_odom_tf", value=publish_tf),
        ]
    )
    try:
        assert (node.odom_pub is not None) == publish_odom
        assert (node.transform_broadcaster is not None) == publish_tf
        node.on_feedback(sample(node))
        node.on_feedback(sample(node, 50_000_000))
        assert node.pose[0] == pytest.approx(0.005)
    finally:
        node.destroy_node()
        rclpy.shutdown()
