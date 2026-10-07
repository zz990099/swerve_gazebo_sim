# SPDX-License-Identifier: Apache-2.0
"""Python chassis execution and encoder odometry over standard joint controllers."""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from swerve_gazebo_sim.bringup import (
    CHASSIS_DEFAULTS,
    OBSERVER_DEFAULTS,
    validate_control,
)
from swerve_gazebo_sim.chassis import Chassis, Command, Fault
from swerve_gazebo_sim.kinematics import integrate_pose
from swerve_gazebo_sim.msg import ChassisCommand, ChassisState


class SwerveController(Node):
    def __init__(self, **kwargs):
        super().__init__("swerve_controller", **kwargs)
        defaults = dict(
            CHASSIS_DEFAULTS,
            **OBSERVER_DEFAULTS,
            wheelbase=0.6,
            track_width=0.5,
            wheel_radius=0.1,
            joint_prefix="",
            frame_prefix="",
        )
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.configuration = {key: self.get_parameter(key).value for key in defaults}
        validate_control(self.configuration)
        geometry = {
            k: self.configuration[k]
            for k in ("wheelbase", "track_width", "wheel_radius")
        }
        self.chassis = Chassis(self.configuration, geometry)
        self.kinematics = self.chassis.kinematics
        self.body_frame = self.configuration["frame_prefix"] + "base_footprint"
        self.steering_pub = self.create_publisher(
            Float64MultiArray, "steering_controller/commands", 1
        )
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, "wheel_controller/commands", 1
        )
        self.state_pub = self.create_publisher(
            ChassisState, "chassis_controller/state", qos_profile_sensor_data
        )
        self.create_subscription(
            ChassisCommand, "chassis_controller/command", self.on_command, 1
        )
        self.wheel_joint_names = [
            self.configuration["joint_prefix"] + c + "_wheel_joint"
            for c in ("fl", "fr", "rl", "rr")
        ]
        self.steering_joint_names = [
            self.configuration["joint_prefix"] + c + "_steering_joint"
            for c in ("fl", "fr", "rl", "rr")
        ]
        self.odom_frame = (
            self.configuration["odom_frame"]
            or self.configuration["frame_prefix"] + "odom"
        )
        self.odom_child_frame = (
            self.configuration["odom_child_frame"]
            or self.configuration["frame_prefix"] + "base_footprint"
        )
        if self.odom_frame == self.odom_child_frame:
            raise ValueError("Odometry frames must differ")
        self.pose = (0.0, 0.0, 0.0)
        self.feedback_stamp = self.previous_odom_stamp_ns = None
        self._last_clock_ns = self.get_clock().now().nanoseconds
        self.odom_pub = (
            self.create_publisher(Odometry, self.configuration["odom_topic"], 10)
            if self.configuration["publish_odom"]
            else None
        )
        self.transform_broadcaster = (
            TransformBroadcaster(self)
            if self.configuration["publish_odom_tf"]
            else None
        )
        self.create_subscription(
            JointState, "joint_states", self.on_feedback, qos_profile_sensor_data
        )

        # A steady timer can publish zero targets even while Gazebo /clock is paused.
        self.timer = self.create_timer(
            1.0 / self.configuration["update_rate"],
            self.on_tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.encoder_stamp = None

    @staticmethod
    def planar(twist):
        if any((twist.linear.z, twist.angular.x, twist.angular.y)):
            raise ValueError("Only planar velocity is supported")
        return twist.linear.x, twist.linear.y, twist.angular.z

    def on_command(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self._observe_clock(now_ns)
        try:
            if (
                msg.header.frame_id != self.body_frame
                or not 0 <= msg.header.stamp.nanosec < 1_000_000_000
            ):
                raise ValueError("Invalid command header")
            command = Command(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                msg.mode,
                self.planar(msg.velocity),
                msg.request_id,
                self.planar(msg.entry_velocity),
            )
            self.chassis.receive(command, now_ns * 1e-9, time.monotonic())
        except ValueError:
            self.chassis.stop(Fault.COMMAND)

    def publish_targets(self):
        self.wheel_pub.publish(Float64MultiArray(data=self.chassis.speeds))
        self.steering_pub.publish(Float64MultiArray(data=self.chassis.angles))

    def on_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        self._observe_clock(now_ns)
        self.chassis.step(now_ns * 1e-9, time.monotonic())
        self.publish_targets()
        if self.encoder_stamp is None:
            return
        chassis = self.chassis
        state = ChassisState()
        state.header.stamp = self.encoder_stamp
        state.header.frame_id = self.body_frame
        state.request_id = chassis.request_id
        state.actual_mode, state.requested_mode = int(chassis.actual_mode), int(
            chassis.requested_mode
        )
        state.phase, state.fault = int(chassis.phase), int(chassis.fault)
        state.confirmed = chassis.confirmed
        state.steering_angles = chassis.measured_angles
        state.wheel_speeds = chassis.measured_speeds
        state.velocity.linear.x, state.velocity.linear.y, state.velocity.angular.z = (
            self.kinematics.forward(chassis.measured_speeds, chassis.measured_angles)
        )
        (
            state.accepted_entry_velocity.linear.x,
            state.accepted_entry_velocity.linear.y,
            state.accepted_entry_velocity.angular.z,
        ) = chassis.entry
        state.accepted_steering = chassis.accepted_steering
        self.state_pub.publish(state)

    def _observe_clock(self, now_ns):
        if now_ns < self._last_clock_ns:
            self.feedback_stamp = self.previous_odom_stamp_ns = None
            self.pose = (0.0, 0.0, 0.0)
            self.encoder_stamp = None
            self.chassis.clock_reset()
        self._last_clock_ns = now_ns

    def on_feedback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self._observe_clock(now_ns)
        try:
            if any(
                msg.name.count(name) != 1
                for name in self.wheel_joint_names + self.steering_joint_names
            ):
                raise ValueError("Expected every chassis joint exactly once")
            wheel_indices = [msg.name.index(name) for name in self.wheel_joint_names]
            steering_indices = [
                msg.name.index(name) for name in self.steering_joint_names
            ]
            speeds = [msg.velocity[index] for index in wheel_indices]
            angles = [msg.position[index] for index in steering_indices]
            twist = self.kinematics.forward(speeds, angles)
        except (ValueError, IndexError):
            self.chassis.stop(Fault.FEEDBACK)
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age_s = (now_ns - stamp_ns) * 1e-9
        if (
            not 0 <= msg.header.stamp.nanosec < 1_000_000_000
            or stamp_ns < 0
            or age_s < -0.1
            or age_s > self.configuration["feedback_timeout"]
        ):
            return
        if not self.chassis.observe(angles, speeds, stamp_ns * 1e-9, time.monotonic()):
            return
        if self.previous_odom_stamp_ns is not None:
            elapsed_ns = stamp_ns - self.previous_odom_stamp_ns
            if elapsed_ns <= 0:
                return
            dt = elapsed_ns * 1e-9
            if dt <= self.configuration["feedback_timeout"]:
                self.pose = integrate_pose(self.pose, twist, dt)
            # A gap does not justify extrapolating an unobserved trajectory.
        self.encoder_stamp = msg.header.stamp
        self.previous_odom_stamp_ns = stamp_ns
        self.feedback_stamp = stamp_ns * 1e-9
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.odom_child_frame
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        odom.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        odom.twist.twist.linear.x = twist[0]
        odom.twist.twist.linear.y = twist[1]
        odom.twist.twist.angular.z = twist[2]
        for idx in (0, 7, 35):
            odom.pose.covariance[idx] = self.configuration["pose_variance"]
            odom.twist.covariance[idx] = self.configuration["twist_variance"]
        for idx in (14, 21, 28):
            odom.pose.covariance[idx] = 1e6
            odom.twist.covariance[idx] = 1e6
        if self.odom_pub is not None:
            self.odom_pub.publish(odom)
        if self.transform_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.odom_child_frame
            transform.transform.translation.x = self.pose[0]
            transform.transform.translation.y = self.pose[1]
            transform.transform.rotation = odom.pose.pose.orientation
            self.transform_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SwerveController()

    def interrupt(signum, frame):
        del signum, frame
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.chassis.stop(Fault.COMMAND)
            node.publish_targets()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
