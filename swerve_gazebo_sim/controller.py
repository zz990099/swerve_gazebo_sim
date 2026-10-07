# SPDX-License-Identifier: Apache-2.0
"""Encoder odometry observer. The ros2_control plugin exclusively owns commands."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from swerve_gazebo_sim.bringup import OBSERVER_DEFAULTS
from swerve_gazebo_sim.kinematics import SwerveKinematics, integrate_pose


class OdometryObserver(Node):
    def __init__(self, **kwargs):
        super().__init__("swerve_odometry", **kwargs)
        defaults = dict(
            OBSERVER_DEFAULTS,
            wheelbase=0.6,
            track_width=0.5,
            wheel_radius=0.1,
            joint_prefix="",
            frame_prefix="",
        )
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.configuration = {key: self.get_parameter(key).value for key in defaults}
        self.kinematics = SwerveKinematics(
            *(
                self.configuration[k]
                for k in ("wheelbase", "track_width", "wheel_radius")
            )
        )
        for key in ("feedback_timeout", "pose_variance", "twist_variance"):
            if (
                not math.isfinite(self.configuration[key])
                or self.configuration[key] <= 0
            ):
                raise ValueError(f"Invalid {key}")
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

    def _observe_clock(self, now_ns):
        if now_ns < self._last_clock_ns:
            self.feedback_stamp = self.previous_odom_stamp_ns = None
            self.pose = (0.0, 0.0, 0.0)
        self._last_clock_ns = now_ns

    def on_feedback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self._observe_clock(now_ns)
        try:
            wheel_indices = [msg.name.index(name) for name in self.wheel_joint_names]
            steering_indices = [
                msg.name.index(name) for name in self.steering_joint_names
            ]
            speeds = [msg.velocity[index] for index in wheel_indices]
            angles = [msg.position[index] for index in steering_indices]
            twist = self.kinematics.forward(speeds, angles)
        except (ValueError, IndexError):
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age_s = (now_ns - stamp_ns) * 1e-9
        if (
            stamp_ns < 0
            or age_s < -0.1
            or age_s > self.configuration["feedback_timeout"]
        ):
            return
        if self.previous_odom_stamp_ns is not None:
            elapsed_ns = stamp_ns - self.previous_odom_stamp_ns
            if elapsed_ns <= 0:
                return
            dt = elapsed_ns * 1e-9
            if dt <= self.configuration["feedback_timeout"]:
                self.pose = integrate_pose(self.pose, twist, dt)
            # A gap does not justify extrapolating an unobserved trajectory.
        self.previous_odom_stamp_ns = stamp_ns
        self.feedback_stamp = stamp_ns * 1e-9
        self.angles = angles
        self.measured_speeds = speeds
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
    rclpy.init(args=args)
    node = OdometryObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
