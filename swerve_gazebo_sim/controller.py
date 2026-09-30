# SPDX-License-Identifier: Apache-2.0
"""ROS adapter for bounded steering, wheel commands and encoder odometry."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .bringup import TRANSITION_DEFAULTS, validate_control
from .kinematics import SwerveKinematics, integrate_pose
from .motion import MotionSupervisor, TransitionPhase


class SwerveController(Node):
    def __init__(self, **kwargs):
        super().__init__("swerve_controller", **kwargs)
        defaults = {
            "wheelbase": 0.6,
            "track_width": 0.5,
            "wheel_radius": 0.1,
            "steering_limit": math.pi / 2,
            "update_rate": 100.0,
            "cmd_timeout": 0.5,
            "feedback_timeout": 0.25,
            "max_wheel_speed": 20.0,
            "max_wheel_acceleration": 40.0,
            "max_steering_rate": 2.5,
            "steering_alignment_tolerance": 0.05,
            **TRANSITION_DEFAULTS,
            "publish_odom_tf": True,
            "pose_variance": 0.02,
            "twist_variance": 0.02,
            "joint_prefix": "",
            "frame_prefix": "",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.configuration = {key: self.get_parameter(key).value for key in defaults}
        validate_control(self.configuration)

        self.kinematics = SwerveKinematics(
            self.configuration["wheelbase"],
            self.configuration["track_width"],
            self.configuration["wheel_radius"],
            self.configuration["steering_limit"],
        )
        self.supervisor = MotionSupervisor(self.kinematics, self.configuration)
        self.measured_speeds = [0.0] * 4
        corners = ("fl", "fr", "rl", "rr")
        self.wheel_joint_names = [
            self.configuration["joint_prefix"] + corner + "_wheel_joint"
            for corner in corners
        ]
        self.steering_joint_names = [
            self.configuration["joint_prefix"] + corner + "_steering_joint"
            for corner in corners
        ]
        self.odom_frame = self.configuration["frame_prefix"] + "odom"
        self.base_frame = self.configuration["frame_prefix"] + "base_footprint"
        self.command = (0.0, 0.0, 0.0)
        self.command_stamp = None
        self.feedback_stamp = None
        self.angles = [0.0] * 4
        self.sent_angles = [0.0] * 4
        self.sent_speeds = [0.0] * 4
        self.requested_mode = None
        self.active_mode = None
        self.steering_is_aligned = True
        self.pose = (0.0, 0.0, 0.0)
        self.previous_odom_stamp = None
        self.last_tick = self.get_clock().now().nanoseconds * 1e-9
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, "wheel_controller/commands", 10
        )
        self.steer_pub = self.create_publisher(
            Float64MultiArray, "steering_controller/commands", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.status_pub = self.create_publisher(DiagnosticArray, "drive_status", 10)
        self.transform_broadcaster = (
            TransformBroadcaster(self)
            if self.configuration["publish_odom_tf"]
            else None
        )
        self.create_subscription(TwistStamped, "cmd_vel", self.on_command, 10)
        self.create_subscription(JointState, "joint_states", self.on_feedback, 10)
        self.create_timer(1.0 / self.configuration["update_rate"], self.on_timer)
        self.create_timer(
            1.0 / self.configuration["status_publish_rate"], self.publish_status
        )

    def on_command(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        # Empty frame means base frame; a zero stamp explicitly uses reception time.
        if (
            not all(math.isfinite(v) for v in values)
            or msg.header.frame_id not in ("", self.base_frame)
            or (
                stamp != 0
                and (
                    now - stamp > self.configuration["cmd_timeout"] or stamp - now > 0.1
                )
            )
        ):
            self.command_stamp = None
            return
        self.command = values
        self.command_stamp = now if stamp == 0 else stamp

    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_tick
        self.last_tick = now
        if dt < 0:
            self.command_stamp = None
            self.feedback_stamp = None
            self.previous_odom_stamp = None
            self.pose = (0.0, 0.0, 0.0)
            self.supervisor.reset()
        dt = max(0.0, min(dt, 0.1))
        command_is_current = self.command_stamp is not None and (
            0 <= now - self.command_stamp <= self.configuration["cmd_timeout"]
        )
        feedback_is_current = self.feedback_stamp is not None and (
            0 <= now - self.feedback_stamp <= self.configuration["feedback_timeout"]
        )
        previous_phase = self.supervisor.phase
        decision = self.supervisor.update(
            self.command,
            self.angles,
            self.measured_speeds,
            self.sent_angles,
            now,
            command_is_current,
            feedback_is_current,
        )
        self.requested_mode = self.supervisor.requested_mode
        self.active_mode = self.supervisor.active_mode
        self.steering_is_aligned = feedback_is_current and all(
            abs(target - measured) <= self.configuration["steering_alignment_tolerance"]
            for target, measured in zip(decision.steering_targets, self.angles)
        )
        self._move_steering_toward(decision.steering_targets, dt)
        if decision.drive_enabled:
            self._move_wheels_toward(decision.wheel_targets, dt)
        else:
            self.sent_speeds = [0.0] * 4
        if (
            self.supervisor.phase is TransitionPhase.FAULT
            and previous_phase is not TransitionPhase.FAULT
        ):
            self.get_logger().error(self.supervisor.reason)

        self.wheel_pub.publish(Float64MultiArray(data=self.sent_speeds))
        self.steer_pub.publish(Float64MultiArray(data=self.sent_angles))

    def publish_status(self):
        status = DiagnosticStatus()
        status.name = self.get_fully_qualified_name() + "/motion"
        status.hardware_id = self.base_frame
        if self.supervisor.phase is TransitionPhase.FAULT:
            status.level = DiagnosticStatus.ERROR
        elif self.supervisor.reason not in ("", "stopped"):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = self.supervisor.reason or self.supervisor.phase.value
        status.values = [
            KeyValue(
                key="requested_mode",
                value=self.requested_mode.value if self.requested_mode else "none",
            ),
            KeyValue(
                key="active_mode",
                value=self.active_mode.value if self.active_mode else "none",
            ),
            KeyValue(key="phase", value=self.supervisor.phase.value),
            KeyValue(
                key="max_steering_error", value=str(self.supervisor.max_steering_error)
            ),
            KeyValue(key="wheels_stopped", value=str(self.supervisor.wheels_stopped)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)

    def _move_steering_toward(self, targets, dt):
        maximum_step = self.configuration["max_steering_rate"] * dt
        for module_index, target in enumerate(targets):
            error = target - self.sent_angles[module_index]
            step = max(-maximum_step, min(maximum_step, error))
            self.sent_angles[module_index] += step

    def _move_wheels_toward(self, targets, dt):
        speed_scale = max(
            1.0,
            max(abs(value) for value in targets)
            / self.configuration["max_wheel_speed"],
        )
        maximum_step = self.configuration["max_wheel_acceleration"] * dt
        for module_index, target in enumerate(targets):
            limited_target = target / speed_scale
            error = limited_target - self.sent_speeds[module_index]
            step = max(-maximum_step, min(maximum_step, error))
            self.sent_speeds[module_index] += step

    def on_feedback(self, msg):
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
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp - now > 0.1 or now - stamp > self.configuration["feedback_timeout"]:
            return
        if self.previous_odom_stamp is not None:
            dt = stamp - self.previous_odom_stamp
            if dt < 0:
                self.pose = (0.0, 0.0, 0.0)
                self.command_stamp = None
            elif dt == 0:
                return
            elif dt <= self.configuration["feedback_timeout"]:
                self.pose = integrate_pose(self.pose, twist, dt)
            # A gap does not justify extrapolating an unobserved trajectory.
        self.previous_odom_stamp = stamp
        if self.feedback_stamp is None:
            self.sent_angles = list(angles)
        self.feedback_stamp = stamp
        self.angles = angles
        self.measured_speeds = speeds
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
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
        self.odom_pub.publish(odom)
        if self.transform_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.pose[0]
            transform.transform.translation.y = self.pose[1]
            transform.transform.rotation = odom.pose.pose.orientation
            self.transform_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
