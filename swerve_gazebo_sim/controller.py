# SPDX-License-Identifier: Apache-2.0
"""ROS adapter for bounded steering, wheel commands and encoder odometry."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .kinematics import SwerveKinematics, integrate_pose


class SwerveController(Node):
    def __init__(self, **kwargs):
        super().__init__('swerve_controller', **kwargs)
        defaults = dict(wheelbase=0.6, track_width=0.5, wheel_radius=0.1,
                        steering_limit=math.pi / 2, update_rate=100.0,
                        cmd_timeout=0.5, feedback_timeout=0.25,
                        max_wheel_speed=20.0, max_wheel_acceleration=40.0,
                        max_steering_rate=2.5, publish_odom_tf=True,
                        pose_variance=0.02, twist_variance=0.02,
                        joint_prefix='', frame_prefix='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.cfg.items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
        c = self.cfg
        self.kinematics = SwerveKinematics(c['wheelbase'], c['track_width'],
                                         c['wheel_radius'], c['steering_limit'])
        self.wheels = [c['joint_prefix'] + p + '_wheel_joint' for p in ('fl', 'fr', 'rl', 'rr')]
        self.steers = [c['joint_prefix'] + p + '_steering_joint' for p in ('fl', 'fr', 'rl', 'rr')]
        self.odom_frame = c['frame_prefix'] + 'odom'
        self.base_frame = c['frame_prefix'] + 'base_footprint'
        self.command = (0.0, 0.0, 0.0)
        self.command_stamp = None
        self.feedback_stamp = None
        self.angles = [0.0] * 4
        self.sent_angles = [0.0] * 4
        self.sent_speeds = [0.0] * 4
        self.pose = (0.0, 0.0, 0.0)
        self.previous_odom_stamp = None
        self.last_tick = self.get_clock().now().nanoseconds * 1e-9
        self.wheel_pub = self.create_publisher(Float64MultiArray, 'wheel_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, 'steering_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self) if c['publish_odom_tf'] else None
        self.create_subscription(TwistStamped, 'cmd_vel', self.on_command, 10)
        self.create_subscription(JointState, 'joint_states', self.on_feedback, 10)
        self.create_timer(1.0 / c['update_rate'], self.on_timer)

    def on_command(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        # Empty frame means base frame; a zero stamp explicitly uses reception time.
        if (not all(math.isfinite(v) for v in values)
                or msg.header.frame_id not in ('', self.base_frame)
                or (stamp != 0 and (now - stamp > self.cfg['cmd_timeout'] or stamp - now > 0.1))):
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
        dt = max(0.0, min(dt, 0.1))
        active = (self.command_stamp is not None and self.feedback_stamp is not None
                  and 0 <= now - self.command_stamp <= self.cfg['cmd_timeout']
                  and 0 <= now - self.feedback_stamp <= self.cfg['feedback_timeout']
                  and any(abs(v) > 1e-9 for v in self.command))
        if active:
            speeds, targets = self.kinematics.inverse(*self.command, self.angles)
            scale = max(1.0, max(abs(v) for v in speeds) / self.cfg['max_wheel_speed'])
            for i in range(4):
                step = self.cfg['max_steering_rate'] * dt
                self.sent_angles[i] += max(-step, min(step, targets[i] - self.sent_angles[i]))
                # Reduce drive while the physical module is not aligned.
                target = speeds[i] / scale * max(0.0, math.cos(targets[i] - self.angles[i]))
                step = self.cfg['max_wheel_acceleration'] * dt
                self.sent_speeds[i] += max(-step, min(step, target - self.sent_speeds[i]))
        else:
            # Explicit stop, expired command or missing feedback: stop all wheels.
            self.sent_speeds = [0.0] * 4
        self.wheel_pub.publish(Float64MultiArray(data=self.sent_speeds))
        self.steer_pub.publish(Float64MultiArray(data=self.sent_angles))

    def on_feedback(self, msg):
        try:
            wi = [msg.name.index(name) for name in self.wheels]
            si = [msg.name.index(name) for name in self.steers]
            speeds = [msg.velocity[i] for i in wi]
            angles = [msg.position[i] for i in si]
            twist = self.kinematics.forward(speeds, angles)
        except (ValueError, IndexError):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp - now > 0.1 or now - stamp > self.cfg['feedback_timeout']:
            return
        if self.previous_odom_stamp is not None:
            dt = stamp - self.previous_odom_stamp
            if dt < 0:
                self.pose = (0.0, 0.0, 0.0)
                self.command_stamp = None
            elif dt == 0:
                return
            elif dt <= self.cfg['feedback_timeout']:
                self.pose = integrate_pose(self.pose, twist, dt)
            # A gap does not justify extrapolating an unobserved trajectory.
        self.previous_odom_stamp = stamp
        if self.feedback_stamp is None:
            self.sent_angles = list(angles)
        self.feedback_stamp = stamp
        self.angles = angles
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[:2]
        odom.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        odom.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = twist[:2]
        odom.twist.twist.angular.z = twist[2]
        for idx in (0, 7, 35):
            odom.pose.covariance[idx] = self.cfg['pose_variance']
            odom.twist.covariance[idx] = self.cfg['twist_variance']
        for idx in (14, 21, 28):
            odom.pose.covariance[idx] = odom.twist.covariance[idx] = 1e6
        self.odom_pub.publish(odom)
        if self.tf is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.pose[0]
            transform.transform.translation.y = self.pose[1]
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)


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
