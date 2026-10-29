#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Ideal localization for simulation tests: publish map -> odom, never odom."""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from swerve_gazebo_sim.planar_localization import PoseHistory, map_to_odom
from tf2_ros import TransformBroadcaster


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def planar_pose(message):
    p = message.pose.pose
    q = p.orientation
    norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError("Invalid orientation")
    result = (
        p.position.x,
        p.position.y,
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
    )
    if not all(math.isfinite(v) for v in result):
        raise ValueError("Nonfinite pose")
    return result


class GroundTruthLocalization(Node):
    def __init__(self):
        super().__init__("ground_truth_localization")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.body_frame = self.declare_parameter("body_frame", "base_footprint").value
        odom_topic = self.declare_parameter("odom_topic", "odom").value
        if self.map_frame in (self.odom_frame, self.body_frame):
            raise ValueError("map, odom and body frames must differ")
        self.history = PoseHistory()
        self.pending = deque(maxlen=64)
        self.last_clock = None
        self.last_truth = -math.inf
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self.wheel, 100)
        self.create_subscription(Odometry, "ground_truth/odom", self.truth, 100)
        self.get_logger().info(
            "IDEAL simulation localization enabled; no SLAM/noise model"
        )

    def fresh(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.history = PoseHistory()
            self.pending.clear()
            self.last_truth = -math.inf
        self.last_clock = now
        t = stamp(message)
        return math.isfinite(t) and -0.01 <= now - t <= 0.2

    def wheel(self, message):
        if (
            not self.fresh(message)
            or message.header.frame_id != self.odom_frame
            or message.child_frame_id != self.body_frame
        ):
            return
        try:
            self.history.add(stamp(message), planar_pose(message))
        except ValueError:
            return
        self.drain()

    def truth(self, message):
        if not self.fresh(message) or stamp(message) <= self.last_truth:
            return
        try:
            pose = planar_pose(message)
        except ValueError:
            return
        self.last_truth = stamp(message)
        self.pending.append((message.header.stamp, self.last_truth, pose))
        self.drain()

    def drain(self):
        while self.pending and self.history.samples:
            source_stamp, t, truth = self.pending[0]
            if t > self.history.samples[-1][0] + 1e-8:
                return
            self.pending.popleft()
            wheel = self.history.at(t)
            now = self.get_clock().now().nanoseconds * 1e-9
            if wheel is None or now - t > 0.2:
                continue
            x, y, angle = map_to_odom(truth, wheel)
            out = TransformStamped()
            out.header.stamp = source_stamp
            out.header.frame_id = self.map_frame
            out.child_frame_id = self.odom_frame
            out.transform.translation.x = x
            out.transform.translation.y = y
            out.transform.rotation.z = math.sin(angle / 2)
            out.transform.rotation.w = math.cos(angle / 2)
            self.tf.sendTransform(out)


def main():
    rclpy.init()
    node = GroundTruthLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
