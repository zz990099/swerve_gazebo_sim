"""Exercise explicit body/mode input on the physical simulator, without MPPI planning."""

import argparse
import math
import time

import rclpy
from builtin_interfaces.msg import Time
from gazebo_truth import GazeboTruth
from geometry_msgs.msg import Pose2D, Vector3
from rclpy.node import Node
from rclpy.parameter import Parameter
from swerve_gazebo_sim.msg import ChassisCommand, ChassisContext, ChassisState


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def seconds(t):
    return t.sec + t.nanosec * 1e-9


class Probe(Node):
    def __init__(self, family):
        super().__init__(
            "chassis_probe", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.state = None
        self.create_subscription(
            ChassisState, "chassis_controller/state", self.observe, 1
        )
        self.commands = self.create_publisher(
            ChassisCommand, "chassis_controller/command", 1
        )
        self.contexts = self.create_publisher(
            ChassisContext, "chassis_controller/context", 1
        )
        self.truth = GazeboTruth("/ground_truth/odom", family)
        self.session = 1
        self.sequence = 0
        self.last_boundary = None
        self.path = [Pose2D(x=0.0), Pose2D(x=3.0)]
        self.mode = 0
        self.velocity = Vector3()
        self.request = None
        self.streaming = False
        self.timer = self.create_timer(0.01, self.send)

    def observe(self, state):
        self.state = state
        if self.streaming:
            # Keep the actual observation stamp; publish without waiting behind
            # the command timer. Loss of feedback cannot manufacture freshness.
            self.publish_context(seconds(state.header.stamp))

    def publish_context(self, now):
        msg = ChassisContext()
        msg.header.stamp = stamp(min(now, seconds(self.state.header.stamp)))
        msg.header.frame_id = "odom"
        msg.path = self.path
        self.contexts.publish(msg)

    def packet(self, now, boundary, arm=False):
        msg = ChassisCommand()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = stamp(now)
        msg.source_stamp = self.state.header.stamp
        msg.execute_at = stamp(boundary)
        msg.valid_until = stamp(boundary + 0.025)
        msg.session_id = self.session
        self.sequence += 1
        msg.sequence = self.sequence
        msg.authorized = True
        msg.arm = arm
        msg.mode = self.mode
        msg.velocity = self.velocity
        msg.source_path = self.path
        if self.request:
            msg.has_mode_request = True
            msg.request_id, msg.requested_mode, msg.entry_velocity = self.request
        return msg

    def send(self):
        if not self.streaming or self.state is None:
            return
        now = max(
            self.get_clock().now().nanoseconds * 1e-9,
            seconds(self.state.header.stamp),
        )
        if self.state.fault:
            return
        boundary = seconds(self.state.next_execute_at)
        # Send one fresh command ahead of the observed execution boundary.
        if boundary == self.last_boundary or boundary < now:
            return
        self.commands.publish(self.packet(now, boundary))
        self.last_boundary = boundary

    def wait(self, condition, timeout=30, healthy=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)
            if healthy and self.state and self.state.fault:
                raise AssertionError(f"Chassis fault: {self.state}")
            if condition():
                return
        raise AssertionError(f"Chassis deadline exceeded: {self.state}")

    def hold(self, duration):
        end = seconds(self.state.header.stamp) + duration
        self.wait(lambda: seconds(self.state.header.stamp) >= end, healthy=True)

    def arm(self):
        # Command and context use separate DDS topics. Warm the context stream
        # while disarmed instead of assuming publication order is arrival order.
        self.streaming = True
        self.last_boundary = None
        warm_until = seconds(self.state.header.stamp) + 0.1
        self.wait(lambda: seconds(self.state.header.stamp) >= warm_until)
        for attempt in range(5):
            now = max(
                self.get_clock().now().nanoseconds * 1e-9,
                seconds(self.state.header.stamp),
            )
            print(
                f"ARM attempt={attempt} session={self.session} issued={now:.6f} "
                f"source={seconds(self.state.header.stamp):.6f}",
                flush=True,
            )
            self.commands.publish(self.packet(now, now, arm=True))
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.005)
                if self.state.session_id == self.session and not self.state.fault:
                    print(f"ACK session={self.session}", flush=True)
                    return
            print(f"ARM rejected: {self.state}", flush=True)
            self.session += 1
        raise AssertionError(f"Unable to arm stopped chassis: {self.state}")

    def switch(self, mode, request_id, entry):
        self.mode = mode
        self.velocity = Vector3()
        self.request = (request_id, mode, entry)
        self.wait(
            lambda: self.state.actual_mode == mode
            and self.state.confirmed
            and self.state.request_id == request_id,
            healthy=True,
        )
        assert self.state.has_accepted_request
        assert max(map(abs, self.state.wheel_speeds)) < 0.0051
        print(f"CONFIRMED mode={mode} request={request_id}", flush=True)
        self.request = None
        self.hold(0.2)

    def stopped(self):
        if self.state is None or self.truth.latest is None:
            return False
        v = self.truth.latest.twist.twist
        return (
            max(map(abs, self.state.wheel_speeds)) < 0.0051
            and math.hypot(v.linear.x, v.linear.y) < 0.03
            and abs(v.angular.z) < 0.04
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gazebo-version", choices=("gz", "ign"), required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args.gazebo_version)
    try:
        node.wait(
            lambda: node.state is not None
            and node.commands.get_subscription_count() == 1
            and node.contexts.get_subscription_count() == 1
            and node.stopped(),
            timeout=90,
        )
        node.arm()
        node.switch(2, 1, Vector3(x=0.15))
        start = node.truth.latest.pose.pose.position.x
        node.velocity = Vector3(x=0.15)
        node.hold(1.5)
        assert node.state.actual_mode == 2, "Crab forward was reclassified"
        assert node.truth.latest.pose.pose.position.x - start > 0.08
        node.velocity = Vector3()
        node.hold(0.7)
        node.wait(node.stopped, healthy=True)
        assert node.state.actual_mode == 2
        node.switch(1, 2, Vector3(z=0.2))
        angles = list(node.state.steering_angles)
        node.hold(0.4)
        assert (
            max(abs(a - b) for a, b in zip(angles, node.state.steering_angles)) < 0.05
        )
        assert max(map(abs, angles)) > 0.2, "zero speed returned steering to home"
        node.velocity = Vector3(z=0.2)
        node.hold(1.0)
        assert abs(node.truth.latest.twist.twist.angular.z) > 0.05
        # Stop all command/context publication: independent execution must fault.
        node.streaming = False
        node.wait(lambda: node.state.fault)
        node.wait(node.stopped)
        node.session += 1
        node.velocity = Vector3()
        node.arm()
        node.hold(0.3)
        assert node.state.actual_mode == 1
        print(
            "PASS: explicit Crab/Spin, retained zero-mode/steering, physical motion, loss and recovery"
        )
    finally:
        node.streaming = False
        node.truth.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
