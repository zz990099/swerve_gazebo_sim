"""Physical smoke test: run against demo_ign.launch.py / demo_gz.launch.py publish_ground_truth:=true.

Uses simulation time for segments and wall time only for bounded test deadlines.
Requires a clear world; sends motion commands to the selected simulation robot.
"""

import argparse
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def pose(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return (
        position.x,
        position.y,
        math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
        ),
    )


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def relative(start, finish):
    delta_x = finish[0] - start[0]
    delta_y = finish[1] - start[1]
    return (
        math.cos(start[2]) * delta_x + math.sin(start[2]) * delta_y,
        -math.sin(start[2]) * delta_x + math.cos(start[2]) * delta_y,
        angle_diff(finish[2], start[2]),
    )


class Probe(Node):
    def __init__(self, namespace, peer):
        super().__init__(
            "swerve_motion_probe",
            namespace=namespace,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.truth = None
        self.odom = None
        self.peer = None
        self.peer_odom = None
        self.joint_state = None
        self.drive_status = None
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.create_subscription(
            Odometry, "ground_truth/odom", lambda msg: setattr(self, "truth", msg), 10
        )
        self.create_subscription(
            Odometry, "odom", lambda msg: setattr(self, "odom", msg), 10
        )
        self.create_subscription(
            JointState,
            "joint_states",
            lambda msg: setattr(self, "joint_state", msg),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "drive_status",
            lambda msg: setattr(self, "drive_status", msg),
            10,
        )
        if peer:
            self.create_subscription(
                Odometry,
                "/" + peer.strip("/") + "/ground_truth/odom",
                lambda msg: setattr(self, "peer", msg),
                10,
            )
            self.create_subscription(
                Odometry,
                "/" + peer.strip("/") + "/odom",
                lambda msg: setattr(self, "peer_odom", msg),
                10,
            )

    def run_for(self, duration, velocity=None):
        start = self.get_clock().now().nanoseconds * 1e-9
        deadline = time.monotonic() + 30 + duration * 8
        samples = []
        while self.get_clock().now().nanoseconds * 1e-9 - start < duration:
            if time.monotonic() > deadline:
                raise AssertionError("Simulation clock stalled")
            if velocity is not None:
                cmd = TwistStamped()
                cmd.twist.linear.x = velocity[0]
                cmd.twist.linear.y = velocity[1]
                cmd.twist.angular.z = velocity[2]
                self.publisher.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.drive_status is not None:
                assert all(
                    status.level != DiagnosticStatus.ERROR
                    for status in self.drive_status.status
                ), "Controller transition fault"
            for msg in (self.truth, self.odom, self.peer, self.peer_odom):
                if msg is not None:
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    assert (
                        now - stamp < 0.5
                    ), "Stale odometry: a robot or bridge stopped"
            if (
                self.truth is not None
                and self.get_clock().now().nanoseconds * 1e-9 - start > duration - 1
            ):
                v = self.truth.twist.twist
                samples.append((v.linear.x, v.linear.y, v.angular.z))
            time.sleep(0.002)
        return (
            [sum(v[i] for v in samples) / len(samples) for i in range(3)]
            if samples
            else None
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="")
    parser.add_argument("--stationary-peer", default="")
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args.namespace, args.stationary_peer)
    try:
        deadline = time.monotonic() + 60
        while (
            node.truth is None
            or node.odom is None
            or node.joint_state is None
            or node.drive_status is None
            or (args.stationary_peer and (node.peer is None or node.peer_odom is None))
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                raise AssertionError(
                    "Missing joint state, wheel odometry or Gazebo ground truth"
                )
        node.run_for(2, (0.0, 0.0, 0.0))
        prefix = (
            args.namespace.strip("/").replace("/", "_") + "_" if args.namespace else ""
        )
        assert node.odom.header.frame_id == prefix + "odom"
        assert node.odom.child_frame_id == prefix + "base_footprint"
        node.tf_buffer.lookup_transform(
            prefix + "odom", prefix + "fl_wheel_link", rclpy.time.Time()
        )
        if args.stationary_peer:
            peer_prefix = args.stationary_peer.strip("/").replace("/", "_") + "_"
            assert node.peer_odom.header.frame_id == peer_prefix + "odom"
            node.tf_buffer.lookup_transform(
                peer_prefix + "odom", peer_prefix + "fl_wheel_link", rclpy.time.Time()
            )
        peer_start = pose(node.peer) if node.peer is not None else None
        for name, target in [
            ("forward", (0.3, 0.0, 0.0)),
            ("reverse", (-0.3, 0.0, 0.0)),
            ("left", (0.0, 0.25, 0.0)),
            ("right", (0.0, -0.25, 0.0)),
            ("diagonal_crab", (0.2, 0.15, 0.0)),
            ("spin_ccw", (0.0, 0.0, 0.35)),
            ("spin_cw", (0.0, 0.0, -0.35)),
            ("differential_turn", (0.25, 0.0, 0.2)),
        ]:
            node.run_for(1, (0.0, 0.0, 0.0))
            truth_start = pose(node.truth)
            odom_start = pose(node.odom)
            measured = node.run_for(6, target)
            truth_delta = relative(truth_start, pose(node.truth))
            odom_delta = relative(odom_start, pose(node.odom))
            position_error = math.hypot(
                truth_delta[0] - odom_delta[0], truth_delta[1] - odom_delta[1]
            )
            yaw_error = abs(angle_diff(truth_delta[2], odom_delta[2]))
            print(
                f"{name}: target={target} measured={measured} "
                f"odom_error={position_error:.4f}m/{yaw_error:.4f}rad",
                flush=True,
            )
            assert measured is not None
            assert all(
                abs(a - b) < limit
                for a, b, limit in zip(measured, target, (0.08, 0.08, 0.12))
            ), name
            assert position_error < 0.25 and yaw_error < 0.2, name + " odometry"

        modes = {
            "differential": (0.25, 0.0, 0.2),
            "crab": (0.2, 0.15, 0.0),
            "spin": (0.0, 0.0, 0.35),
        }
        for first, source_command in modes.items():
            for second, target_command in modes.items():
                if first == second:
                    continue
                node.run_for(3, source_command)
                # No zero command between modes: exercise actual direct switches.
                measured = node.run_for(4, target_command)
                assert all(
                    abs(actual - target) < tolerance
                    for actual, target, tolerance in zip(
                        measured, target_command, (0.08, 0.08, 0.12)
                    )
                ), f"Direct transition {first} -> {second} failed"
                values = {
                    item.key: item.value for item in node.drive_status.status[0].values
                }
                assert values["active_mode"] == second
                assert values["phase"] == "active"
                print(
                    f"direct_transition: {first} -> {second} measured={measured}",
                    flush=True,
                )

        for pending in ("crab", "spin"):
            node.run_for(3, (0.25, 0.0, 0.0))
            node.run_for(0.25, modes[pending])
            measured = node.run_for(3, (0.25, 0.0, 0.0))
            assert abs(measured[0] - 0.25) < 0.08
            assert abs(measured[1]) < 0.08 and abs(measured[2]) < 0.12
            print(
                f"interrupted_transition: {pending} -> differential passed", flush=True
            )

        stopped = node.run_for(2, None)
        print(f"timeout_stop: measured={stopped}", flush=True)
        assert all(abs(v) < 0.03 for v in stopped), "Watchdog failed to stop chassis"

        steering_names = [
            f"{prefix}{corner}_steering_joint" for corner in ("fl", "fr", "rl", "rr")
        ]
        steering_positions = {
            name: node.joint_state.position[index]
            for index, name in enumerate(node.joint_state.name)
            if name in steering_names
        }
        assert set(steering_positions) == set(steering_names)
        assert all(
            abs(steering_positions[name]) < 0.05 for name in steering_names
        ), "Steering did not return home after command timeout"
        print(
            f"timeout_steering={steering_positions}",
            flush=True,
        )
        if peer_start is not None:
            delta = relative(peer_start, pose(node.peer))
            print(f"stationary_peer_delta={delta}", flush=True)
            assert math.hypot(*delta[:2]) < 0.03 and abs(delta[2]) < 0.03
        print("PASS: all physical motion checks", flush=True)
    finally:
        node.publisher.publish(TwistStamped())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
