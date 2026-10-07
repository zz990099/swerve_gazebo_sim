"""Physical chassis acceptance using explicit velocity/mode commands only."""

import argparse
import math
import time

import rclpy
from gazebo_truth import GazeboTruth
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from swerve_gazebo_sim.msg import ChassisCommand, ChassisState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gazebo-version", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node(
        "chassis_acceptance",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    state = [None]
    node.create_subscription(
        ChassisState,
        "/chassis_controller/state",
        lambda msg: state.__setitem__(0, msg),
        qos_profile_sensor_data,
    )
    publisher = node.create_publisher(ChassisCommand, "/chassis_controller/command", 1)
    truth = GazeboTruth("/ground_truth/odom", args.gazebo_version)
    command = ChassisCommand()
    command.header.frame_id = "base_footprint"
    command.session_id = 1

    def tick(publish=True):
        if publish:
            command.header.stamp = node.get_clock().now().to_msg()
            command.sequence += 1
            publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.005)
        time.sleep(0.005)

    def wait(predicate, timeout=12, publish=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            tick(publish)
            if predicate():
                return
        raise AssertionError(f"Condition timed out; state={state[0]}")

    def run(seconds):
        until = node.get_clock().now().nanoseconds * 1e-9 + seconds
        wait(lambda: node.get_clock().now().nanoseconds * 1e-9 >= until)
        assert state[0].fault == 0, state[0]

    def velocity(mode):
        return ((0.25, 0.0, 0.05), (0.0, 0.0, 0.3), (0.0, 0.2, 0.0))[mode]

    def set_twist(twist, values):
        twist.linear.x, twist.linear.y, twist.angular.z = values

    try:
        wait(
            lambda: state[0] is not None
            and truth.latest is not None
            and publisher.get_subscription_count() > 0,
            timeout=60,
            publish=False,
        )
        wait(lambda: max(map(abs, state[0].wheel_speeds)) < 0.05, publish=False)
        command.reset = True
        # Reset is a one-shot transaction; retries use a strictly newer session.
        for _ in range(20):
            command.session_id += 1
            tick()
            for _ in range(10):
                tick(False)
            if state[0].confirmed and state[0].session_id == command.session_id:
                break
        assert state[0].confirmed, state[0]
        command.reset = False
        for target in (1, 2, 0, 2, 1, 0):
            command.request_id += 1
            command.mode = target
            set_twist(command.velocity, (0.0, 0.0, 0.0))
            set_twist(command.entry_velocity, velocity(target))
            wait(
                lambda target=target: state[0].request_id == command.request_id
                and state[0].actual_mode == target
                and state[0].confirmed
            )
            start = truth.latest
            set_twist(command.velocity, velocity(target))
            run(1.5)
            end = truth.latest
            assert end.header.stamp != start.header.stamp, "Truth stopped updating"
            if target == 1:
                a, b = start.pose.pose.orientation, end.pose.pose.orientation
                assert math.hypot(a.z - b.z, a.w - b.w) > 0.03, "No physical rotation"
            else:
                a, b = start.pose.pose.position, end.pose.pose.position
                assert (
                    math.hypot(a.x - b.x, a.y - b.y) > 0.08
                ), "No physical translation"
            angles = tuple(state[0].steering_angles)
            set_twist(command.velocity, (0.0, 0.0, 0.0))
            run(0.6)
            assert state[0].actual_mode == target
            assert (
                max(abs(a - b) for a, b in zip(angles, state[0].steering_angles)) < 0.07
            )
        set_twist(command.velocity, velocity(0))
        run(0.6)
        wait(lambda: state[0].fault != 0, publish=False)
        wait(lambda: max(map(abs, state[0].wheel_speeds)) < 0.05, publish=False)
        print(
            "PASS: six mode transitions, physical motion, zero hold, command loss",
            flush=True,
        )
    finally:
        truth.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
