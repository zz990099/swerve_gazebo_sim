"""Physical continuous chassis acceptance using independent Gazebo poses."""

import argparse
import json
import math
import time

import rclpy
from gazebo_truth import GazeboTruth
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from swerve_chassis_msgs.msg import ChassisState


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
    publisher = node.create_publisher(TwistStamped, "/cmd_vel", 1)
    truth = GazeboTruth("/ground_truth/odom", args.gazebo_version)
    command = TwistStamped()
    command.header.frame_id = "base_footprint"
    previous_stamp = [None]

    def tick(publish=True):
        stamp = node.get_clock().now().nanoseconds
        if publish and stamp != previous_stamp[0]:
            command.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(command)
            previous_stamp[0] = stamp
        rclpy.spin_once(node, timeout_sec=0.005)
        time.sleep(0.005)

    def wait(predicate, timeout=15, publish=True):
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

    def set_twist(values):
        command.twist.linear.x, command.twist.linear.y, command.twist.angular.z = map(
            float, values
        )

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def yaw(msg):
        q = msg.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    reports = []
    try:
        wait(
            lambda: state[0] is not None
            and truth.latest is not None
            and publisher.get_subscription_count() > 0,
            timeout=60,
        )
        wait(lambda: state[0].fault == 0)
        for target in [
            (0.25, 0, 0),
            (0, 0.2, 0),
            (0, 0, 0.3),
            (0.3, 0.15, 0.25),
            (-0.2, 0.01, 0),
            (-0.2, -0.01, 0),
            (0.075, -0.09, 0.3),
        ]:
            set_twist(target)
            wait(
                lambda target=target: state[0].fault == 0
                and max(
                    abs(a - b)
                    for a, b in zip(
                        (
                            state[0].velocity.linear.x,
                            state[0].velocity.linear.y,
                            state[0].velocity.angular.z,
                        ),
                        target,
                    )
                )
                < 0.04
            )
            run(1.0)
            start = truth.latest
            run(1.5)
            end = truth.latest
            dt = stamp(end) - stamp(start)
            assert dt > 1.0, "Truth stopped updating"
            # Integrate the requested body twist analytically from independent start pose.
            a = target[2] * dt
            s = math.sin(a) / a if abs(a) > 1e-8 else 1
            c = (1 - math.cos(a)) / a if abs(a) > 1e-8 else 0
            bx, by = dt * (s * target[0] - c * target[1]), dt * (
                c * target[0] + s * target[1]
            )
            heading = yaw(start)
            expected_x = (
                start.pose.pose.position.x
                + math.cos(heading) * bx
                - math.sin(heading) * by
            )
            expected_y = (
                start.pose.pose.position.y
                + math.sin(heading) * bx
                + math.cos(heading) * by
            )
            position_error = math.hypot(
                end.pose.pose.position.x - expected_x,
                end.pose.pose.position.y - expected_y,
            )
            heading_error = abs(math.remainder(yaw(end) - heading - a, 2 * math.pi))
            reports.append(
                {
                    "target": target,
                    "position_error": position_error,
                    "heading_error": heading_error,
                    "residual": state[0].consistency_residual,
                }
            )
            print(json.dumps(reports[-1]), flush=True)
            assert position_error < 0.15, reports[-1]
            assert heading_error < 0.10, reports[-1]
            angles = tuple(state[0].target_steering)
            set_twist((0, 0, 0))
            run(1)
            assert max(map(abs, state[0].wheel_speeds)) < 0.05
            assert (
                max(abs(a - b) for a, b in zip(angles, state[0].target_steering)) < 0.01
            )
        set_twist((0.25, 0, 0))
        run(1)
        wait(lambda: state[0].fault != 0, publish=False)
        wait(lambda: max(map(abs, state[0].wheel_speeds)) < 0.05, publish=False)
        set_twist((0, 0, 0))
        wait(lambda: state[0].fault == 0)
        print(
            "PASS: continuous physical motion, combination, boundaries, single-wheel zero, zero hold, timeout/recovery",
            flush=True,
        )
    finally:
        truth.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
