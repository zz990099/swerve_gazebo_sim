"""Exercise the actual C++ node with a deterministic ROS clock and fake encoders."""

import subprocess
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from swerve_chassis_msgs.msg import ChassisState


def test_controller_watchdogs_and_feedback():
    rclpy.init()
    node = Node("continuous_controller_test")
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "swerve_chassis_controller",
            "swerve_controller",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "cmd_future_tolerance:=0.03",
            "-r",
            "__ns:=/controller_test",
        ]
    )
    clock = node.create_publisher(Clock, "/clock", 10)
    feedback = node.create_publisher(JointState, "/controller_test/joint_states", 10)
    command = node.create_publisher(TwistStamped, "/controller_test/cmd_vel", 10)
    states = []
    wheels = [0.0] * 4
    angles = [0.0] * 4
    node.create_subscription(
        ChassisState,
        "/controller_test/chassis_controller/state",
        states.append,
        rclpy.qos.qos_profile_sensor_data,
    )
    node.create_subscription(
        Float64MultiArray,
        "/controller_test/wheel_controller/commands",
        lambda m: wheels.__setitem__(slice(None), m.data),
        10,
    )
    node.create_subscription(
        Float64MultiArray,
        "/controller_test/steering_controller/commands",
        lambda m: angles.__setitem__(slice(None), m.data),
        10,
    )
    sim = [10.0]

    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def step(velocity=(0, 0, 0), advance=True, send_feedback=True, command_offset_ms=0):
        if advance:
            sim[0] += 0.02
        clk = Clock()
        clk.clock.sec = int(sim[0])
        clk.clock.nanosec = int((sim[0] % 1) * 1e9)
        clock.publish(clk)
        # Allow /clock to reach the controller before stamped input.
        rclpy.spin_once(node, timeout_sec=0.005)
        time.sleep(0.005)
        if command_offset_ms:
            deadline = time.monotonic() + 0.18
            while not states or stamp_ns(states[-1].header.stamp) != stamp_ns(clk.clock):
                assert time.monotonic() < deadline, "Controller did not receive /clock"
                rclpy.spin_once(node, timeout_sec=0.01)
        if send_feedback:
            j = JointState()
            j.header.stamp = clk.clock
            j.name = [f"{c}_steering_joint" for c in ("fl", "fr", "rl", "rr")] + [
                f"{c}_wheel_joint" for c in ("fl", "fr", "rl", "rr")
            ]
            j.position = list(angles) + [0.0] * 4
            j.velocity = [0.0] * 4 + list(wheels)
            feedback.publish(j)
        if velocity is not None:
            msg = TwistStamped()
            sec, nanosec = divmod(
                stamp_ns(clk.clock) + command_offset_ms * 1_000_000, 1_000_000_000
            )
            msg.header.stamp = Time(sec=sec, nanosec=nanosec)
            msg.header.frame_id = "base_footprint"
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = map(
                float, velocity
            )
            command.publish(msg)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.003)
        time.sleep(0.01)
        assert process.poll() is None

    def until(predicate, **kwargs):
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            step(**kwargs)
            if states and predicate(states[-1]):
                return
        raise AssertionError(states[-1] if states else "No state")

    try:
        until(lambda s: s.fault == 0 and s.feedback_valid)
        for offset in (15, 25):
            until(
                lambda s: s.fault == 0
                and s.command_fresh
                and stamp_ns(s.command_stamp) - stamp_ns(s.header.stamp) > 10_000_000,
                velocity=(0.12, 0, 0),
                command_offset_ms=offset,
            )
        last_accepted = states[-1].command_sequence
        until(lambda s: s.fault == 1, velocity=(0.12, 0, 0), command_offset_ms=35)
        assert states[-1].command_sequence == last_accepted
        assert max(map(abs, states[-1].target_wheel_speeds)) == 0
        until(lambda s: s.fault == 0)
        until(lambda s: abs(s.velocity.linear.y - 0.2) < 0.03, velocity=(0.3, 0.2, 0.1))
        until(lambda s: s.fault != 0, velocity=None)
        until(lambda s: max(map(abs, s.wheel_speeds)) < 0.05, velocity=None)
        until(lambda s: s.fault == 0)
        until(lambda s: s.velocity.linear.x > 0.15, velocity=(0.25, 0, 0))
        # Frozen simulation time cannot keep commands or encoder samples alive.
        until(lambda s: s.fault != 0, velocity=None, advance=False)
        assert max(map(abs, states[-1].target_wheel_speeds)) == 0
        until(lambda s: s.fault == 0)
        until(lambda s: s.fault == 2, send_feedback=False)
        until(lambda s: s.fault == 0)
        sim[0] -= 5
        step(velocity=None)
        until(lambda s: s.fault != 0, velocity=None)
        assert not states[-1].generator_valid
        until(lambda s: s.fault == 0)
    finally:
        process.terminate()
        process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
