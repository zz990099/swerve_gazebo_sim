"""External endpoint acceptance: SIGKILL the command publisher during motion.

Uses real joint feedback and independent Gazebo truth. No MPPI trajectory or
physical braking bound is certified by this low-speed endpoint smoke test.
"""

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from gazebo_truth import GazeboTruth
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, UInt64MultiArray


class Probe(Node):
    def __init__(self, namespace, family, worker=False):
        super().__init__(
            "endpoint_sender" if worker else "endpoint_probe",
            namespace=namespace,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.joints = None
        self.status = None
        self.create_subscription(
            JointState, "joint_states", lambda msg: setattr(self, "joints", msg), 10
        )
        self.create_subscription(
            UInt64MultiArray,
            "guarded_joint_controller/status",
            lambda msg: setattr(self, "status", msg.data),
            10,
        )
        self.publisher = (
            self.create_publisher(
                Float64MultiArray, "guarded_joint_controller/commands", 1
            )
            if worker
            else None
        )
        self.truth = (
            None
            if worker
            else GazeboTruth(
                "/"
                + "/".join(filter(None, (namespace.strip("/"), "ground_truth/odom"))),
                family,
            )
        )
        self.prefix = namespace.strip("/").replace("/", "_") + "_" if namespace else ""

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def encoders(self):
        indices = {name: index for index, name in enumerate(self.joints.name)}
        angles, speeds = [], []
        for corner in ("fl", "fr", "rl", "rr"):
            angles.append(
                self.joints.position[indices[f"{self.prefix}{corner}_steering_joint"]]
            )
            speeds.append(
                self.joints.velocity[indices[f"{self.prefix}{corner}_wheel_joint"]]
            )
        return angles, speeds

    def send(self, session, sequence, arm, angles, speed):
        now = self.now()
        self.publisher.publish(
            Float64MultiArray(
                data=[
                    float(session),
                    float(sequence),
                    float(arm),
                    now,
                    now + 0.03,
                    *angles,
                    *([speed] * 4),
                    1.0,
                ]
            )
        )

    def wait(self, condition, timeout=30):
        deadline = time.monotonic() + timeout
        while True:
            rclpy.spin_once(self, timeout_sec=0.005)
            if condition():
                return
            if time.monotonic() > deadline:
                raise AssertionError("Endpoint/feedback acceptance deadline exceeded")
            time.sleep(0.001)

    def stopped(self):
        if self.joints is None or self.truth.latest is None:
            return False
        _, speeds = self.encoders()
        v = self.truth.latest.twist.twist
        return (
            max(map(abs, speeds)) < 0.05 and math.hypot(v.linear.x, v.linear.y) < 0.03
        )

    def close(self):
        if self.truth is not None:
            self.truth.close()
        self.destroy_node()


def worker(node, session, no_arm):
    node.wait(
        lambda: node.joints is not None
        and node.status is not None
        and node.now() > 0
        and node.publisher.get_subscription_count() == 1
    )
    angles, _ = node.encoders()
    sequence = 1
    if not no_arm:
        node.send(session, sequence, True, angles, 0.0)
    arm_stamp = node.now()
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.002)
        sequence += 1
        # Immediately stream zero targets pending the arm acknowledgement, so
        # status delivery does not consume the short command validity window.
        armed = node.status[0] == session and node.status[2] == 0
        speed = 2.0 if no_arm else min(2.0, max(0.0, node.now() - arm_stamp) * 4)
        node.send(session, sequence, False, angles, speed if armed or no_arm else 0.0)
        time.sleep(0.002)
    raise AssertionError("Publisher was not killed by the acceptance test")


def acceptance(node, args):
    node.wait(
        lambda: node.joints is not None
        and node.status is not None
        and node.truth.latest is not None
        and node.now() > 0
    )
    assert node.get_publishers_info_by_topic("wheel_controller/commands") == []
    assert node.get_publishers_info_by_topic("steering_controller/commands") == []
    processes = []

    def start(session, no_arm=False):
        command = [
            sys.executable,
            str(Path(__file__)),
            "--worker",
            "--session",
            str(session),
            "--namespace",
            args.namespace,
            "--gazebo-version",
            args.gazebo_version,
        ]
        if no_arm:
            command.append("--no-arm")
        process = subprocess.Popen(command)
        processes.append(process)
        return process

    def moving(session):
        return (
            node.status[0] == session
            and node.status[2] == 0
            and min(node.encoders()[1]) > 1.5
            and node.truth.latest.twist.twist.linear.x > 0.12
        )

    def kill_and_check(process):
        process.kill()  # SIGKILL: no sender shutdown callback can emit a stop.
        process.wait(timeout=5)
        node.wait(lambda: node.status[2] == 1 and node.stopped(), timeout=15)
        position = node.truth.latest.pose.pose.position
        start_x, start_y, start_sim = position.x, position.y, node.now()
        node.wait(lambda: node.now() - start_sim >= 0.5)
        position = node.truth.latest.pose.pose.position
        assert math.hypot(position.x - start_x, position.y - start_y) < 0.03

    try:
        first_session = node.status[0] + 1
        first = start(first_session)
        node.wait(lambda: moving(first_session))
        kill_and_check(first)
        stale = start(first_session, no_arm=True)
        start_sim = node.now()
        node.wait(lambda: node.now() - start_sim >= 0.5)
        assert node.status[2] == 1 and node.stopped(), "Stale session restarted motion"
        stale.kill()
        stale.wait(timeout=5)
        recovered_session = first_session + 1
        recovered = start(recovered_session)
        node.wait(lambda: moving(recovered_session))
        kill_and_check(recovered)
        print(
            "PASS: external publisher SIGKILL stop, stale-session rejection and recovery"
        )
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="")
    parser.add_argument("--gazebo-version", choices=("ign", "gz"), default="ign")
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--no-arm", action="store_true")
    parser.add_argument("--session", type=int, default=1)
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args.namespace, args.gazebo_version, args.worker)
    try:
        if args.worker:
            worker(node, args.session, args.no_arm)
        else:
            acceptance(node, args)
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
