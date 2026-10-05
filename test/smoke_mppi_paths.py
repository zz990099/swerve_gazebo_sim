"""Exercise curved paths against actual cylinders and monitor independent truth."""

import argparse
import math

import rclpy
from geometry_msgs.msg import Pose2D
from obstacle_fixture import OBSTACLES, clearance
from smoke_chassis import seconds
from smoke_mppi import Probe
from swerve_gazebo_sim.msg import CircleObstacle


class PathProbe(Probe):
    def __init__(self, family):
        super().__init__(family)
        self.required_modes = (0, 1)
        self.obstacles = [CircleObstacle(x=x, y=y, radius=r) for x, y, r in OBSTACLES]
        self.previous_truth = None
        self.min_clearance = math.inf

    def check_truth(self):
        truth = self.truth.latest
        if truth is None:
            return
        stamp = seconds(truth.header.stamp)
        position = (truth.pose.pose.position.x, truth.pose.pose.position.y)
        if self.previous_truth is not None:
            previous_stamp, previous_position = self.previous_truth
            assert stamp >= previous_stamp, "Physical truth time moved backwards"
            if stamp > previous_stamp:
                assert stamp - previous_stamp <= 0.15, "Physical truth sampling gap"
            value = clearance(previous_position, position)
            self.min_clearance = min(self.min_clearance, value)
            assert value > 0, f"Inflated physical obstacle clearance violated: {value}"
        self.previous_truth = stamp, position
        if self.state is not None:
            assert (
                seconds(self.state.header.stamp) - stamp < 0.15
            ), "Stale physical truth"

    def wait(self, condition, timeout=30, healthy=False):
        def monitored():
            self.check_truth()
            return condition()

        return super().wait(monitored, timeout, healthy)

    def stop_and_wait(self):
        assert self.service(self.stop_client).success
        self.wait(lambda: self.state.fault and self.stopped())

    def curve(self):
        self.stop_and_wait()
        p = self.state.pose
        truth = self.truth.latest.pose.pose.position
        x0, y0 = truth.x, truth.y
        self.path_id += 1
        self.path = [
            Pose2D(x=p.x + 2 * math.sin(a), y=p.y + 2 * (1 - math.cos(a)), theta=a)
            for a in (0.5 * i / 20 for i in range(21))
        ]
        self.arm()
        self.wait(
            lambda: self.planner.path_id == self.path_id and self.planner.goal_reached,
            timeout=60,
            healthy=True,
        )
        self.hold(0.4)
        physical = self.truth.latest.pose.pose
        error = math.hypot(
            physical.position.x - x0 - 2 * math.sin(0.5),
            physical.position.y - y0 - 2 * (1 - math.cos(0.5)),
        )
        yaw = 2 * math.atan2(physical.orientation.z, physical.orientation.w)
        assert error < 0.12, f"Physical curved goal error: {error}"
        assert abs(math.atan2(math.sin(yaw - 0.5), math.cos(yaw - 0.5))) < 0.12
        assert self.stopped()
        print(f"COMPLETE curved corridor physical_error={error:.4f}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gazebo-version", choices=("gz", "ign"), required=True)
    parser.add_argument("--metrics", default=None)
    args = parser.parse_args()
    rclpy.init()
    node = PathProbe(args.gazebo_version)
    try:
        node.wait(
            lambda: node.state is not None
            and node.planner is not None
            and node.arm_client.service_is_ready()
            and node.stop_client.service_is_ready()
            and node.contexts.get_subscription_count() == 1
            and node.stopped(),
            timeout=90,
        )
        # Perception and static world share the initial odometry/world origin.
        physical = node.truth.latest.pose.pose
        assert math.hypot(physical.position.x, physical.position.y) < 0.02
        assert math.hypot(node.state.pose.x, node.state.pose.y) < 0.02
        assert abs(node.state.pose.theta) < 0.02
        assert (
            abs(2 * math.atan2(physical.orientation.z, physical.orientation.w)) < 0.02
        )
        node.task(-0.6, 0, 0, 0)
        node.task(0.6, 0, 0, 0)
        node.curve()
        node.heading_policy = 1  # GoalOnly: translate then settle final heading.
        node.task(0.5, 0, 0.4, 1)
        node.stop_and_wait()
        p = node.state.pose
        node.path_id += 1
        node.path = [
            Pose2D(x=p.x, y=p.y, theta=p.theta),
            Pose2D(x=p.x + 1, y=p.y, theta=p.theta),
        ]
        node.arm()
        node.wait(
            lambda: math.hypot(
                node.truth.latest.twist.twist.linear.x,
                node.truth.latest.twist.twist.linear.y,
            )
            > 0.06,
            healthy=True,
        )
        # Invalid perception must immediately cancel; fresh input cannot auto-rearm.
        node.obstacles[0].radius = math.nan
        node.wait(lambda: node.state.fault and node.planner.phase == 3)
        node.wait(node.stopped)
        session = node.state.session_id
        node.obstacles[0].radius = OBSTACLES[0][2]
        node.hold(0.5, healthy=False)
        assert node.state.fault and node.state.session_id == session and node.stopped()
        p = node.state.pose
        node.path_id += 1
        node.path = [Pose2D(x=p.x, y=p.y, theta=p.theta)] * 2
        node.arm()
        node.hold(0.4)
        assert node.state.session_id > session and node.stopped()
        assert node.max_compute < 0.075
        node.report_model_metrics(args.metrics)
        print(
            f"PASS: reverse, curved physical corridor, terminal yaw, malformed obstacles, latched stop and recovery; min_clearance={node.min_clearance:.4f}m max_compute={node.max_compute:.4f}s",
            flush=True,
        )
    finally:
        node.streaming = False
        node.report_partial_metrics(args.metrics)
        node.truth.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
