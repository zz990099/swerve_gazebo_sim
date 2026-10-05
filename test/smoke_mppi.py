"""Verify actual MPPI planning, measured completion and context-loss stop in Gazebo."""

import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from gazebo_truth import GazeboTruth
from geometry_msgs.msg import Pose2D
from model_metrics import ModelMetrics, acceptance, statistics
from rclpy.node import Node
from rclpy.parameter import Parameter
from smoke_chassis import seconds, snapshot_qos
from std_srvs.srv import Trigger
from swerve_gazebo_sim.msg import ChassisContext, ChassisState, PlannerState


class Probe(Node):
    def __init__(self, family):
        super().__init__(
            "mppi_probe", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.state = None
        self.model_metrics = ModelMetrics()
        self.metrics_reported = False
        self.required_modes = (0, 1, 2)
        self.last_feedback_wall = None
        self.last_feedback_stamp = None
        self.feedback_publish_skips = 0
        self.timing_samples = {
            name: deque(maxlen=30000)
            for name in (
                "feedback_wall_gap_seconds",
                "feedback_sim_gap_seconds",
                "context_age_seconds",
                "command_receipt_age_seconds",
                "command_source_age_seconds",
                "command_schedule_slack_seconds",
                "planning_context_age_seconds",
                "schedule_margin_seconds",
            )
        }
        self.planner = None
        self.create_subscription(
            ChassisState, "chassis_controller/state", self.observe, snapshot_qos()
        )
        self.create_subscription(
            PlannerState, "mppi_planner/state", self.observe_planner, snapshot_qos()
        )
        self.contexts = self.create_publisher(
            ChassisContext, "mppi_planner/context", snapshot_qos()
        )
        self.arm_client = self.create_client(Trigger, "mppi_planner/arm")
        self.stop_client = self.create_client(Trigger, "mppi_planner/stop")
        # 50 Hz truth over the runner's bounded 300 s lifetime. Analysis runs
        # only at stopped boundaries, outside the context-publication loop.
        self.truth = GazeboTruth("/ground_truth/odom", family, history_length=16000)
        self.path = [Pose2D(), Pose2D(x=0.7)]
        self.path_id = 1
        self.obstacles = []
        self.heading_policy = 0
        self.streaming = True
        self.max_compute = 0
        self.driving_modes = set()
        # Publish immediately from each new observation; a separate timer can
        # add avoidable callback-queue delay while reusing an older source stamp.

    def observe(self, state):
        observed_wall = time.monotonic()
        observed_stamp = seconds(state.header.stamp)
        if not state.fault and self.last_feedback_wall is not None:
            self.timing_samples["feedback_wall_gap_seconds"].append(
                observed_wall - self.last_feedback_wall
            )
            if observed_stamp >= self.last_feedback_stamp:
                self.timing_samples["feedback_sim_gap_seconds"].append(
                    observed_stamp - self.last_feedback_stamp
                )
        self.last_feedback_wall = observed_wall
        self.last_feedback_stamp = observed_stamp
        self.feedback_publish_skips = max(
            self.feedback_publish_skips, state.feedback_publish_skips
        )
        self.state = state
        self.publish_context()
        if not state.fault:
            for name in (
                "context_age_seconds",
                "command_receipt_age_seconds",
                "command_source_age_seconds",
                "command_schedule_slack_seconds",
            ):
                value = getattr(state, name)
                if value >= 0 and math.isfinite(value):
                    self.timing_samples[name].append(value)
        if state.fault:
            self.model_metrics.interrupt(
                state.session_id,
                state.header.stamp.sec * 1_000_000_000 + state.header.stamp.nanosec,
            )
        if state.has_prediction:

            def ns(stamp):
                return stamp.sec * 1_000_000_000 + stamp.nanosec

            def pose(value):
                return value.x, value.y, value.theta

            self.model_metrics.forecast(
                (state.session_id, state.sequence),
                ns(state.prediction_source_stamp),
                ns(state.prediction_stamp),
                pose(state.prediction_source_pose),
                pose(state.predicted_pose),
                mode=state.actual_mode,
                braking=(
                    math.hypot(state.predicted_velocity.x, state.predicted_velocity.y)
                    < math.hypot(
                        state.prediction_source_velocity.x,
                        state.prediction_source_velocity.y,
                    )
                    - 0.001
                    or abs(state.predicted_velocity.z)
                    < abs(state.prediction_source_velocity.z) - 0.001
                ),
            )
        if not state.fault and (
            math.hypot(state.velocity.x, state.velocity.y) > 0.03
            or abs(state.velocity.z) > 0.04
        ):
            self.driving_modes.add(state.actual_mode)

    def observe_planner(self, state):
        self.planner = state
        if state.authorized:
            for name in ("planning_context_age_seconds", "schedule_margin_seconds"):
                value = getattr(state, name)
                if value >= 0 and math.isfinite(value):
                    self.timing_samples[name].append(value)
        self.max_compute = max(self.max_compute, state.compute_seconds)

    def publish_context(self):
        if self.state is None or not self.streaming:
            return
        context = ChassisContext()
        context.header.frame_id = "odom"
        context.header.stamp = self.state.header.stamp
        context.path_id = self.path_id
        context.path = self.path
        context.obstacles = self.obstacles
        context.heading_policy = self.heading_policy
        self.contexts.publish(context)

    def wait(self, condition, timeout=30, healthy=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)
            if healthy and self.state and self.state.fault:
                raise AssertionError(
                    f"Execution fault: {self.state}; planner={self.planner}"
                )
            if condition():
                return
        raise AssertionError(
            f"MPPI timeout: state={self.state}; planner={self.planner}"
        )

    def collect_model_metrics(self):
        if not self.model_metrics.pending:
            return
        observations = self.truth.history
        if not observations:
            return
        earliest = min(forecast[0] for forecast in self.model_metrics.pending.values())
        history = []
        # Include one sample at/before the earliest source for interpolation.
        # This runs while stopped, rather than converting truth on every spin.
        for observation in reversed(observations):
            stamp = observation.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            pose = observation.pose.pose
            history.append(
                (
                    stamp_ns,
                    pose.position.x,
                    pose.position.y,
                    2 * math.atan2(pose.orientation.z, pose.orientation.w),
                )
            )
            if stamp_ns <= earliest:
                break
        history.reverse()
        self.model_metrics.collect(history)

    def report_model_metrics(self, destination, require_samples=True):
        self.collect_model_metrics()
        report = self.model_metrics.summary()
        report["max_compute_seconds"] = self.max_compute
        report["feedback_publish_skips"] = self.feedback_publish_skips
        report["sufficient_moving_samples"] = report["moving_intervals"] >= 20
        report["acceptance"] = acceptance(report, self.required_modes)
        report["timing"] = {
            name: {
                "samples": len(values),
                "min": min(values) if values else None,
                **statistics(values),
            }
            for name, values in self.timing_samples.items()
        }
        print("MODEL_METRICS: " + json.dumps(report, sort_keys=True), flush=True)
        if destination:
            Path(destination).write_text(json.dumps(report, indent=2) + "\n")
        self.metrics_reported = True
        if require_samples:
            assert report["acceptance"]["passed"], report["acceptance"]["errors"]

    def report_partial_metrics(self, destination):
        if not self.metrics_reported:
            try:
                self.report_model_metrics(destination, require_samples=False)
            except (ValueError, RuntimeError, OSError) as error:
                # Preserve the original physical/control failure and cleanup.
                print(f"MODEL_METRICS_UNAVAILABLE: {error}", flush=True)

    def hold(self, duration, healthy=True):
        end = seconds(self.state.header.stamp) + duration
        self.wait(lambda: seconds(self.state.header.stamp) >= end, healthy=healthy)

    def stopped(self):
        if self.state is None or self.truth.latest is None:
            return False
        v = self.truth.latest.twist.twist
        return (
            max(map(abs, self.state.wheel_speeds)) < 0.0051
            and math.hypot(v.linear.x, v.linear.y) < 0.03
            and abs(v.angular.z) < 0.04
        )

    def service(self, client):
        future = client.call_async(Trigger.Request())
        self.wait(future.done)
        return future.result()

    def arm(self):
        self.collect_model_metrics()
        self.streaming = True
        self.hold(0.2, healthy=False)
        for _ in range(5):
            response = self.service(self.arm_client)
            if response.success:
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.005)
                    if self.planner.phase == 2 and not self.state.fault:
                        return
            print(f"Arm retry: {response.message}; planner={self.planner}", flush=True)
            self.hold(0.2, healthy=False)
        raise AssertionError("Explicit stopped arm did not reach Running")

    def task(self, dx, dy, dyaw, mode):
        if self.planner.phase == 2:
            assert self.service(self.stop_client).success
            self.wait(lambda: self.state.fault and self.stopped())
        p = self.state.pose
        self.path_id += 1
        self.path = [
            Pose2D(x=p.x, y=p.y, theta=p.theta),
            Pose2D(x=p.x + dx, y=p.y + dy, theta=p.theta + dyaw),
        ]
        before = self.truth.latest.pose.pose
        x0, y0 = before.position.x, before.position.y
        yaw0 = 2 * math.atan2(before.orientation.z, before.orientation.w)
        self.driving_modes.clear()
        self.arm()
        self.wait(
            lambda: self.planner.path_id == self.path_id and self.planner.goal_reached,
            timeout=45,
            healthy=True,
        )
        self.hold(0.4)
        assert self.stopped(), "Planner completion must correspond to physical stopping"
        assert mode in self.driving_modes, (
            f"Expected driving mode {mode}, observed {self.driving_modes}; "
            f"terminal mode={self.state.actual_mode}"
        )
        assert self.planner.evaluated_rollouts == 0 or self.planner.authorized
        physical = self.truth.latest.pose.pose
        physical_yaw = 2 * math.atan2(physical.orientation.z, physical.orientation.w)
        position_error = math.hypot(
            physical.position.x - x0 - dx, physical.position.y - y0 - dy
        )
        yaw_error = abs(
            math.atan2(
                math.sin(physical_yaw - yaw0 - dyaw),
                math.cos(physical_yaw - yaw0 - dyaw),
            )
        )
        assert position_error < 0.12, f"Physical goal position error: {position_error}"
        assert yaw_error < 0.12, f"Physical goal yaw error: {yaw_error}"
        print(
            f"COMPLETE MPPI task={self.path_id} driving_mode={mode} terminal_mode={self.state.actual_mode} "
            f"physical_position_error={position_error:.4f} yaw_error={yaw_error:.4f}",
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gazebo-version", choices=("gz", "ign"), required=True)
    parser.add_argument("--metrics", default=None)
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args.gazebo_version)
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
        node.task(0.7, 0, 0, 0)
        node.task(0, 0.5, 0, 2)
        node.task(0, 0, 0.6, 1)
        # Begin another translation, then revoke context while actually moving.
        assert node.service(node.stop_client).success
        node.wait(lambda: node.state.fault and node.stopped())
        p = node.state.pose
        node.path_id += 1
        node.path = [
            Pose2D(x=p.x, y=p.y, theta=p.theta),
            Pose2D(x=p.x + 1.0, y=p.y, theta=p.theta),
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
        # Loss of the independent context stream cannot renew obstacle/task age.
        node.streaming = False
        node.wait(lambda: node.state.fault and node.planner.phase == 3)
        node.wait(node.stopped)
        old_session = node.state.session_id
        retained_mode = node.state.actual_mode
        p = node.state.pose
        node.path_id += 1
        node.path = [Pose2D(x=p.x, y=p.y, theta=p.theta)] * 2
        node.arm()
        node.hold(0.4)
        assert node.state.session_id > old_session
        assert node.state.actual_mode == retained_mode
        assert node.max_compute < 0.075
        node.report_model_metrics(args.metrics)
        print(
            f"PASS: MPPI straight/Crab/Spin goals, physical settling, context loss and explicit recovery; max_compute={node.max_compute:.4f}s"
        )
    finally:
        node.streaming = False
        node.report_partial_metrics(args.metrics)
        node.truth.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
