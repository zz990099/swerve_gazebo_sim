"""A deliberately slow diagnostic subscriber must not stall control snapshots."""

import time

import rclpy
from rclpy.node import Node
from smoke_chassis import snapshot_qos
from swerve_gazebo_sim.msg import ChassisState, PlannerState


def main():
    rclpy.init()
    node = Node("slow_diagnostic_observer")

    def observe(_):
        time.sleep(0.2)

    node.create_subscription(
        ChassisState, "chassis_controller/state", observe, snapshot_qos()
    )
    node.create_subscription(
        PlannerState, "mppi_planner/state", observe, snapshot_qos()
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
