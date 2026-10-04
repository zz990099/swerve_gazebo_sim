# SPDX-License-Identifier: Apache-2.0
"""Run the MPPI planning owner with the explicit simulation execution interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from swerve_gazebo_sim.bringup import gazebo_variant, load_config, names


def setup(context):
    package = get_package_share_directory("swerve_gazebo_sim")
    family = LaunchConfiguration("gazebo_version").perform(context)
    gazebo_variant(family)
    configuration = load_config(LaunchConfiguration("config").perform(context))
    g, c = configuration["geometry"], configuration["control"]
    namespace, prefix = names(
        LaunchConfiguration("namespace").perform(context),
        LaunchConfiguration("robot_name").perform(context),
        LaunchConfiguration("prefix").perform(context),
    )
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package, "launch", f"demo_{family}.launch.py")
        ),
        launch_arguments={
            key: LaunchConfiguration(key).perform(context)
            for key in (
                "config",
                "namespace",
                "robot_name",
                "prefix",
                "headless",
                "publish_ground_truth",
            )
        }
        | {"chassis_control": "true", "external_joint_control": "false"},
    )
    planner = Node(
        package="swerve_gazebo_sim",
        executable="mppi_planner",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "body_frame": prefix + "base_footprint",
                "odom_frame": c["odom_frame"] or prefix + "odom",
                "wheelbase_m": g["wheelbase"],
                "track_m": g["track_width"],
                "wheel_radius_m": g["wheel_radius"],
                "max_wheel_speed_mps": c["max_wheel_speed"] * g["wheel_radius"],
                "max_wheel_accel_mps2": c["max_wheel_acceleration"] * g["wheel_radius"],
                "max_steer_rate_radps": c["max_steering_rate"],
            }
        ],
    )
    return [demo, planner]


def generate_launch_description():
    package = get_package_share_directory("swerve_gazebo_sim")
    defaults = {
        "gazebo_version": "gz",
        "config": os.path.join(package, "config", "swerve.yaml"),
        "namespace": "",
        "robot_name": "swerve_robot",
        "prefix": "",
        "headless": "false",
        "publish_ground_truth": "false",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(key, default_value=value)
                for key, value in defaults.items()
            ],
            OpaqueFunction(function=setup),
        ]
    )
