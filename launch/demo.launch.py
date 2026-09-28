# SPDX-License-Identifier: Apache-2.0
"""Start a Gazebo world and spawn one swerve chassis."""

import os
import shlex

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
from swerve_gazebo_sim.bringup import gazebo_variant

PACKAGE_NAME = "swerve_gazebo_sim"


def launch_setup(context):
    package_share = get_package_share_directory(PACKAGE_NAME)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    gazebo_version = LaunchConfiguration("gazebo_version").perform(context)
    version = gazebo_variant(gazebo_version)

    world_path = LaunchConfiguration("world").perform(context)
    if not world_path:
        world_path = os.path.join(package_share, "worlds", version["world"])

    gazebo_arguments = ["-r", "-v", "2"]
    if LaunchConfiguration("headless").perform(context).lower() == "true":
        gazebo_arguments.append("-s")
    gazebo_arguments.append(world_path)

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": shlex.join(gazebo_arguments),
            "gz_version": version["sim_version"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    clock_bridge_config = os.path.join(
        package_share,
        "config",
        version["clock_bridge"],
    )
    clock_bridge_parameters = {
        "config_file": clock_bridge_config,
    }
    start_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[clock_bridge_parameters],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "spawn.launch.py")
        ),
        launch_arguments={
            "gazebo_version": gazebo_version,
            "config": LaunchConfiguration("config").perform(context),
            "namespace": LaunchConfiguration("namespace").perform(context),
            "prefix": LaunchConfiguration("prefix").perform(context),
            "robot_name": LaunchConfiguration("robot_name").perform(context),
            "world_name": LaunchConfiguration("world_name").perform(context),
            "publish_ground_truth": LaunchConfiguration("publish_ground_truth").perform(
                context
            ),
            "x": LaunchConfiguration("x").perform(context),
            "y": LaunchConfiguration("y").perform(context),
            "z": LaunchConfiguration("z").perform(context),
            "yaw": LaunchConfiguration("yaw").perform(context),
        }.items(),
    )

    return [
        start_gazebo,
        start_clock_bridge,
        spawn_robot,
    ]


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)

    declare_gazebo_version = DeclareLaunchArgument(
        "gazebo_version",
        default_value="ign",
        description="Gazebo family: 'ign' for Fortress or 'gz' for Harmonic",
    )
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run the Gazebo server without the graphical client",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="",
        description="World SDF path; empty selects the matching bundled world",
    )
    declare_config = DeclareLaunchArgument(
        "config",
        default_value=os.path.join(package_share, "config", "swerve.yaml"),
        description="Swerve geometry and controller configuration",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace for this robot",
    )
    declare_prefix = DeclareLaunchArgument(
        "prefix",
        default_value="auto",
        description="Link, joint and frame prefix; 'auto' derives it from namespace",
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="swerve",
        description="Unique Gazebo entity name",
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="swerve_world",
        description="Gazebo world name used by the spawn service",
    )
    declare_publish_ground_truth = DeclareLaunchArgument(
        "publish_ground_truth",
        default_value="false",
        description="Publish Gazebo ground-truth odometry for validation",
    )
    declare_x = DeclareLaunchArgument(
        "x",
        default_value="0",
        description="Initial world x position in metres",
    )
    declare_y = DeclareLaunchArgument(
        "y",
        default_value="0",
        description="Initial world y position in metres",
    )
    declare_z = DeclareLaunchArgument(
        "z",
        default_value="0.02",
        description="Initial world z position in metres",
    )
    declare_yaw = DeclareLaunchArgument(
        "yaw",
        default_value="0",
        description="Initial world yaw angle in radians",
    )

    launch_description = LaunchDescription()
    launch_description.add_action(declare_gazebo_version)
    launch_description.add_action(declare_headless)
    launch_description.add_action(declare_world)
    launch_description.add_action(declare_config)
    launch_description.add_action(declare_namespace)
    launch_description.add_action(declare_prefix)
    launch_description.add_action(declare_robot_name)
    launch_description.add_action(declare_world_name)
    launch_description.add_action(declare_publish_ground_truth)
    launch_description.add_action(declare_x)
    launch_description.add_action(declare_y)
    launch_description.add_action(declare_z)
    launch_description.add_action(declare_yaw)
    launch_description.add_action(OpaqueFunction(function=launch_setup))
    return launch_description
