# SPDX-License-Identifier: Apache-2.0
"""Shared startup mechanics for fixed Gazebo-version launch entry points."""

import os
import shlex
import sys
import tempfile
from functools import partial

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

from .bringup import ODOMETRY_DEFAULTS, controller_config, load_config, names

PACKAGE_NAME = "swerve_gazebo_sim"


def odometry_launch_arguments():
    descriptions = {
        "publish_odom": "Publish kinematic Odometry; empty uses YAML",
        "odom_topic": "Odometry topic; relative names use the robot namespace",
        "publish_odom_tf": "Publish odometry TF independently of Odometry messages; empty uses YAML",
        "odom_frame": "Odometry / TF parent frame; empty uses YAML or automatic prefix",
        "odom_child_frame": "Odometry / TF child frame; empty uses YAML or automatic prefix",
    }
    return [
        DeclareLaunchArgument(name, default_value="", description=description)
        for name, description in descriptions.items()
    ]


def odometry_parameters(context, configuration):
    parameters = {
        key: configuration[key] for key in (*ODOMETRY_DEFAULTS, "publish_odom_tf")
    }
    for key in parameters:
        value = LaunchConfiguration(key).perform(context)
        if not value:
            continue
        if key in ("publish_odom", "publish_odom_tf"):
            if value.lower() not in ("true", "false"):
                raise ValueError(f"{key} must be true or false")
            value = value.lower() == "true"
        parameters[key] = value
    return parameters


def spawn_setup(context, variant):
    package_share = get_package_share_directory(PACKAGE_NAME)

    config_file = LaunchConfiguration("config").perform(context)
    namespace, prefix = names(
        LaunchConfiguration("namespace").perform(context),
        LaunchConfiguration("robot_name").perform(context),
        LaunchConfiguration("prefix").perform(context),
    )
    configuration = load_config(config_file)

    controller_template = os.path.join(
        package_share,
        "config",
        "controllers.yaml",
    )
    controller_parameters = controller_config(
        controller_template,
        configuration,
        namespace,
        prefix,
    )

    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix="swerve_controllers_",
        suffix=".yaml",
        delete=False,
    ) as controller_stream:
        yaml.safe_dump(controller_parameters, controller_stream)
        controllers_file = controller_stream.name

    model_file = os.path.join(
        package_share,
        "urdf",
        "swerve_drive.urdf.xacro",
    )
    xacro_mappings = {
        "gazebo_version": variant["name"],
        "config_file": config_file,
        "prefix": prefix,
        "namespace": namespace,
        "controllers_file": controllers_file,
        "publish_ground_truth": LaunchConfiguration("publish_ground_truth").perform(
            context
        ),
    }
    try:
        robot_description = xacro.process_file(
            model_file,
            mappings=xacro_mappings,
        ).toxml()
    except Exception:
        os.unlink(controllers_file)
        raise

    def remove_temporary_controller_file(event, launch_context):
        del event, launch_context
        if os.path.exists(controllers_file):
            os.unlink(controllers_file)
        return []

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_model = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        output="screen",
        arguments=[
            "-world",
            LaunchConfiguration("world_name").perform(context),
            "-name",
            LaunchConfiguration("robot_name").perform(context),
            "-topic",
            f"{namespace}/robot_description",
            "-allow_renaming",
            "false",
            "-x",
            LaunchConfiguration("x").perform(context),
            "-y",
            LaunchConfiguration("y").perform(context),
            "-z",
            LaunchConfiguration("z").perform(context),
            "-Y",
            LaunchConfiguration("yaw").perform(context),
        ],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "steering_controller",
            "wheel_controller",
            "--controller-manager",
            f"{namespace}/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--param-file",
            controllers_file,
        ],
    )

    controller_node_parameters = dict(configuration["control"])
    for parameter_name in ("wheelbase", "track_width", "wheel_radius"):
        controller_node_parameters[parameter_name] = configuration["geometry"][
            parameter_name
        ]
    controller_node_parameters.update(
        {
            "use_sim_time": True,
            "joint_prefix": prefix,
            "frame_prefix": prefix,
        }
    )
    controller_node_parameters.update(
        odometry_parameters(context, configuration["control"])
    )
    swerve_controller = Node(
        executable=sys.executable,
        arguments=["-m", "swerve_gazebo_sim.controller"],
        exec_name="swerve_controller",
        namespace=namespace,
        output="screen",
        parameters=[controller_node_parameters],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge",
        namespace=namespace,
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_bridge")),
        parameters=[
            {
                "config_file": LaunchConfiguration("bridge_config").perform(context),
                "expand_gz_topic_names": True,
                "use_sim_time": True,
            }
        ],
    )

    def start_controllers_after_spawn(event, launch_context):
        del launch_context
        if event.returncode == 0:
            return [controller_spawner]
        return [EmitEvent(event=Shutdown(reason="Model spawn failed"))]

    def start_swerve_controller(event, launch_context):
        del launch_context
        if event.returncode == 0:
            return [swerve_controller]
        return [EmitEvent(event=Shutdown(reason="Controller activation failed"))]

    cleanup_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=remove_temporary_controller_file)
    )
    spawn_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_model,
            on_exit=start_controllers_after_spawn,
        )
    )
    controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=start_swerve_controller,
        )
    )

    return [
        cleanup_handler,
        spawn_handler,
        controller_handler,
        robot_state_publisher,
        bridge,
        spawn_model,
    ]


def generate_spawn_launch_description(variant):
    package_share = get_package_share_directory(PACKAGE_NAME)

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
        description="Name of the running Gazebo world",
    )
    declare_publish_ground_truth = DeclareLaunchArgument(
        "publish_ground_truth",
        default_value="false",
        description="Publish Gazebo-only ground truth for validation; never bridged to ROS",
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
    launch_description.add_action(declare_config)
    launch_description.add_action(
        DeclareLaunchArgument(
            "bridge_config",
            default_value=os.path.join(package_share, "config", variant["bridge"]),
            description="Complete YAML for the single generic bridge; may include sensor topics",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start the generic bridge; disable when reusing an existing bridge",
        )
    )
    for argument in odometry_launch_arguments():
        launch_description.add_action(argument)
    launch_description.add_action(declare_namespace)
    launch_description.add_action(declare_prefix)
    launch_description.add_action(declare_robot_name)
    launch_description.add_action(declare_world_name)
    launch_description.add_action(declare_publish_ground_truth)
    launch_description.add_action(declare_x)
    launch_description.add_action(declare_y)
    launch_description.add_action(declare_z)
    launch_description.add_action(declare_yaw)
    launch_description.add_action(
        OpaqueFunction(function=partial(spawn_setup, variant=variant))
    )
    return launch_description


def demo_setup(context, variant):
    package_share = get_package_share_directory(PACKAGE_NAME)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    version = variant

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

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", variant["spawn_launch"])
        ),
        launch_arguments={
            "config": LaunchConfiguration("config").perform(context),
            "bridge_config": LaunchConfiguration("bridge_config").perform(context),
            "start_bridge": LaunchConfiguration("start_bridge").perform(context),
            **{
                key: LaunchConfiguration(key).perform(context)
                for key in (*ODOMETRY_DEFAULTS, "publish_odom_tf")
            },
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
        spawn_robot,
    ]


def generate_demo_launch_description(variant):
    package_share = get_package_share_directory(PACKAGE_NAME)

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
        description="Publish Gazebo-only ground truth for validation; never bridged to ROS",
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
    launch_description.add_action(declare_headless)
    launch_description.add_action(declare_world)
    launch_description.add_action(declare_config)
    launch_description.add_action(
        DeclareLaunchArgument(
            "bridge_config",
            default_value=os.path.join(package_share, "config", variant["bridge"]),
            description="Complete YAML for the single generic bridge; may include sensor topics",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start the generic bridge; disable when reusing an existing bridge",
        )
    )
    for argument in odometry_launch_arguments():
        launch_description.add_action(argument)
    launch_description.add_action(declare_namespace)
    launch_description.add_action(declare_prefix)
    launch_description.add_action(declare_robot_name)
    launch_description.add_action(declare_world_name)
    launch_description.add_action(declare_publish_ground_truth)
    launch_description.add_action(declare_x)
    launch_description.add_action(declare_y)
    launch_description.add_action(declare_z)
    launch_description.add_action(declare_yaw)
    launch_description.add_action(
        OpaqueFunction(function=partial(demo_setup, variant=variant))
    )
    return launch_description
