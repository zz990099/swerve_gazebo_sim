# SPDX-License-Identifier: Apache-2.0
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    share = get_package_share_directory('swerve_gazebo_sim')
    gz_share = get_package_share_directory('ros_gz_sim')
    gz_args = '-r -v 2 ' + ('-s ' if arg('headless').lower() == 'true' else '') + shlex.quote(arg('world'))
    world = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
                                     launch_arguments={'gz_args': gz_args, 'gz_version': '6', 'on_exit_shutdown': 'true'}.items())
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen')
    spawn = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'spawn.launch.py')),
                                     launch_arguments={k: arg(k) for k in
                                                       ('config', 'namespace', 'prefix', 'robot_name', 'world_name',
                                                        'publish_ground_truth', 'x', 'y', 'z', 'yaw')}.items())
    return [world, clock, spawn]


def generate_launch_description():
    share = get_package_share_directory('swerve_gazebo_sim')
    defaults = dict(headless='false', world=os.path.join(share, 'worlds', 'empty.sdf'),
                    config=os.path.join(share, 'config', 'swerve.yaml'), namespace='', prefix='auto',
                    robot_name='swerve', world_name='swerve_world', publish_ground_truth='false',
                    x='0', y='0', z='0.02', yaw='0')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
