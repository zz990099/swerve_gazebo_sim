# SPDX-License-Identifier: Apache-2.0
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from swerve_gazebo_sim.bringup import controller_config, load_config, names


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('swerve_gazebo_sim')
    cfg = load_config(arg('config'))
    namespace, prefix = names(arg('namespace'), arg('robot_name'), arg('prefix'))
    truth = arg('publish_ground_truth').lower() == 'true'
    if arg('use_sim_time').lower() != 'true':
        raise ValueError('Gazebo simulation requires use_sim_time:=true')
    params = controller_config(os.path.join(share, 'config', 'controllers.yaml'), cfg, namespace, prefix)
    with tempfile.NamedTemporaryFile(mode='w', prefix='swerve_controllers_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(params, stream)
        controllers_file = stream.name
    try:
        mappings = {key: str(value) for key, value in cfg['geometry'].items()}
        mappings.update({key: str(cfg['control'][key]) for key in
                         ('steering_limit', 'max_wheel_speed', 'max_steering_rate')})
        mappings.update(prefix=prefix, namespace=namespace, controllers_file=controllers_file,
                        robot_name=arg('robot_name'), publish_ground_truth=str(truth).lower())
        description = xacro.process_file(os.path.join(share, 'urdf', 'swerve_chassis.urdf.xacro'),
                                         mappings=mappings).toxml()
    except Exception:
        os.unlink(controllers_file)
        raise

    def cleanup(event, launch_context):
        if os.path.exists(controllers_file):
            os.unlink(controllers_file)
        return []

    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                           namespace=namespace, name='robot_state_publisher', output='screen',
                           parameters=[{'robot_description': description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create', namespace=namespace, output='screen',
                 arguments=['-world', arg('world_name'), '-name', arg('robot_name'),
                            '-topic', f'{namespace}/robot_description', '-allow_renaming', 'false',
                            '-x', arg('x'), '-y', arg('y'), '-z', arg('z'), '-Y', arg('yaw')])
    spawner = Node(package='controller_manager', executable='spawner', namespace=namespace,
                   output='screen', arguments=['joint_state_broadcaster', 'steering_controller',
                                                'wheel_controller', '--controller-manager',
                                                f'{namespace}/controller_manager',
                                                '--controller-manager-timeout', '60',
                                                '--param-file', controllers_file])
    controller_params = dict(cfg['control'])
    controller_params.update({key: cfg['geometry'][key] for key in ('wheelbase', 'track_width', 'wheel_radius')})
    controller_params.update(use_sim_time=True, joint_prefix=prefix, frame_prefix=prefix)
    controller = Node(package='swerve_gazebo_sim', executable='swerve_controller',
                      namespace=namespace, output='screen', parameters=[controller_params])

    def after_spawn(event, launch_context):
        return [spawner] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Model spawn failed'))]

    def after_controllers(event, launch_context):
        return [controller] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Controller activation failed'))]

    actions = [RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
               RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
               RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_controllers)),
               state_publisher, spawn]
    if truth:
        topic = f'/model/{arg("robot_name")}/odometry'
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            namespace=namespace, name='ground_truth_bridge',
                            arguments=[topic + '@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                            remappings=[(topic, f'{namespace}/ground_truth/odom')],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('swerve_gazebo_sim')
    defaults = dict(config=os.path.join(share, 'config', 'swerve.yaml'), namespace='', prefix='auto',
                    robot_name='swerve', world_name='swerve_world', x='0', y='0', z='0.02', yaw='0',
                    use_sim_time='true', publish_ground_truth='false')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
