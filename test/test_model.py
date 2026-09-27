from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

from swerve_gazebo_sim.bringup import controller_config, load_config, names


ROOT = Path(__file__).resolve().parents[1]


def test_geometry_has_eight_actuated_joints_and_consistent_dimensions(tmp_path):
    cfg = load_config(ROOT / 'config/swerve.yaml')
    wrapper = tmp_path / 'chassis_only.urdf.xacro'
    wrapper.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        f'<xacro:property name="settings" value="${{xacro.load_yaml(\'{ROOT / "config/swerve.yaml"}\')}}"/>'
        f'<xacro:include filename="{ROOT / "urdf/chassis.urdf.xacro"}"/>'
        '<xacro:swerve_chassis prefix="test_" '
        'geometry="${settings[\'geometry\']}" control="${settings[\'control\']}"/>'
        '</robot>')
    root = ET.fromstring(xacro.process_file(str(wrapper)).toxml())
    joints = [j for j in root.findall('joint') if j.get('type') != 'fixed']
    assert len(joints) == 8
    for corner, sx, sy in [('fl', 1, 1), ('fr', 1, -1), ('rl', -1, 1), ('rr', -1, -1)]:
        j = root.find(f"joint[@name='test_{corner}_steering_joint']")
        xyz = [float(v) for v in j.find('origin').get('xyz').split()]
        assert xyz[:2] == pytest.approx([sx * cfg['geometry']['wheelbase'] / 2,
                                        sy * cfg['geometry']['track_width'] / 2])
        wheel = root.find(f"link[@name='test_{corner}_wheel_link']/collision/geometry/cylinder")
        assert float(wheel.get('radius')) == cfg['geometry']['wheel_radius']
    assert not root.findall('.//mesh')
    assert not root.findall('.//gazebo')
    assert not root.findall('.//ros2_control')
    for inertial in root.findall('.//inertial'):
        assert float(inertial.find('mass').get('value')) > 0
        assert all(float(inertial.find('inertia').get(key)) > 0 for key in ('ixx', 'iyy', 'izz'))


def test_public_model_combines_chassis_and_plugins():
    model = ROOT / 'urdf/swerve_drive.urdf.xacro'
    mappings = dict(config_file=str(ROOT / 'config/swerve.yaml'),
                    prefix='robot1_', namespace='/robot1', robot_name='robot1',
                    controllers_file='/tmp/controllers.yaml')
    root = ET.fromstring(xacro.process_file(str(model), mappings=mappings).toxml())
    assert len([j for j in root.findall('joint') if j.get('type') != 'fixed']) == 8
    assert len(root.findall('.//ros2_control')) == 1
    assert len(root.findall(".//gazebo[@reference]")) == 4
    assert len(root.findall('.//gazebo/plugin')) == 1
    assert root.find('.//gazebo/plugin/parameters').text == '/tmp/controllers.yaml'
    assert root.find('.//gazebo/plugin/ros/namespace').text == '/robot1'
    with_truth = ET.fromstring(xacro.process_file(
        str(model), mappings=dict(mappings, publish_ground_truth='true')).toxml())
    assert len(with_truth.findall('.//gazebo/plugin')) == 2
    assert with_truth.find('.//gazebo/plugin/odom_topic').text == '/model/robot1/odometry'


def test_public_model_uses_custom_config_without_dimension_arguments(tmp_path):
    cfg = load_config(ROOT / 'config/swerve.yaml')
    cfg['geometry']['wheelbase'] = 0.8
    cfg['control']['max_wheel_speed'] = 16.0
    config = tmp_path / 'custom.yaml'
    config.write_text(yaml.safe_dump(cfg))
    model = ROOT / 'urdf/swerve_drive.urdf.xacro'
    root = ET.fromstring(xacro.process_file(
        str(model), mappings={'config_file': str(config)}).toxml())
    steering = root.find("joint[@name='fl_steering_joint']/origin")
    assert float(steering.get('xyz').split()[0]) == pytest.approx(0.4)
    maximum = root.find("ros2_control/joint[@name='fl_wheel_joint']/command_interface/param[@name='max']")
    assert float(maximum.text) == 16.0


def test_namespace_and_controller_joint_alignment():
    ns, prefix = names('/fleet/robot1/', 'robot1', 'auto')
    assert (ns, prefix) == ('/fleet/robot1', 'fleet_robot1_')
    cfg = controller_config(ROOT / 'config/controllers.yaml', load_config(ROOT / 'config/swerve.yaml'), ns, prefix)
    assert cfg['/fleet/robot1/wheel_controller']['ros__parameters']['joints'][0] == 'fleet_robot1_fl_wheel_joint'


def test_invalid_namespace():
    with pytest.raises(ValueError):
        names('bad-name', 'swerve', 'auto')
