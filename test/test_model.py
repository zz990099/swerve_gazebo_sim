import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro

from swerve_gazebo_sim.bringup import controller_config, load_config, names


ROOT = Path(__file__).resolve().parents[1]


def test_geometry_has_eight_actuated_joints_and_consistent_dimensions():
    cfg = load_config(ROOT / 'config/swerve.yaml')
    mappings = {key: str(value) for key, value in cfg['geometry'].items()}
    mappings.update(enable_control='false', prefix='test_')
    root = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/swerve_chassis.urdf.xacro'), mappings=mappings).toxml())
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
    for inertial in root.findall('.//inertial'):
        assert float(inertial.find('mass').get('value')) > 0
        assert all(float(inertial.find('inertia').get(key)) > 0 for key in ('ixx', 'iyy', 'izz'))


def test_namespace_and_controller_joint_alignment():
    ns, prefix = names('/fleet/robot1/', 'robot1', 'auto')
    assert (ns, prefix) == ('/fleet/robot1', 'fleet_robot1_')
    cfg = controller_config(ROOT / 'config/controllers.yaml', load_config(ROOT / 'config/swerve.yaml'), ns, prefix)
    assert cfg['/fleet/robot1/wheel_controller']['ros__parameters']['joints'][0] == 'fleet_robot1_fl_wheel_joint'


def test_invalid_namespace():
    with pytest.raises(ValueError):
        names('bad-name', 'swerve', 'auto')
