import xml.etree.ElementTree as ET
from pathlib import Path
from textwrap import dedent

import pytest
import xacro
import yaml

from swerve_gazebo_sim.bringup import (
    ODOMETRY_DEFAULTS,
    TRANSITION_DEFAULTS,
    controller_config,
    gazebo_variant,
    load_config,
    names,
)

ROOT = Path(__file__).resolve().parents[1]


def test_external_control_configuration_defaults_and_type(tmp_path):
    config = yaml.safe_load((ROOT / "config/swerve.yaml").read_text())
    del config["control"]["external_joint_control"]
    path = tmp_path / "legacy.yaml"
    path.write_text(yaml.safe_dump(config))
    assert load_config(path)["control"]["external_joint_control"] is False
    config["control"]["external_joint_control"] = True
    path.write_text(yaml.safe_dump(config))
    assert load_config(path)["control"]["external_joint_control"] is True
    config["control"]["external_joint_control"] = "true"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="external_joint_control"):
        load_config(path)


def test_geometry_has_eight_actuated_joints_and_consistent_dimensions(tmp_path):
    cfg = load_config(ROOT / "config/swerve.yaml")
    wrapper = tmp_path / "chassis_only.urdf.xacro"
    wrapper.write_text(
        dedent(f"""\
            <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
              <xacro:property
                name="settings"
                value="${{xacro.load_yaml('{ROOT / 'config/swerve.yaml'}')}}"
              />
              <xacro:include
                filename="{ROOT / 'urdf/chassis.urdf.xacro'}"
              />
              <xacro:swerve_chassis
                prefix="test_"
                geometry="${{settings['geometry']}}"
                control="${{settings['control']}}"
              />
            </robot>
            """),
        encoding="utf-8",
    )
    root = ET.fromstring(xacro.process_file(str(wrapper)).toxml())
    joints = [j for j in root.findall("joint") if j.get("type") != "fixed"]
    assert len(joints) == 8
    for corner, sx, sy in [("fl", 1, 1), ("fr", 1, -1), ("rl", -1, 1), ("rr", -1, -1)]:
        j = root.find(f"joint[@name='test_{corner}_steering_joint']")
        xyz = [float(v) for v in j.find("origin").get("xyz").split()]
        assert xyz[:2] == pytest.approx(
            [
                sx * cfg["geometry"]["wheelbase"] / 2,
                sy * cfg["geometry"]["track_width"] / 2,
            ]
        )
        wheel = root.find(
            f"link[@name='test_{corner}_wheel_link']/collision/geometry/cylinder"
        )
        assert float(wheel.get("radius")) == cfg["geometry"]["wheel_radius"]
    assert not root.findall(".//mesh")
    assert not root.findall(".//gazebo")
    assert not root.findall(".//ros2_control")
    for inertial in root.findall(".//inertial"):
        assert float(inertial.find("mass").get("value")) > 0
        assert all(
            float(inertial.find("inertia").get(key)) > 0
            for key in ("ixx", "iyy", "izz")
        )


@pytest.mark.parametrize(
    ("gazebo_version", "plugin_filename", "plugin_name"),
    [
        (
            "ign",
            "ignition-gazebo-odometry-publisher-system",
            "ignition::gazebo::systems::OdometryPublisher",
        ),
        (
            "gz",
            "gz-sim-odometry-publisher-system",
            "gz::sim::systems::OdometryPublisher",
        ),
    ],
)
def test_public_model_combines_chassis_and_plugins(
    gazebo_version,
    plugin_filename,
    plugin_name,
):
    model = ROOT / "urdf/swerve_drive.urdf.xacro"
    mappings = {
        "gazebo_version": gazebo_version,
        "config_file": str(ROOT / "config/swerve.yaml"),
        "prefix": "robot1_",
        "namespace": "/robot1",
        "controllers_file": "/tmp/controllers.yaml",
    }
    root = ET.fromstring(xacro.process_file(str(model), mappings=mappings).toxml())
    assert len([j for j in root.findall("joint") if j.get("type") != "fixed"]) == 8
    assert len(root.findall(".//ros2_control")) == 1
    wheel_contacts = root.findall(".//gazebo[@reference]")
    assert len(wheel_contacts) == 4
    for contact in wheel_contacts:
        assert contact.find("mu1").text == "1.0"
        assert contact.find("mu2").text == "1.0"
        assert contact.find("fdir1") is None
    assert len(root.findall(".//gazebo/plugin")) == 1
    assert root.find(".//gazebo/plugin/parameters").text == "/tmp/controllers.yaml"
    assert root.find(".//gazebo/plugin/ros/namespace").text == "/robot1"
    with_truth = ET.fromstring(
        xacro.process_file(
            str(model),
            mappings=dict(mappings, publish_ground_truth="true"),
        ).toxml()
    )
    assert len(with_truth.findall(".//gazebo/plugin")) == 2
    odometry_plugin = with_truth.find(
        f".//gazebo/plugin[@filename='{plugin_filename}']"
    )
    assert odometry_plugin is not None
    assert odometry_plugin.get("name") == plugin_name
    assert odometry_plugin.find("odom_topic").text == "/robot1/ground_truth/odom"


def test_gazebo_variants_reference_version_specific_files():
    ignition = gazebo_variant("ign")
    harmonic = gazebo_variant("gz")

    assert ignition["sim_version"] == "6"
    assert harmonic["sim_version"] == "8"
    assert (ROOT / "worlds" / ignition["world"]).is_file()
    assert (ROOT / "worlds" / harmonic["world"]).is_file()
    for variant, message_prefix in ((ignition, "ignition"), (harmonic, "gz")):
        bridge_file = ROOT / "config" / variant["bridge"]
        combined = yaml.safe_load(bridge_file.read_text(encoding="utf-8"))
        assert len(combined) == 1
        assert combined[0]["topic_name"] == "/clock"
        assert combined[0]["gz_type_name"] == f"{message_prefix}.msgs.Clock"
        assert all(
            entry["ros_type_name"] != "nav_msgs/msg/Odometry" for entry in combined
        )

    ignition_world = ET.parse(ROOT / "worlds" / ignition["world"]).getroot()
    harmonic_world = ET.parse(ROOT / "worlds" / harmonic["world"]).getroot()
    for world in (ignition_world, harmonic_world):
        physics = world.find(".//physics")
        assert float(physics.find("max_step_size").text) == pytest.approx(0.005)
        assert float(physics.find("real_time_factor").text) == pytest.approx(1.0)
    ignition_plugins = [
        plugin.get("filename") for plugin in ignition_world.findall(".//plugin")
    ]
    harmonic_plugins = [
        plugin.get("filename") for plugin in harmonic_world.findall(".//plugin")
    ]
    assert "ignition-gazebo-physics-system" in ignition_plugins
    assert "gz-sim-physics-system" in harmonic_plugins

    with pytest.raises(ValueError):
        gazebo_variant("automatic")


def test_public_model_uses_custom_config_without_dimension_arguments(tmp_path):
    cfg = load_config(ROOT / "config/swerve.yaml")
    cfg["geometry"]["wheelbase"] = 0.8
    cfg["control"]["max_wheel_speed"] = 16.0
    config = tmp_path / "custom.yaml"
    config.write_text(yaml.safe_dump(cfg))
    model = ROOT / "urdf/swerve_drive.urdf.xacro"
    root = ET.fromstring(
        xacro.process_file(str(model), mappings={"config_file": str(config)}).toxml()
    )
    steering = root.find("joint[@name='fl_steering_joint']/origin")
    assert float(steering.get("xyz").split()[0]) == pytest.approx(0.4)
    maximum = root.find(
        "ros2_control/joint[@name='fl_wheel_joint']/command_interface/param[@name='max']"
    )
    assert float(maximum.text) == 16.0


def test_namespace_and_controller_joint_alignment():
    ns, prefix = names("/fleet/robot1/", "robot1", "auto")
    assert (ns, prefix) == ("/fleet/robot1", "fleet_robot1_")
    cfg = controller_config(
        ROOT / "config/controllers.yaml",
        load_config(ROOT / "config/swerve.yaml"),
        ns,
        prefix,
    )
    assert (
        cfg["/fleet/robot1/wheel_controller"]["ros__parameters"]["joints"][0]
        == "fleet_robot1_fl_wheel_joint"
    )


def test_invalid_namespace():
    with pytest.raises(ValueError):
        names("bad-name", "swerve", "auto")


def test_legacy_configuration_gets_transition_defaults(tmp_path):
    cfg = load_config(ROOT / "config/swerve.yaml")
    for key in {**TRANSITION_DEFAULTS, **ODOMETRY_DEFAULTS}:
        del cfg["control"][key]
    path = tmp_path / "legacy.yaml"
    path.write_text(yaml.safe_dump(cfg))
    loaded = load_config(path)
    for key, value in {**TRANSITION_DEFAULTS, **ODOMETRY_DEFAULTS}.items():
        assert loaded["control"][key] == value
