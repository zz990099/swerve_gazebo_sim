"""Validate public launch entry points in a sourced ROS workspace."""

import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch import LaunchContext
from swerve_gazebo_sim import launch_support

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("family", ["ign", "gz"])
@pytest.mark.parametrize("kind", ["spawn", "demo"])
def test_version_specific_launch_has_no_runtime_version_argument(
    monkeypatch, family, kind
):
    monkeypatch.setattr(
        launch_support, "get_package_share_directory", lambda package: str(ROOT)
    )
    path = ROOT / "launch" / f"{kind}_{family}.launch.py"
    spec = importlib.util.spec_from_file_location(f"{kind}_{family}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    arguments = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert "gazebo_version" not in arguments
    assert {"config", "namespace", "robot_name", "world_name"} <= arguments
    assert {
        "bridge_config",
        "start_bridge",
        "external_joint_control",
        "publish_odom",
        "odom_topic",
        "publish_odom_tf",
        "odom_frame",
        "odom_child_frame",
    } <= arguments
    assert ("world" in arguments) == (kind == "demo")
    assert ("headless" in arguments) == (kind == "demo")


@pytest.mark.parametrize("family", ["ign", "gz"])
def test_entry_points_bind_their_fixed_family(monkeypatch, family):
    for kind in ("spawn", "demo"):
        selected = []
        monkeypatch.setattr(
            launch_support,
            f"generate_{kind}_launch_description",
            lambda variant, selected=selected: selected.append(variant),
        )
        path = ROOT / "launch" / f"{kind}_{family}.launch.py"
        spec = importlib.util.spec_from_file_location(f"binding_{kind}_{family}", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.generate_launch_description()
        assert selected[0]["name"] == family
        assert selected[0]["spawn_launch"] == f"spawn_{family}.launch.py"


def test_generic_entry_points_are_removed():
    assert not (ROOT / "launch/spawn.launch.py").exists()
    assert not (ROOT / "launch/demo.launch.py").exists()


def test_odometry_launch_overrides_preserve_independent_switches():
    configuration = launch_support.load_config(ROOT / "config/swerve.yaml")["control"]
    context = LaunchContext()
    context.launch_configurations.update(
        publish_odom="false",
        publish_odom_tf="true",
        odom_topic="wheel/odom",
        odom_frame="local_odom",
        odom_child_frame="robot_base",
    )
    result = launch_support.odometry_parameters(context, configuration)
    assert result == {
        "publish_odom": False,
        "publish_odom_tf": True,
        "odom_topic": "wheel/odom",
        "odom_frame": "local_odom",
        "odom_child_frame": "robot_base",
    }


def test_empty_odometry_launch_values_use_yaml():
    configuration = launch_support.load_config(ROOT / "config/swerve.yaml")["control"]
    configuration.update(publish_odom_tf=False, odom_topic="custom/odom")
    context = LaunchContext()
    context.launch_configurations.update(
        {key: "" for key in (*launch_support.ODOMETRY_DEFAULTS, "publish_odom_tf")}
    )
    result = launch_support.odometry_parameters(context, configuration)
    assert result["publish_odom_tf"] is False
    assert result["odom_topic"] == "custom/odom"


@pytest.mark.parametrize(
    "override,expected", [("", True), ("true", True), ("false", False)]
)
def test_external_control_override(override, expected):
    context = LaunchContext()
    context.launch_configurations["external_joint_control"] = override
    assert launch_support.external_control_parameters(
        context, {"external_joint_control": True}
    ) == {"external_joint_control": expected}


def test_external_control_override_requires_boolean():
    context = LaunchContext()
    context.launch_configurations["external_joint_control"] = "yes"
    with pytest.raises(ValueError, match="external_joint_control"):
        launch_support.external_control_parameters(
            context, {"external_joint_control": False}
        )


@pytest.mark.parametrize("family", ["ign", "gz"])
@pytest.mark.parametrize("world", ["", "/tmp/physical-obstacles.sdf"])
def test_mppi_launch_shares_model_and_exclusive_execution(monkeypatch, family, world):
    path = ROOT / "launch/mppi.launch.py"
    spec = importlib.util.spec_from_file_location("mppi_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda package: str(ROOT)
    )
    captured = []
    monkeypatch.setattr(module, "Node", lambda **kwargs: captured.append(kwargs))
    context = LaunchContext()
    context.launch_configurations.update(
        gazebo_version=family,
        config=str(ROOT / "config/swerve.yaml"),
        namespace="robot1",
        robot_name="robot1",
        prefix="auto",
        headless="true",
        publish_ground_truth="true",
        world=world,
    )
    demo = module.setup(context)[0]
    assert isinstance(demo, IncludeLaunchDescription)
    arguments = dict(demo.launch_arguments)
    assert arguments["chassis_control"] == "true"
    assert arguments["external_joint_control"] == "false"
    assert arguments["world"] == world
    assert captured[0]["namespace"] == "/robot1"
    assert captured[0]["executable"] == "mppi_planner"
    model = captured[0]["parameters"][0]
    assert model["body_frame"] == "robot1_base_footprint"
    assert model["odom_frame"] == "robot1_odom"
    assert model["max_wheel_speed_mps"] == 2.0
    assert model["max_wheel_accel_mps2"] == 4.0
    assert model["use_sim_time"] is True
    assert model["confirmation_timeout_s"] == 5.0
    assert model["robot_radius_m"] == 0.5
    assert model["collision_margin_m"] == 0.05


def test_mppi_launch_uses_custom_collision_bound(monkeypatch, tmp_path):
    import yaml

    path = ROOT / "launch/mppi.launch.py"
    spec = importlib.util.spec_from_file_location("large_mppi_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda package: str(ROOT)
    )
    cfg = module.load_config(ROOT / "config/swerve.yaml")
    cfg["geometry"]["body_length"] = 1.4
    cfg["safety"]["collision_margin"] = 0.08
    config = tmp_path / "large.yaml"
    config.write_text(yaml.safe_dump(cfg))
    captured = []
    monkeypatch.setattr(module, "Node", lambda **kwargs: captured.append(kwargs))
    context = LaunchContext()
    context.launch_configurations.update(
        gazebo_version="gz",
        config=str(config),
        namespace="",
        robot_name="robot",
        prefix="",
        headless="true",
        publish_ground_truth="true",
        world="",
    )
    module.setup(context)
    planner = captured[0]["parameters"][0]
    from swerve_gazebo_sim.bringup import controller_config

    execution = controller_config(
        ROOT / "config/controllers.yaml",
        dict(cfg, control=dict(cfg["control"], chassis_control=True)),
        "",
        "",
    )["/chassis_controller"]["ros__parameters"]
    for key in ("robot_radius_m", "collision_margin_m"):
        assert planner[key] == execution[key]
    assert planner["robot_radius_m"] == 0.72
