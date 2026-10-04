"""External mode must claim all eight interfaces through one protected endpoint."""

from pathlib import Path

from swerve_gazebo_sim.bringup import controller_config, load_config

ROOT = Path(__file__).resolve().parents[1]


def test_external_endpoint_is_exclusive_and_prefixed():
    cfg = load_config(ROOT / "config/swerve.yaml")
    cfg["control"]["external_joint_control"] = True
    data = controller_config(ROOT / "config/controllers.yaml", cfg, "/bot", "bot_")
    manager = data["/bot/controller_manager"]["ros__parameters"]
    assert "steering_controller" not in manager and "wheel_controller" not in manager
    assert manager["guarded_joint_controller"]["type"] == (
        "swerve_gazebo_sim/GuardedJointController"
    )
    guarded = data["/bot/guarded_joint_controller"]["ros__parameters"]
    assert guarded["steering_joints"] == [
        f"bot_{corner}_steering_joint" for corner in ("fl", "fr", "rl", "rr")
    ]
    assert guarded["wheel_joints"] == [
        f"bot_{corner}_wheel_joint" for corner in ("fl", "fr", "rl", "rr")
    ]
    assert guarded["max_wheel_speed_radps"] == cfg["control"]["max_wheel_speed"]
    assert guarded["use_sim_time"] is True
    assert "/bot/steering_controller" not in data
    assert "/bot/wheel_controller" not in data


def test_default_endpoint_remains_twist_owned():
    cfg = load_config(ROOT / "config/swerve.yaml")
    data = controller_config(ROOT / "config/controllers.yaml", cfg, "", "")
    manager = data["/controller_manager"]["ros__parameters"]
    assert "guarded_joint_controller" not in manager
    assert "steering_controller" in manager and "wheel_controller" in manager


def test_explicit_chassis_claims_all_joints_and_maps_units():
    cfg = load_config(ROOT / "config/swerve.yaml")
    cfg["control"]["chassis_control"] = True
    data = controller_config(ROOT / "config/controllers.yaml", cfg, "/bot", "bot_")
    manager = data["/bot/controller_manager"]["ros__parameters"]
    assert "guarded_joint_controller" not in manager
    assert "steering_controller" not in manager
    assert "wheel_controller" not in manager
    assert (
        manager["chassis_controller"]["type"] == "swerve_gazebo_sim/ChassisController"
    )
    controller = data["/bot/chassis_controller"]["ros__parameters"]
    assert controller["body_frame"] == "bot_base_footprint"
    assert controller["odom_frame"] == "bot_odom"
    assert controller["max_wheel_speed_mps"] == 2.0
    assert controller["max_wheel_accel_mps2"] == 4.0
    assert controller["steering_joints"][0] == "bot_fl_steering_joint"


def test_chassis_and_external_owners_cannot_coexist():
    import pytest

    from swerve_gazebo_sim.bringup import validate_control

    cfg = load_config(ROOT / "config/swerve.yaml")["control"]
    cfg.update(chassis_control=True, external_joint_control=True)
    with pytest.raises(ValueError, match="mutually exclusive"):
        validate_control(cfg)
