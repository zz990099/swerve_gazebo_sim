"""Validate public launch entry points in a sourced ROS workspace."""

import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument

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
