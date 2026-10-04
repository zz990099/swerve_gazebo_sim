"""Process cleanup and transport-isolation regression tests."""

import importlib.util
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import psutil
import pytest
from obstacle_fixture import OBSTACLES, clearance, corridor_world

RUNNER = Path(__file__).with_name("run_simulation.py")
spec = importlib.util.spec_from_file_location("simulation_runner", RUNNER)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize("family", ["ign", "gz"])
def test_corridor_uses_real_collision_geometry_and_keeps_physics(family, tmp_path):
    world = (
        ET.parse(corridor_world(family, tmp_path / "world.sdf")).getroot().find("world")
    )
    assert world.find("physics/max_step_size").text == "0.005"
    assert len(world.findall("plugin")) == 3
    models = [
        m for m in world.findall("model") if m.get("name").startswith("corridor_")
    ]
    assert len(models) == 4
    for model, (x, y, radius) in zip(models, OBSTACLES):
        pose = list(map(float, model.find("pose").text.split()))
        assert pose[:3] == [x, y, 0.4]
        assert model.find("static").text == "true"
        for kind in ("collision", "visual"):
            cylinder = model.find(f"link/{kind}/geometry/cylinder")
            assert float(cylinder.find("radius").text) == radius
            assert float(cylinder.find("length").text) == 0.8


def test_clearance_detects_crossing_even_with_clear_endpoints():
    x, y, _ = OBSTACLES[0]
    left, right = (x - 2, y), (x + 2, y)
    assert clearance(left, left) > 0
    assert clearance(right, right) > 0
    assert clearance(left, right) < 0


def test_each_run_has_an_independent_transport_partition():
    original = {"GZ_PARTITION": "parent", "ROS_DOMAIN_ID": "71"}
    first = runner.simulation_environment(original)
    second = runner.simulation_environment(original)
    assert first["GZ_PARTITION"] != second["GZ_PARTITION"]
    assert first["GZ_PARTITION"] == first["IGN_PARTITION"]
    assert first["ROS_DOMAIN_ID"] == "71"
    assert original == {"GZ_PARTITION": "parent", "ROS_DOMAIN_ID": "71"}


def test_simulator_descendant_is_stopped_even_after_launch_parent_exits(tmp_path):
    pid_file = tmp_path / "child_pid"
    ready_file = tmp_path / "child_ready"
    child_code = (
        "import signal,sys,time; from pathlib import Path; "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "Path(sys.argv[1]).write_text('ready'); time.sleep(60)"
    )
    parent_code = (
        "import subprocess,sys,time; from pathlib import Path; "
        "child=subprocess.Popen([sys.executable,'-c',sys.argv[2],sys.argv[3]],start_new_session=True); "
        "Path(sys.argv[1]).write_text(str(child.pid)); time.sleep(60)"
    )
    parent = subprocess.Popen(
        [sys.executable, "-c", parent_code, str(pid_file), child_code, str(ready_file)],
        start_new_session=True,
    )
    tracked = {}
    try:
        deadline = time.monotonic() + 5
        while not ready_file.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ready_file.exists() and pid_file.exists()
        child_pid = int(pid_file.read_text())
        runner.track_descendants([parent], tracked)
        assert any(child.pid == child_pid for child in tracked.values())
        parent.terminate()
        parent.wait(timeout=5)
        runner.stop_descendants(tracked)
        try:
            child = psutil.Process(child_pid)
            assert child.status() == psutil.STATUS_ZOMBIE
        except psutil.NoSuchProcess:
            pass
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
        runner.stop_descendants(tracked)
