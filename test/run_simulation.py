"""Run an isolated chassis acceptance world and clean up descendants."""

import argparse
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import psutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gazebo-version", choices=("ign", "gz"), required=True)
    parser.add_argument("--logs", required=True)
    args = parser.parse_args()
    directory = Path(args.logs)
    directory.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    partition = "chassis_" + uuid.uuid4().hex
    environment.update(GZ_PARTITION=partition, IGN_PARTITION=partition)
    children = {}
    with (directory / "simulation.log").open("w") as output:
        simulation = subprocess.Popen(
            [
                "ros2",
                "launch",
                "swerve_gazebo_sim",
                f"demo_{args.gazebo_version}.launch.py",
                "headless:=true",
                "publish_ground_truth:=true",
            ],
            stdout=output,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        probe = None
        try:
            probe = subprocess.Popen(
                [
                    sys.executable,
                    str(Path(__file__).with_name("smoke_chassis.py")),
                    "--gazebo-version",
                    args.gazebo_version,
                ],
                env=environment,
                start_new_session=True,
            )
            deadline = time.monotonic() + 180
            while probe.poll() is None:
                try:
                    for child in psutil.Process(simulation.pid).children(
                        recursive=True
                    ):
                        children[(child.pid, child.create_time())] = child
                except psutil.NoSuchProcess:
                    pass
                if simulation.poll() is not None:
                    raise RuntimeError("Simulation exited before acceptance")
                if time.monotonic() > deadline:
                    raise TimeoutError("Chassis acceptance timed out")
                time.sleep(0.2)
            if probe.returncode:
                raise RuntimeError(f"Chassis acceptance failed: {probe.returncode}")
            # Killing the Python command owner must tear down the isolated demo.
            controllers = [
                p
                for p in psutil.Process(simulation.pid).children(recursive=True)
                if "swerve_gazebo_sim.controller" in p.cmdline()
            ]
            if len(controllers) != 1:
                raise RuntimeError(f"Expected one chassis process, got {controllers}")
            controllers[0].kill()
            simulation.wait(timeout=20)
            print("PASS: chassis process death shuts down demo", flush=True)
        finally:
            for process in (probe, simulation):
                if process is not None:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
            remaining = []
            for child in children.values():
                try:
                    child.terminate()
                    remaining.append(child)
                except psutil.NoSuchProcess:
                    pass
            _, alive = psutil.wait_procs(remaining, timeout=3)
            for child in alive:
                try:
                    child.kill()
                except psutil.NoSuchProcess:
                    pass
            print((directory / "simulation.log").read_text()[-12000:], flush=True)


if __name__ == "__main__":
    main()
