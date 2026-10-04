"""Start an isolated test world, run the motion probe and clean up all processes."""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

import psutil


def simulation_environment(environment):
    """Give each run its own transport services, even in a reused ROS workspace."""
    result = dict(environment)
    partition = (
        result.get("GZ_PARTITION") or result.get("IGN_PARTITION") or "swerve_test"
    )
    partition += "_" + uuid.uuid4().hex
    result["GZ_PARTITION"] = partition
    result["IGN_PARTITION"] = partition
    return result


def track_descendants(processes, tracked):
    for process in processes:
        try:
            for child in psutil.Process(process.pid).children(recursive=True):
                tracked[(child.pid, child.create_time())] = child
        except psutil.NoSuchProcess:
            pass


def stop_descendants(tracked):
    """Launch exit does not prove its simulator grandchildren have exited."""
    survivors = []
    for child in tracked.values():
        try:
            if child.is_running() and child.status() != psutil.STATUS_ZOMBIE:
                child.terminate()
                survivors.append(child)
        except psutil.NoSuchProcess:
            pass
    _, alive = psutil.wait_procs(survivors, timeout=3)
    for child in alive:
        try:
            child.kill()
        except psutil.NoSuchProcess:
            pass
    psutil.wait_procs(alive, timeout=3)
    for child in alive:
        try:
            if child.is_running() and child.status() != psutil.STATUS_ZOMBIE:
                raise RuntimeError(f"Simulation descendant {child.pid} did not exit")
        except psutil.NoSuchProcess:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--multi", action="store_true")
    parser.add_argument("--external", action="store_true")
    parser.add_argument("--chassis", action="store_true")
    parser.add_argument(
        "--gazebo-version",
        choices=("ign", "gz"),
        default="ign",
    )
    parser.add_argument("--logs", default=None)
    args = parser.parse_args()
    if args.chassis and (args.multi or args.external):
        parser.error("--chassis requires one exclusive chassis endpoint")
    if args.multi and args.external:
        parser.error("--external currently validates one protected endpoint")
    directory = Path(args.logs or tempfile.mkdtemp(prefix="swerve_validation_"))
    directory.mkdir(parents=True, exist_ok=True)
    processes = []
    streams = []
    descendants = {}
    environment = simulation_environment(os.environ)
    print(f"Gazebo partition: {environment['GZ_PARTITION']}", flush=True)

    def start(name, command):
        stream = (directory / name).open("w")
        streams.append(stream)
        process = subprocess.Popen(
            command,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=environment,
        )
        processes.append(process)

    try:
        command = [
            "ros2",
            "launch",
            "swerve_gazebo_sim",
            f"demo_{args.gazebo_version}.launch.py",
            "headless:=true",
            "publish_ground_truth:=true",
        ]
        if args.chassis:
            command += ["chassis_control:=true"]
        if args.external:
            command += ["external_joint_control:=true"]
        if args.multi:
            command += ["namespace:=robot1", "robot_name:=robot1"]
        start("demo.log", command)
        probe = [
            sys.executable,
            str(
                Path(__file__).with_name(
                    "smoke_chassis.py"
                    if args.chassis
                    else "smoke_endpoint.py" if args.external else "smoke_motion.py"
                )
            ),
            "--gazebo-version",
            args.gazebo_version,
        ]
        if args.multi:
            start(
                "peer.log",
                [
                    "ros2",
                    "launch",
                    "swerve_gazebo_sim",
                    f"spawn_{args.gazebo_version}.launch.py",
                    "namespace:=robot2",
                    "robot_name:=robot2",
                    "x:=5",
                    "publish_ground_truth:=true",
                    "start_bridge:=false",
                ],
            )
            probe += ["--namespace", "robot1", "--stationary-peer", "robot2"]
        critical = list(processes)
        result = subprocess.Popen(probe, start_new_session=True, env=environment)
        processes.append(result)
        deadline = time.monotonic() + 300
        while (
            result.poll() is None
            and all(process.poll() is None for process in critical)
            and time.monotonic() < deadline
        ):
            track_descendants(processes, descendants)
            time.sleep(0.1)
        returncode = result.poll()
        if returncode is None:
            returncode = 1
            print("FAIL: launch exited early or test timed out", flush=True)
        if returncode:
            for name in ("demo.log", "peer.log"):
                path = directory / name
                if path.exists():
                    print(path.read_text()[-16000:])
        return returncode
    finally:
        track_descendants(processes, descendants)
        for process in reversed(processes):
            if process.poll() is None:
                # Let ros2 launch forward SIGINT once to its children.
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        stop_descendants(descendants)
        for stream in streams:
            stream.close()
        print(f"Launch logs: {directory}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
