"""Start an isolated test world, run the motion probe and clean up all processes."""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--multi", action="store_true")
    parser.add_argument(
        "--gazebo-version",
        choices=("ign", "gz"),
        default="ign",
    )
    parser.add_argument("--logs", default=None)
    args = parser.parse_args()
    directory = Path(args.logs or tempfile.mkdtemp(prefix="swerve_validation_"))
    directory.mkdir(parents=True, exist_ok=True)
    processes = []
    streams = []

    def start(name, command):
        stream = (directory / name).open("w")
        streams.append(stream)
        process = subprocess.Popen(
            command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True
        )
        processes.append(process)

    try:
        command = [
            "ros2",
            "launch",
            "swerve_gazebo_sim",
            "demo.launch.py",
            f"gazebo_version:={args.gazebo_version}",
            "headless:=true",
            "publish_ground_truth:=true",
        ]
        if args.multi:
            command += ["namespace:=robot1", "robot_name:=robot1"]
        start("demo.log", command)
        probe = [sys.executable, str(Path(__file__).with_name("smoke_motion.py"))]
        if args.multi:
            start(
                "peer.log",
                [
                    "ros2",
                    "launch",
                    "swerve_gazebo_sim",
                    "spawn.launch.py",
                    f"gazebo_version:={args.gazebo_version}",
                    "namespace:=robot2",
                    "robot_name:=robot2",
                    "x:=5",
                    "publish_ground_truth:=true",
                ],
            )
            probe += ["--namespace", "robot1", "--stationary-peer", "robot2"]
        critical = list(processes)
        result = subprocess.Popen(probe, start_new_session=True)
        processes.append(result)
        deadline = time.monotonic() + 180
        while (
            result.poll() is None
            and all(process.poll() is None for process in critical)
            and time.monotonic() < deadline
        ):
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
        for stream in streams:
            stream.close()
        print(f"Launch logs: {directory}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
