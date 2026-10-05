"""Read independent Gazebo truth for tests without a ROS bridge or publisher."""

import json
import os
import signal
import subprocess
import threading
from collections import deque
from types import SimpleNamespace


def decode_odometry(data):
    """Adapt Gazebo Protobuf JSON to the attributes consumed by the motion probe."""
    stamp = data["header"]["stamp"]
    pose = data["pose"]
    twist = data["twist"]

    def vector(value, components="xyz"):
        return SimpleNamespace(**{key: float(value.get(key, 0)) for key in components})

    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=int(stamp.get("sec", 0)), nanosec=int(stamp.get("nsec", 0))
            )
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(pose.get("position", {})),
                orientation=vector(pose.get("orientation", {}), "xyzw"),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=vector(twist.get("linear", {})),
                angular=vector(twist.get("angular", {})),
            )
        ),
    )


class GazeboTruth:
    def __init__(self, topic, family, history_length=512):
        self._latest = None
        self._error = None
        self._history = deque(maxlen=history_length)
        self._lock = threading.Lock()
        self.process = subprocess.Popen(
            [family, "topic", "-e", "-t", topic, "--json-output"],
            stdout=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                if line.lstrip().startswith("{"):
                    message = decode_odometry(json.loads(line))
                    with self._lock:
                        self._latest = message
                        self._history.append(message)
        except (ValueError, KeyError, TypeError) as error:
            self._error = str(error)

    @property
    def latest(self):
        if self._error is not None:
            raise RuntimeError(f"Invalid Gazebo truth message: {self._error}")
        if self.process.poll() is not None:
            raise RuntimeError("Gazebo truth reader exited before the test finished")
        return self._latest

    @property
    def history(self):
        with self._lock:
            return tuple(self._history)

    def close(self):
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        if self.process.poll() is None:
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3)
        self.thread.join(timeout=3)
        self.process.stdout.close()
