# SPDX-License-Identifier: Apache-2.0
"""Strict current chassis configuration shared by model, plugin and launch."""

import math
import re

import yaml

GEOMETRY = (
    "wheelbase",
    "track_width",
    "wheel_radius",
    "wheel_width",
    "body_length",
    "body_width",
    "body_height",
    "ground_clearance",
    "body_mass",
    "wheel_mass",
    "steering_mass",
)

CHASSIS_PARAMETERS = (
    "steering_limit",
    "max_wheel_speed",
    "max_wheel_acceleration",
    "max_steering_rate",
    "steering_alignment_tolerance",
    "steering_alignment_duration",
    "mode_switch_timeout",
    "stopped_wheel_speed",
    "cmd_timeout",
    "wall_timeout",
    "update_rate",
    "drive_steering_limit",
    "max_linear_speed",
    "max_angular_speed",
    "max_linear_acceleration",
    "max_angular_acceleration",
)
ODOMETRY_DEFAULTS = {
    "publish_odom": True,
    "odom_topic": "odom",
    "odom_frame": "",
    "odom_child_frame": "",
}
OBSERVER_DEFAULTS = {
    "publish_odom": True,
    "publish_odom_tf": True,
    "odom_topic": "odom",
    "odom_frame": "",
    "odom_child_frame": "",
    "pose_variance": 0.02,
    "twist_variance": 0.02,
    "feedback_timeout": 0.25,
}
CONTROL = CHASSIS_PARAMETERS + tuple(OBSERVER_DEFAULTS)

GAZEBO_VARIANTS = {
    "ign": {
        "name": "ign",
        "spawn_launch": "spawn_ign.launch.py",
        "sim_version": "6",
        "world": "empty_ign.sdf",
        "bridge": "bridge_ign.yaml",
    },
    "gz": {
        "name": "gz",
        "spawn_launch": "spawn_gz.launch.py",
        "sim_version": "8",
        "world": "empty_gz.sdf",
        "bridge": "bridge_gz.yaml",
    },
}


IGNITION = GAZEBO_VARIANTS["ign"]
HARMONIC = GAZEBO_VARIANTS["gz"]


def gazebo_variant(name):
    """Return the explicit Gazebo integration settings for a supported family."""
    try:
        return GAZEBO_VARIANTS[name]
    except KeyError as error:
        supported = ", ".join(GAZEBO_VARIANTS)
        raise ValueError(f"gazebo_version must be one of: {supported}") from error


def names(namespace, robot_name, prefix):
    namespace = "/" + namespace.strip("/") if namespace.strip("/") else ""
    for token in namespace.split("/"):
        if token and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", token):
            raise ValueError("namespace must contain valid ROS names")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", robot_name):
        raise ValueError("robot_name must contain letters, digits and underscores")
    if prefix == "auto":
        prefix = namespace.strip("/").replace("/", "_") + "_" if namespace else ""
    if prefix and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", prefix):
        raise ValueError("prefix must contain letters, digits and underscores")
    return namespace, prefix


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f"Duplicate configuration key: {key}")
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping
)


def validate_control(c):
    for key in CONTROL:
        value = c[key]
        if key in ("publish_odom", "publish_odom_tf"):
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be boolean")
        elif key in ("odom_topic", "odom_frame", "odom_child_frame"):
            if not isinstance(value, str) or any(x.isspace() for x in value):
                raise ValueError(f"Invalid {key}")
            if key == "odom_topic" and not value:
                raise ValueError("odom_topic is required")
            if key != "odom_topic" and value.startswith("/"):
                raise ValueError("Frame names must not start with /")
        elif (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{key} must be finite and positive")
    if c["update_rate"] != 100 or abs(c["steering_limit"] - math.pi / 2) > 1e-9:
        raise ValueError(
            "This chassis requires 100 Hz control and pi/2 steering travel"
        )
    if c["mode_switch_timeout"] <= c["steering_alignment_duration"]:
        raise ValueError("Mode deadline must exceed alignment dwell")
    if c["drive_steering_limit"] < c["steering_alignment_tolerance"]:
        raise ValueError("Moving steering limit must cover alignment tolerance")
    if c["odom_frame"] and c["odom_frame"] == c["odom_child_frame"]:
        raise ValueError("Odometry frames must differ")


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        cfg = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(cfg, dict) or set(cfg) != {"geometry", "control"}:
        raise ValueError("Expected exactly geometry and control")
    for section, keys in (("geometry", GEOMETRY), ("control", CONTROL)):
        if not isinstance(cfg[section], dict) or set(cfg[section]) != set(keys):
            raise ValueError(f"{section} must contain exactly {keys}")
    g = cfg["geometry"]
    for key, value in g.items():
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"Invalid geometry.{key}")
    if (
        g["track_width"] <= g["body_width"] + 2 * g["wheel_radius"]
        or g["wheelbase"] <= 2 * g["wheel_radius"]
    ):
        raise ValueError("Wheel steering sweep must clear chassis and adjacent wheels")
    validate_control(cfg["control"])
    return cfg


def controller_config(template, cfg, namespace, prefix):
    with open(template, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    for node in data.values():
        node["ros__parameters"]["use_sim_time"] = True
    data["controller_manager"]["ros__parameters"]["update_rate"] = int(
        cfg["control"]["update_rate"]
    )
    parameters = {key: float(cfg["control"][key]) for key in CHASSIS_PARAMETERS}
    parameters.update(
        {
            key: float(cfg["geometry"][key])
            for key in ("wheelbase", "track_width", "wheel_radius")
        }
    )
    parameters.update(
        update_rate=int(cfg["control"]["update_rate"]),
        use_sim_time=True,
        body_frame=prefix + "base_footprint",
        simulation_time_interface=prefix + "execution_clock/simulation_time",
    )
    for kind in ("steering", "wheel"):
        parameters[kind + "_joints"] = [
            prefix + corner + "_" + kind + "_joint"
            for corner in ("fl", "fr", "rl", "rr")
        ]
    data["chassis_controller"] = {"ros__parameters": parameters}
    return {f"{namespace}/{key}": value for key, value in data.items()}
