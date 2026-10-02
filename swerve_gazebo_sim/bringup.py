# SPDX-License-Identifier: Apache-2.0
"""Configuration shared by launch and model validation tests."""

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
CONTROL = (
    "external_joint_control",
    "update_rate",
    "cmd_timeout",
    "feedback_timeout",
    "max_wheel_speed",
    "max_wheel_acceleration",
    "max_steering_rate",
    "steering_alignment_tolerance",
    "mode_linear_enter_threshold",
    "mode_linear_exit_threshold",
    "mode_angular_enter_threshold",
    "mode_angular_exit_threshold",
    "mode_dwell_time",
    "steering_alignment_duration",
    "mode_switch_timeout",
    "stopped_wheel_speed",
    "status_publish_rate",
    "steering_limit",
    "publish_odom_tf",
    "publish_odom",
    "odom_topic",
    "odom_frame",
    "odom_child_frame",
    "pose_variance",
    "twist_variance",
)

TRANSITION_DEFAULTS = {
    "mode_linear_enter_threshold": 0.01,
    "mode_linear_exit_threshold": 0.005,
    "mode_angular_enter_threshold": 0.01,
    "mode_angular_exit_threshold": 0.005,
    "mode_dwell_time": 0.1,
    "steering_alignment_duration": 0.05,
    "mode_switch_timeout": 5.0,
    "stopped_wheel_speed": 0.05,
    "status_publish_rate": 10.0,
}

ODOMETRY_DEFAULTS = {
    "publish_odom": True,
    "odom_topic": "odom",
    "odom_frame": "",
    "odom_child_frame": "",
}
COMMAND_DEFAULTS = {"external_joint_control": False}

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


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict) or set(cfg) != {"geometry", "control"}:
        raise ValueError("Configuration must contain geometry and control mappings")
    if isinstance(cfg["control"], dict):
        for key, value in {
            **TRANSITION_DEFAULTS,
            **ODOMETRY_DEFAULTS,
            **COMMAND_DEFAULTS,
        }.items():
            cfg["control"].setdefault(key, value)
    for section, keys in (("geometry", GEOMETRY), ("control", CONTROL)):
        if not isinstance(cfg[section], dict) or set(cfg[section]) != set(keys):
            raise ValueError(f'{section} must contain exactly: {", ".join(keys)}')
        for key, value in cfg[section].items():
            if key in ("publish_odom", "publish_odom_tf", "external_joint_control"):
                if not isinstance(value, bool):
                    raise ValueError(f"{key} must be boolean")
            elif key in ("odom_topic", "odom_frame", "odom_child_frame"):
                continue  # Validated with the controller's startup parameters below.
            elif (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
                or (
                    value == 0
                    and key not in ("mode_dwell_time", "steering_alignment_duration")
                )
            ):
                raise ValueError(f"{section}.{key} must be finite and positive")
            else:
                cfg[section][key] = float(value)
    validate_control(cfg["control"])
    if not math.isclose(cfg["control"]["steering_limit"], math.pi / 2, abs_tol=1e-9):
        raise ValueError("steering_limit must be pi/2 for this release")
    if not cfg["control"]["update_rate"].is_integer():
        raise ValueError("update_rate must be a positive integer frequency")
    geometry = cfg["geometry"]
    if geometry["track_width"] <= geometry["body_width"] + 2 * geometry["wheel_radius"]:
        raise ValueError(
            "track_width must clear the body plus the steering wheel sweep"
        )
    if geometry["wheelbase"] <= 2 * geometry["wheel_radius"]:
        raise ValueError("wheelbase must exceed the wheel diameter")
    return cfg


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


def controller_config(template, cfg, namespace, prefix):
    with open(template, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    for node in data.values():
        node["ros__parameters"]["use_sim_time"] = True
    data["controller_manager"]["ros__parameters"]["update_rate"] = int(
        cfg["control"]["update_rate"]
    )
    controllers = (
        ("steering_controller", "steering"),
        ("wheel_controller", "wheel"),
    )
    for controller_name, joint_type in controllers:
        data[controller_name]["ros__parameters"]["joints"] = [
            f"{prefix}{corner}_{joint_type}_joint"
            for corner in ("fl", "fr", "rl", "rr")
        ]
    if cfg["control"]["external_joint_control"]:
        manager = data["controller_manager"]["ros__parameters"]
        del manager["steering_controller"], manager["wheel_controller"]
        manager["guarded_joint_controller"] = {
            "type": "swerve_gazebo_sim/GuardedJointController"
        }
        guarded = {"use_sim_time": True}
        for kind in ("steering", "wheel"):
            guarded[f"{kind}_joints"] = data[f"{kind}_controller"]["ros__parameters"][
                "joints"
            ]
            del data[f"{kind}_controller"]
        guarded.update(
            max_wheel_speed_radps=cfg["control"]["max_wheel_speed"],
            steering_limit_rad=cfg["control"]["steering_limit"],
            stopped_wheel_radps=cfg["control"]["stopped_wheel_speed"],
        )
        data["guarded_joint_controller"] = {"ros__parameters": guarded}
    return {f"{namespace}/{key}": value for key, value in data.items()}


def validate_control(configuration):
    """Validate controller startup parameters, including hysteresis ordering."""
    for key in CONTROL:
        value = configuration[key]
        if key in ("publish_odom", "publish_odom_tf", "external_joint_control"):
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be boolean")
            continue
        if key in ("odom_topic", "odom_frame", "odom_child_frame"):
            if not isinstance(value, str):
                raise ValueError(f"{key} must be a string")
            if key == "odom_topic" and not value.strip():
                raise ValueError("odom_topic must not be empty")
            if (
                key != "odom_topic"
                and value
                and (
                    value.startswith("/")
                    or any(character.isspace() for character in value)
                )
            ):
                raise ValueError(f"{key} must not start with '/' or contain whitespace")
            continue
        allow_zero = key in ("mode_dwell_time", "steering_alignment_duration")
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
            or (value == 0 and not allow_zero)
        ):
            raise ValueError(
                f"control.{key} must be finite and {'nonnegative' if allow_zero else 'positive'}"
            )
    for kind in ("linear", "angular"):
        if (
            configuration[f"mode_{kind}_exit_threshold"]
            >= configuration[f"mode_{kind}_enter_threshold"]
        ):
            raise ValueError(f"{kind} exit threshold must be below its enter threshold")
    if (
        configuration["mode_switch_timeout"]
        <= configuration["mode_dwell_time"]
        + configuration["steering_alignment_duration"]
    ):
        raise ValueError(
            "mode_switch_timeout must exceed dwell plus alignment duration"
        )
