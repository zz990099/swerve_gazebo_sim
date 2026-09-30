# SPDX-License-Identifier: Apache-2.0
"""Spawn the chassis with Ignition Gazebo integration."""

from swerve_gazebo_sim.bringup import IGNITION
from swerve_gazebo_sim.launch_support import generate_spawn_launch_description


def generate_launch_description():
    return generate_spawn_launch_description(IGNITION)
