# SPDX-License-Identifier: Apache-2.0
"""Spawn the chassis with Harmonic Gazebo integration."""

from swerve_gazebo_sim.bringup import HARMONIC
from swerve_gazebo_sim.launch_support import generate_spawn_launch_description


def generate_launch_description():
    return generate_spawn_launch_description(HARMONIC)
