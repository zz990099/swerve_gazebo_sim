# SPDX-License-Identifier: Apache-2.0
"""Demo the chassis with Ignition Gazebo integration."""

from swerve_gazebo_sim.bringup import IGNITION
from swerve_gazebo_sim.launch_support import generate_demo_launch_description


def generate_launch_description():
    return generate_demo_launch_description(IGNITION)
