"""Collision bounds must follow the physical model rather than motion defaults."""

import math
import tempfile
import unittest
from pathlib import Path

import yaml

from swerve_gazebo_sim.bringup import controller_config, load_config, safety_parameters

ROOT = Path(__file__).resolve().parents[1]


class SafetyGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT / "config/swerve.yaml")

    def test_default_and_legacy_geometry(self):
        self.assertEqual(safety_parameters(self.cfg)["robot_radius_m"], 0.5)
        del self.cfg["safety"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.yaml"
            path.write_text(yaml.safe_dump(self.cfg))
            self.assertEqual(load_config(path)["safety"]["robot_radius"], "auto")

    def test_larger_body_reaches_execution_owner(self):
        self.cfg["geometry"]["body_length"] = 1.4
        self.cfg["control"]["chassis_control"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "large.yaml"
            path.write_text(yaml.safe_dump(self.cfg))
            cfg = load_config(path)
        expected = safety_parameters(cfg)
        self.assertEqual(expected["robot_radius_m"], 0.72)
        endpoint = controller_config(ROOT / "config/controllers.yaml", cfg, "", "")
        parameters = endpoint["/chassis_controller"]["ros__parameters"]
        for key, value in expected.items():
            self.assertEqual(parameters[key], value)

    def test_full_steering_sweep_is_enclosed(self):
        g = self.cfg["geometry"]
        g.update(wheelbase=1.0, track_width=0.8, wheel_width=0.1)
        radius = safety_parameters(self.cfg)["robot_radius_m"]
        # Independently rotate every corner of the wheel's XY projection.
        for step in range(361):
            angle = -math.pi / 2 + step * math.pi / 360
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for dx in (-g["wheel_radius"], g["wheel_radius"]):
                        for dy in (-g["wheel_width"] / 2, g["wheel_width"] / 2):
                            x = (
                                sx * g["wheelbase"] / 2
                                + math.cos(angle) * dx
                                - math.sin(angle) * dy
                            )
                            y = (
                                sy * g["track_width"] / 2
                                + math.sin(angle) * dx
                                + math.cos(angle) * dy
                            )
                            self.assertLessEqual(math.hypot(x, y), radius)

    def test_invalid_explicit_bounds_and_margin(self):
        self.cfg["geometry"]["body_length"] = 1.4
        for radius in (0.5, True, "bad", math.nan, math.inf):
            with self.subTest(radius=radius):
                self.cfg["safety"]["robot_radius"] = radius
                with self.assertRaises(ValueError):
                    safety_parameters(self.cfg)
        self.cfg["safety"]["robot_radius"] = 0.8
        for margin in (-1, True, math.nan, math.inf):
            with self.subTest(margin=margin):
                self.cfg["safety"]["collision_margin"] = margin
                with self.assertRaises(ValueError):
                    safety_parameters(self.cfg)
        self.cfg["safety"]["collision_margin"] = 0
        self.assertEqual(safety_parameters(self.cfg)["robot_radius_m"], 0.8)


if __name__ == "__main__":
    unittest.main()
