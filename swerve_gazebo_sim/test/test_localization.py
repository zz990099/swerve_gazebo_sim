"""Check correction direction, changing drift and source-time interpolation."""

import math

import pytest
from swerve_gazebo_sim.planar_localization import (
    PoseHistory,
    compose,
    inverse,
    map_to_odom,
)


def test_nonidentity_frames_and_accumulating_lateral_drift():
    fixed = (2, -1, 0.4)
    for distance in (0, 3, 6, 12):
        wheel = (0, distance, 0)
        truth = compose(fixed, (0.02 * distance, distance, 0.01))
        correction = map_to_odom(truth, wheel)
        assert compose(correction, wheel) == pytest.approx(truth)
        goal_map = compose(fixed, (0, 12, 0))
        goal_odom = compose(inverse(correction), goal_map)
        assert compose(correction, goal_odom) == pytest.approx(goal_map)
    assert abs(goal_odom[0]) > 0.1


def test_source_time_wrap_interpolation_and_missing_samples():
    history = PoseHistory()
    assert history.add(1, (0, 0, math.pi - 0.02))
    assert history.add(1.04, (0.04, 0, -math.pi + 0.02))
    assert history.at(1.02) == pytest.approx((0.02, 0, math.pi))
    assert history.at(0.99) is None
    assert history.at(1.05) is None
    assert not history.add(1.04, (0, 0, 0))
    assert not history.add(float("nan"), (0, 0, 0))
    history.add(2, (1, 1, 0))
    assert history.at(1.5) is None
