import math

import pytest

from swerve_gazebo_sim.kinematics import SwerveKinematics, integrate_pose


@pytest.fixture
def model():
    return SwerveKinematics(0.6, 0.5, 0.1)


@pytest.mark.parametrize('twist', [(0.5, 0, 0), (-0.5, 0, 0), (0, 0.5, 0),
                                   (0, -0.5, 0), (0, 0, 0.5), (0, 0, -0.5),
                                   (0.3, 0.2, 0.4), (-0.2, 0.3, -0.5)])
def test_wheel_velocity_matches_rigid_body_motion(model, twist):
    speeds, angles = model.inverse(*twist, [0.0] * 4)
    for (x, y), speed, angle in zip(model.positions, speeds, angles):
        assert -math.pi / 2 <= angle <= math.pi / 2
        assert speed * 0.1 * math.cos(angle) == pytest.approx(twist[0] - twist[2] * y)
        assert speed * 0.1 * math.sin(angle) == pytest.approx(twist[1] + twist[2] * x)


def test_rectangular_chassis_spin(model):
    speeds, _ = model.inverse(0, 0, 1, [0.0] * 4)
    assert abs(speeds[0]) == pytest.approx(math.hypot(0.3, 0.25) / 0.1)


def test_reverse_keeps_steering_forward(model):
    speeds, angles = model.inverse(-0.5, 0, 0, [0.0] * 4)
    assert speeds == pytest.approx([-5.0] * 4)
    assert angles == pytest.approx([0.0] * 4)


def test_zero_preserves_angles(model):
    angles = [0.1, -0.2, 0.3, -0.4]
    speeds, result = model.inverse(0, 0, 0, angles)
    assert speeds == [0.0] * 4
    assert result == angles


def test_encoder_yaw_estimate(model):
    # Analytic pure spin, independent of inverse().
    vectors = [(-y, x) for x, y in model.positions]
    speeds = [math.hypot(x, y) / 0.1 for x, y in vectors]
    angles = [math.atan2(y, x) for x, y in vectors]
    assert model.forward(speeds, angles) == pytest.approx((0, 0, 1))


def test_arc_integration():
    assert integrate_pose((0, 0, 0), (1, 0, 1), math.pi / 2) == pytest.approx((1, 1, math.pi / 2))
    assert integrate_pose((0, 0, math.pi / 2), (1, 0, 0), 1) == pytest.approx((0, 1, math.pi / 2))


@pytest.mark.parametrize('geometry', [(0, .5, .1), (.6, -.5, .1), (.6, .5, float('nan'))])
def test_invalid_geometry(geometry):
    with pytest.raises(ValueError):
        SwerveKinematics(*geometry)
