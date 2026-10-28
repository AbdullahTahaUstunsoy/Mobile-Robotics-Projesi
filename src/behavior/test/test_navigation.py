import math

import pytest

from behavior.navigation import (DriveParams, drive_to, follow_line, line_offsets, TurnGains, turn_rate,
                                 world_to_odom, wrap, yaw_from_quaternion)


def test_wrap_keeps_angles_in_range():
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert wrap(0.3) == pytest.approx(0.3)


def test_yaw_from_quaternion():
    half = math.pi / 4      # 90 deg about z
    assert yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(math.pi / 2)


def test_world_to_odom_uses_spawn_pose():
    assert world_to_odom(6.8, 0.65, (-0.5, 0.65, 0.0)) == pytest.approx((7.3, 0.0))
    # Spawned facing +y: a point 1 m north of the spawn is 1 m ahead in odom
    assert world_to_odom(0.0, 1.0, (0.0, 0.0, math.pi / 2)) == pytest.approx((1.0, 0.0))


def test_turn_rate_is_clamped_and_signed():
    gains = TurnGains(gain=1.0, min_speed=0.1, max_speed=0.3, deadband=math.radians(2))
    assert turn_rate(math.radians(1), gains) == 0.0
    assert turn_rate(math.radians(4), gains) == pytest.approx(0.1)
    assert turn_rate(-2.0, gains) == pytest.approx(-0.3)
    assert turn_rate(0.2, gains) == pytest.approx(0.2)


def test_drive_to_turns_in_place_when_facing_away():
    (linear, angular), remaining = drive_to((0.0, 0.0, 0.0), math.pi / 2, (1.0, 0.0))
    assert remaining == pytest.approx(1.0)
    assert linear == 0.0
    assert angular < 0.0     # target is to the right


def test_drive_to_slows_down_near_target():
    params = DriveParams()
    (linear, _), _ = drive_to((0.0, 0.0, 0.0), 0.0, (1.0, 0.0), params)
    assert linear == params.speed
    (linear, _), _ = drive_to((0.9, 0.0, 0.0), 0.0, (1.0, 0.0), params)
    assert linear == params.slow_speed


def test_line_offsets_signs():
    # East-bound line: north of it is to the left
    assert line_offsets((2.0, 0.1), (0.0, 0.0), (5.0, 0.0)) == pytest.approx((3.0, 0.1))
    # West-bound line: north of it is to the right
    assert line_offsets((2.0, 0.1), (5.0, 0.0), (0.0, 0.0)) == pytest.approx((2.0, -0.1))


@pytest.mark.parametrize('start, end', [((0.0, 0.0), (5.0, 0.0)), ((5.0, 0.0), (0.0, 0.0))])
def test_follow_line_steers_back_to_the_line(start, end):
    line_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    (_, angular), _, lateral = follow_line((2.5, 0.08), line_yaw, start, end)
    # Turn towards the line: right when it is to our right (lateral > 0 means we are left of it)
    assert math.copysign(1, angular) == -math.copysign(1, lateral)
