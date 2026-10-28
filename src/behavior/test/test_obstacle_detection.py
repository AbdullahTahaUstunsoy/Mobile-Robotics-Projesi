import math

import numpy as np
from behavior.obstacle_detection import CLEAR, STOP, WARN, cluster, detect

# Same layout as the Gazebo lidar: 360 rays over -pi..pi
N = 360
ANGLE_MIN = -math.pi
INC = 2 * math.pi / N
RANGE_MIN, RANGE_MAX = 0.12, 8.0


def empty_scan():
    return np.full(N, math.inf)


def ray(angle_deg):
    return int(round((math.radians(angle_deg) - ANGLE_MIN) / INC)) % N


def add_wall_ahead(ranges, x, half_width=0.4):
    """Flat wall perpendicular to the x axis at distance x."""
    for i in range(N):
        a = ANGLE_MIN + i * INC
        if math.cos(a) > 0:
            r = x / math.cos(a)
            if abs(r * math.sin(a)) <= half_width:
                ranges[i] = r
    return ranges


def run(ranges):
    return detect(ranges, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)


def test_empty_scan_is_clear():
    result = run(empty_scan())
    assert result.state == CLEAR
    assert math.isinf(result.front_distance)
    assert result.obstacles == []


def test_wall_ahead_states():
    assert run(add_wall_ahead(empty_scan(), 2.0)).state == CLEAR
    assert run(add_wall_ahead(empty_scan(), 0.8)).state == WARN
    result = run(add_wall_ahead(empty_scan(), 0.4))
    assert result.state == STOP
    assert abs(result.front_distance - 0.4) < 1e-6
    assert result.obstacles[0].in_corridor


def test_side_obstacle_is_not_in_corridor():
    # Planter row 0.5 m to the right, running along the aisle
    ranges = empty_scan()
    for i in range(N):
        a = ANGLE_MIN + i * INC
        if math.sin(a) < -0.5:
            ranges[i] = -0.5 / math.sin(a)
    result = run(ranges)
    assert result.state == CLEAR
    assert len(result.obstacles) == 1
    assert not result.obstacles[0].in_corridor


def test_two_separate_obstacles():
    ranges = empty_scan()
    for deg in range(-5, 6):
        ranges[ray(deg)] = 1.5
    for deg in range(85, 96):
        ranges[ray(deg)] = 0.7
    result = run(ranges)
    assert len(result.obstacles) == 2
    assert result.obstacles[0].distance < result.obstacles[1].distance


def test_points_on_robot_body_are_ignored():
    # Camera mast behind-right of the lidar
    ranges = empty_scan()
    for deg in range(-155, -149):
        ranges[ray(deg)] = 0.22
    result = run(ranges)
    assert result.obstacles == []


def test_out_of_range_rays_are_ignored():
    ranges = empty_scan()
    ranges[ray(0)] = 0.05  # below range_min
    ranges[ray(1)] = np.nan
    assert run(ranges).state == CLEAR


def test_cluster_joins_across_scan_seam():
    points = np.array([[-1.0, -0.05], [-1.0, -0.02], [0.0, 2.0], [-1.0, 0.02], [-1.0, 0.05]])
    assert len(cluster(points, 0.2, wraps_around=False)) == 3
    assert len(cluster(points, 0.2, wraps_around=True)) == 2
