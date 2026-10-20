"""Obstacle detection on a 2D laser scan. Pure numpy, no ROS, so it can be unit tested.

All coordinates are in the lidar frame (x forward, y left):
    obstacles  scan points split into clusters wherever two neighbouring points are more than
               cluster_gap apart (a planter row, a wall, a box in the aisle, ...)
    corridor   the strip straight ahead that the robot will drive through: x > 0, |y| <= corridor_half_width.
               The nearest point inside it decides the state:
                   STOP   closer than stop_distance
                   WARN   closer than warn_distance
                   CLEAR  otherwise
Points inside the robot's own footprint (e.g. the camera mast) are dropped first.
"""

import math
from dataclasses import dataclass, field

import numpy as np

CLEAR, WARN, STOP = "CLEAR", "WARN", "STOP"


@dataclass
class DetectorParams:
    corridor_half_width: float = 0.25  # robot half width (0.175 m) + margin
    stop_distance: float = 0.5
    warn_distance: float = 1.0
    cluster_gap: float = 0.2  # neighbouring points farther apart start a new obstacle
    min_cluster_points: int = 3  # smaller clusters are treated as noise
    # Robot footprint in the lidar frame (lidar is 0.2 m ahead of the chassis center)
    footprint_min_x: float = -0.5
    footprint_max_x: float = 0.1
    footprint_half_width: float = 0.2


@dataclass
class Obstacle:
    points: np.ndarray  # (N, 2) x, y in scan order
    nearest: np.ndarray  # x, y of the point closest to the lidar
    distance: float  # range to nearest
    in_corridor: bool  # at least one point is inside the corridor ahead


@dataclass
class DetectionResult:
    state: str
    front_distance: float  # nearest point in the corridor, inf if none
    obstacles: list = field(default_factory=list)


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Return the (N, 2) valid points in scan order."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(ranges))
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r, a = ranges[valid], angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def remove_footprint(points, params):
    x, y = points[:, 0], points[:, 1]
    inside = (
        (x >= params.footprint_min_x)
        & (x <= params.footprint_max_x)
        & (np.abs(y) <= params.footprint_half_width)
    )
    return points[~inside]


def cluster(points, gap, wraps_around=False):
    """Split points (in scan order) into arrays of indices. With a full 360° scan, the
    first and last clusters are joined when they touch across the -pi/+pi seam."""
    if len(points) == 0:
        return []
    jumps = np.linalg.norm(np.diff(points, axis=0), axis=1) > gap
    starts = np.concatenate(([0], np.nonzero(jumps)[0] + 1, [len(points)]))
    clusters = [np.arange(s, e) for s, e in zip(starts[:-1], starts[1:], strict=True)]
    if wraps_around and len(clusters) > 1 and np.linalg.norm(points[-1] - points[0]) <= gap:
        clusters[0] = np.concatenate((clusters.pop(), clusters[0]))
    return clusters


def in_corridor(points, half_width):
    return (points[:, 0] > 0) & (np.abs(points[:, 1]) <= half_width)


def classify(front_distance, params):
    if front_distance < params.stop_distance:
        return STOP
    if front_distance < params.warn_distance:
        return WARN
    return CLEAR


def detect(ranges, angle_min, angle_increment, range_min, range_max, params=None):
    params = params or DetectorParams()
    points = scan_to_points(ranges, angle_min, angle_increment, range_min, range_max)
    points = remove_footprint(points, params)

    ahead = in_corridor(points, params.corridor_half_width)
    front_distance = float(points[ahead, 0].min()) if ahead.any() else math.inf

    full_circle = len(ranges) * abs(angle_increment) >= 2 * math.pi - 1e-3
    obstacles = []
    for idx in cluster(points, params.cluster_gap, wraps_around=full_circle):
        if len(idx) < params.min_cluster_points:
            continue
        pts = points[idx]
        dists = np.linalg.norm(pts, axis=1)
        nearest = int(np.argmin(dists))
        obstacles.append(
            Obstacle(
                points=pts,
                nearest=pts[nearest],
                distance=float(dists[nearest]),
                in_corridor=bool(ahead[idx].any()),
            )
        )
    obstacles.sort(key=lambda o: o.distance)

    return DetectionResult(classify(front_distance, params), front_distance, obstacles)
