"""Heading and motion controllers for the greenhouse mission. Pure Python, no ROS, so it can be unit tested.

Poses are (x, y, yaw) in the odom frame; angles are radians. Each controller returns a
(linear, angular) command for /cmd_vel_raw.
"""

import math
from dataclasses import dataclass


def wrap(angle):
    """Angle wrapped to -pi .. pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def world_to_odom(x, y, spawn):
    """Gazebo world point -> odom point. The odom origin is the spawn pose (x, y, yaw)."""
    sx, sy, syaw = spawn
    dx, dy = x - sx, y - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    return c * dx - s * dy, s * dx + c * dy


@dataclass
class TurnGains:
    gain: float = 1.2
    min_speed: float = 0.12  # below this the robot does not overcome friction
    max_speed: float = 0.35
    deadband: float = math.radians(3)


@dataclass
class DriveParams:
    speed: float = 0.08
    slow_speed: float = 0.05
    slow_distance: float = 0.15
    goal_tolerance: float = 0.05
    turn_in_place: float = math.radians(10)  # heading error above which the robot stops and turns


def turn_rate(error, gains=None):
    """Proportional turn rate, clamped to min..max speed, zero inside the deadband."""
    if gains is None:
        gains = TurnGains()
    if abs(error) <= gains.deadband:
        return 0.0
    return math.copysign(
        min(gains.max_speed, max(gains.min_speed, gains.gain * abs(error))), error
    )


def forward_speed(heading_error, remaining, params=None):
    if params is None:
        params = DriveParams()
    if abs(heading_error) > params.turn_in_place:
        return 0.0
    return params.slow_speed if remaining < params.slow_distance else params.speed


def drive_to(pose, heading, target, params=None, gains=None):
    """Head for target (x, y). Returns ((linear, angular), remaining distance)."""
    if params is None:
        params = DriveParams()
    if gains is None:
        gains = TurnGains()
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    remaining = math.hypot(dx, dy)
    error = wrap(math.atan2(dy, dx) - heading)
    return (forward_speed(error, remaining, params), turn_rate(error, gains)), remaining


def line_offsets(pose, start, end):
    """(along, lateral) of pose relative to the line start -> end.

    along:   distance left to end, measured along the line (negative once past it)
    lateral: signed offset from the line, positive to the left of the travel direction
    """
    lx, ly = end[0] - start[0], end[1] - start[1]
    length = math.hypot(lx, ly)
    ux, uy = lx / length, ly / length
    along = ux * (end[0] - pose[0]) + uy * (end[1] - pose[1])
    lateral = ux * (pose[1] - start[1]) - uy * (pose[0] - start[0])
    return along, lateral


def follow_line(
    pose, heading, start, end, correction_gain=2.0, max_correction=None, params=None, gains=None
):
    """Drive along the line start -> end, steering back towards it.

    Returns ((linear, angular), remaining distance along the line, lateral offset).
    """
    if params is None:
        params = DriveParams()
    if gains is None:
        gains = TurnGains()
    if max_correction is None:
        max_correction = math.radians(10)
    remaining, lateral = line_offsets(pose, start, end)
    line_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    correction = max(-max_correction, min(max_correction, math.atan(-correction_gain * lateral)))
    error = wrap(line_yaw + correction - heading)
    return (forward_speed(error, remaining, params), turn_rate(error, gains)), remaining, lateral
