"""Safety filter for velocity commands. Pure Python, no ROS, so it can be unit tested.

Forward motion (linear.x > 0) is blocked while
    BUMPER    the front bumper touched something less than bumper_hold_time ago
    OBSTACLE  the obstacle detector reports STOP for the corridor ahead (if obstacle_stop is on)
Reversing and turning in place stay allowed, so the robot can always get away from the obstacle.
"""

from dataclasses import dataclass

from behavior.obstacle_detection import STOP

CLEAR, BUMPER, OBSTACLE = 'CLEAR', 'BUMPER', 'OBSTACLE'


@dataclass
class SafetyParams:
    bumper_hold_time: float = 1.0   # s: forward stays blocked this long after the last contact
    obstacle_stop: bool = True      # also block forward on the lidar STOP state


def stop_reason(last_contact_time, now, obstacle_state, params):
    """Return BUMPER, OBSTACLE or CLEAR. last_contact_time is None if the bumper never touched."""
    if last_contact_time is not None and now - last_contact_time < params.bumper_hold_time:
        return BUMPER
    if params.obstacle_stop and obstacle_state == STOP:
        return OBSTACLE
    return CLEAR


def filter_cmd(linear, angular, reason):
    """Return the (linear, angular) command that is safe to send for this stop reason."""
    if reason != CLEAR:
        linear = min(linear, 0.0)
    return linear, angular
