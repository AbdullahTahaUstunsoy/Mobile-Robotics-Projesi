from behavior.obstacle_detection import CLEAR as LIDAR_CLEAR
from behavior.obstacle_detection import STOP, WARN
from behavior.safety import BUMPER, CLEAR, OBSTACLE, SafetyParams, filter_cmd, stop_reason

PARAMS = SafetyParams(bumper_hold_time=1.0)


def test_no_contact_no_obstacle_is_clear():
    assert stop_reason(None, 10.0, LIDAR_CLEAR, PARAMS) == CLEAR
    assert stop_reason(None, 10.0, None, PARAMS) == CLEAR


def test_bumper_blocks_for_hold_time():
    assert stop_reason(10.0, 10.0, LIDAR_CLEAR, PARAMS) == BUMPER
    assert stop_reason(10.0, 10.9, LIDAR_CLEAR, PARAMS) == BUMPER
    assert stop_reason(10.0, 11.0, LIDAR_CLEAR, PARAMS) == CLEAR


def test_lidar_stop_blocks_but_warn_does_not():
    assert stop_reason(None, 10.0, STOP, PARAMS) == OBSTACLE
    assert stop_reason(None, 10.0, WARN, PARAMS) == CLEAR


def test_lidar_stop_can_be_disabled():
    params = SafetyParams(obstacle_stop=False)
    assert stop_reason(None, 10.0, STOP, params) == CLEAR


def test_bumper_wins_over_obstacle():
    assert stop_reason(10.0, 10.5, STOP, PARAMS) == BUMPER


def test_clear_passes_command_through():
    assert filter_cmd(0.3, 0.8, CLEAR) == (0.3, 0.8)


def test_stop_removes_forward_but_keeps_turning():
    assert filter_cmd(0.3, 0.8, BUMPER) == (0.0, 0.8)
    assert filter_cmd(0.3, 0.0, OBSTACLE) == (0.0, 0.0)


def test_stop_allows_reversing():
    assert filter_cmd(-0.3, 0.0, BUMPER) == (-0.3, 0.0)
