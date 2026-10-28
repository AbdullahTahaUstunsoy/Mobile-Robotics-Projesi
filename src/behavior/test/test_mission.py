import math
import os

import pytest
import yaml

from behavior.mission import (build_stages, DONE, DriveTo, FAILED, FollowLine, load_mission, Mission, RUNNING,
                              StartCheck, TurnTo, Wait)
from behavior.navigation import world_to_odom, wrap

MISSION_FILE = os.path.join(os.path.dirname(__file__), '..', 'config', 'mission.yaml')
DT = 0.1


def load(spawn=(-0.5, 0.65, 0.0)):
    with open(MISSION_FILE) as f:
        return load_mission(yaml.safe_load(f), spawn)


def simulate(mission, pose=(0.0, 0.0, 0.0), max_time=3000.0, safety=lambda t: False, on_tick=None):
    """Unicycle robot driven by the mission until it finishes. Returns the final odom pose and the events."""
    x, y, a = pose
    t = 0.0
    events = []
    while mission.status == RUNNING and t < max_time:
        if on_tick:
            on_tick(mission, t)
        linear, angular = mission.step((x, y, a), a, safety(t), t)
        events += mission.events()
        if safety(t):
            linear = 0.0    # the safety filter removes forward motion
        a = wrap(a + angular * DT)
        x += linear * math.cos(a) * DT
        y += linear * math.sin(a) * DT
        t += DT
    events += mission.events()
    return (x, y, a), events


def test_mission_file_loads():
    start_check, stages = load()
    assert start_check.yaw_tolerance == pytest.approx(math.radians(10))
    assert [type(s) for s in stages[:3]] == [FollowLine, TurnTo, Wait]
    assert stages[0].end == pytest.approx((7.3, 0.0))      # world x = 6.8 -> odom x = 7.3


@pytest.mark.parametrize('spawn', [(-0.5, 0.65, 0.0), (-0.3, 0.6, 0.0)])
def test_full_mission_returns_home(spawn):
    _, stages = load(spawn)
    mission = Mission(stages)
    pose, events = simulate(mission)
    assert mission.status == DONE, events
    home = world_to_odom(-0.5, 0.65, spawn)
    assert math.hypot(pose[0] - home[0], pose[1] - home[1]) < 0.1
    assert abs(wrap(pose[2] + spawn[2])) < math.radians(5)     # parked facing world yaw 0
    assert events[-1] == 'MISSION COMPLETE'


def test_detections_are_reported_only_for_watched_labels():
    _, stages = load()
    mission = Mission(stages)

    def detect(mission, t):
        mission.on_detection('red_apple')         # watched in the aisles only
        if mission.stage.name == 'scan_checkpoints':
            mission.on_detection('green_checkpoint')

    simulate(mission, on_tick=detect)
    assert mission.seen == {
        'first_aisle': {'red_apple'},
        'scan_checkpoints': {'green_checkpoint'},
        'second_aisle': {'red_apple'},
    }


def test_drifting_off_the_aisle_fails():
    stage = FollowLine(name='aisle', start=(0.0, 0.0), end=(5.0, 0.0), max_lateral=0.12)
    mission = Mission([stage])
    mission.step((1.0, 0.2, 0.0), 0.0, False, 0.0)
    assert mission.status == FAILED
    assert 'lateral offset' in mission.events()[-1]


def test_stage_timeout_fails():
    mission = Mission([Wait(name='wait', duration=10.0, timeout=2.0)])
    _, events = simulate(mission)
    assert mission.status == FAILED
    assert 'timed out' in events[-1]


def test_safety_stop_pauses_driving_then_resumes():
    mission = Mission([DriveTo(name='drive', target=(1.0, 0.0))], safety_timeout=30.0)
    assert mission.step((0.0, 0.0, 0.0), 0.0, True, 0.0) == (0.0, 0.0)
    assert mission.step((0.0, 0.0, 0.0), 0.0, False, 1.0)[0] > 0.0
    assert mission.status == RUNNING
    assert any('resuming' in e for e in mission.events())


def test_safety_stop_does_not_block_turning():
    mission = Mission([TurnTo(name='turn', yaw=math.pi / 2)])
    assert mission.step((0.0, 0.0, 0.0), 0.0, True, 0.0)[1] > 0.0


def test_long_safety_stop_fails():
    mission = Mission([DriveTo(name='drive', target=(1.0, 0.0))], safety_timeout=5.0)
    _, events = simulate(mission, safety=lambda t: True)
    assert mission.status == FAILED
    assert 'safety stop longer than' in events[-1]


def test_start_check():
    check = StartCheck(xy_tolerance=0.15, yaw_tolerance=math.radians(10))
    assert check.ok((0.05, -0.05, 0.1))
    assert not check.ok((0.3, 0.0, 0.0))
    assert not check.ok((0.0, 0.0, math.radians(20)))


def test_unknown_stage_keys_are_rejected():
    with pytest.raises(ValueError):
        build_stages([{'type': 'turn_to', 'yaw_deg': 90, 'speed': 1.0}])
    with pytest.raises(ValueError):
        build_stages([{'type': 'drive_to', 'to': [1, 0], 'drive': {'sped': 0.1}}])
    with pytest.raises(ValueError):
        build_stages([{'type': 'jump'}])
