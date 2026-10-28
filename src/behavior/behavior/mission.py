"""Greenhouse mission as a list of stages run by one state machine. Pure Python, no ROS, so it can be unit tested.

The route is read from config/mission.yaml in Gazebo world coordinates and converted to the odom
frame (origin = spawn pose), so a different spawn pose does not shift the targets.

Stage types:
    follow_line  drive along a line (an aisle), steering back to it; fails if it drifts too far
    drive_to     head straight for a point
    turn_to      turn in place to a world yaw
    wait         stand still for some seconds (e.g. to scan checkpoints)
Every stage can 'watch' color detector labels; the first sighting of each is reported.

Safety: while /safety/stop is True, driving stages hold still and turning stages keep turning
(the safety filter only blocks forward motion). The mission fails if the stop lasts longer than
safety_timeout.
"""

from dataclasses import dataclass, field, fields
import math

from behavior.navigation import DriveParams, drive_to, follow_line, TurnGains, turn_rate, world_to_odom, wrap

RUNNING = 'RUNNING'
DONE = 'DONE'
FAILED = 'FAILED'

STOPPED = (0.0, 0.0)


@dataclass
class Stage:
    name: str
    timeout: float = 240.0
    watch: tuple = ()
    drive: DriveParams = field(default_factory=DriveParams)
    turn: TurnGains = field(default_factory=TurnGains)
    drives_forward = True

    def step(self, pose, heading, elapsed):
        """-> (command, status, message)"""
        raise NotImplementedError


@dataclass
class FollowLine(Stage):
    start: tuple = (0.0, 0.0)
    end: tuple = (0.0, 0.0)
    max_lateral: float = 0.12

    def step(self, pose, heading, elapsed):
        cmd, remaining, lateral = follow_line(pose, heading, self.start, self.end,
                                              params=self.drive, gains=self.turn)
        if abs(lateral) > self.max_lateral:
            return STOPPED, FAILED, f'lateral offset {lateral:.3f} m > {self.max_lateral:.2f} m'
        if remaining <= self.drive.goal_tolerance:
            return STOPPED, DONE, f'lateral offset {lateral:.3f} m'
        return cmd, RUNNING, f'remaining {remaining:.2f} m, lateral {lateral:.3f} m'


@dataclass
class DriveTo(Stage):
    target: tuple = (0.0, 0.0)

    def step(self, pose, heading, elapsed):
        cmd, remaining = drive_to(pose, heading, self.target, self.drive, self.turn)
        if remaining <= self.drive.goal_tolerance:
            return STOPPED, DONE, f'position error {remaining:.3f} m'
        return cmd, RUNNING, f'remaining {remaining:.2f} m'


@dataclass
class TurnTo(Stage):
    yaw: float = 0.0
    drives_forward = False

    def step(self, pose, heading, elapsed):
        error = wrap(self.yaw - heading)
        if abs(error) <= self.turn.deadband:
            return STOPPED, DONE, f'heading error {math.degrees(error):.1f} deg'
        return (0.0, turn_rate(error, self.turn)), RUNNING, f'remaining {math.degrees(error):.1f} deg'


@dataclass
class Wait(Stage):
    duration: float = 5.0
    drives_forward = False

    def step(self, pose, heading, elapsed):
        if elapsed >= self.duration:
            return STOPPED, DONE, ''
        return STOPPED, RUNNING, f'{self.duration - elapsed:.1f} s left'


def _params(cls, spec):
    """Dataclass from a YAML dict; keys ending in _deg are converted to radians."""
    spec = dict(spec or {})
    for key in [k for k in spec if k.endswith('_deg')]:
        spec[key[:-4]] = math.radians(spec.pop(key))
    unknown = set(spec) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f'unknown {cls.__name__} keys: {sorted(unknown)}')
    return cls(**spec)


def build_stages(specs, spawn=(0.0, 0.0, 0.0)):
    """Stage objects from the YAML stage list; world coordinates -> odom (spawn = world pose of odom origin)."""
    stages = []
    for i, spec in enumerate(specs):
        spec = dict(spec)
        kind = spec.pop('type')
        common = {
            'name': spec.pop('name', f'{i + 1}_{kind}'),
            'timeout': float(spec.pop('timeout', 240.0)),
            'watch': tuple(spec.pop('watch', ())),
            'drive': _params(DriveParams, spec.pop('drive', None)),
            'turn': _params(TurnGains, spec.pop('turn', None)),
        }
        if kind == 'follow_line':
            stage = FollowLine(**common, start=world_to_odom(*spec.pop('from'), spawn),
                               end=world_to_odom(*spec.pop('to'), spawn),
                               max_lateral=float(spec.pop('max_lateral', 0.12)))
        elif kind == 'drive_to':
            stage = DriveTo(**common, target=world_to_odom(*spec.pop('to'), spawn))
        elif kind == 'turn_to':
            stage = TurnTo(**common, yaw=wrap(math.radians(spec.pop('yaw_deg')) - spawn[2]))
        elif kind == 'wait':
            stage = Wait(**common, duration=float(spec.pop('duration')))
        else:
            raise ValueError(f'unknown stage type {kind!r}')
        if spec:
            raise ValueError(f'stage {common["name"]}: unknown keys {sorted(spec)}')
        stages.append(stage)
    return stages


@dataclass
class StartCheck:
    xy_tolerance: float = 0.15
    yaw_tolerance: float = math.radians(10)

    def ok(self, pose):
        """The mission assumes a fresh odom: the robot must be at the odom origin."""
        return (math.hypot(pose[0], pose[1]) <= self.xy_tolerance
                and abs(wrap(pose[2])) <= self.yaw_tolerance)


class Mission:
    """Runs the stages in order. Call step() at a fixed rate; read status / events after each call."""

    def __init__(self, stages, safety_timeout=30.0):
        self.stages = stages
        self.safety_timeout = safety_timeout
        self.index = 0
        self.status = RUNNING
        self.progress = ''
        self.seen = {}              # stage name -> labels seen during that stage
        self._events = []
        self._stage_start = None
        self._stop_since = None

    @property
    def stage(self):
        return self.stages[self.index] if self.index < len(self.stages) else None

    def events(self):
        """Messages since the last call (stage started / finished, detections, failure reason)."""
        out, self._events = self._events, []
        return out

    def fail(self, reason):
        self.status = FAILED
        self._events.append(f'MISSION FAILED: {reason}')

    def on_detection(self, label):
        stage = self.stage
        # Only once the stage has started, so a sighting is never reported before its START line
        if self.status != RUNNING or self._stage_start is None or label not in stage.watch:
            return
        seen = self.seen.setdefault(stage.name, set())
        if label not in seen:
            seen.add(label)
            self._events.append(f'{stage.name}: {label} detected')

    def step(self, pose, heading, safety_stop, now):
        """-> (linear, angular) command for this tick."""
        if self.status != RUNNING:
            return STOPPED
        stage = self.stage
        if self._stage_start is None:
            self._stage_start = now
            self._events.append(f'STAGE {self.index + 1}/{len(self.stages)} START: {stage.name}')
        elapsed = now - self._stage_start

        if elapsed > stage.timeout:
            self.fail(f'{stage.name} timed out after {stage.timeout:.0f} s')
            return STOPPED

        if safety_stop:
            if self._stop_since is None:
                self._stop_since = now
                self._events.append(f'{stage.name}: safety stop, waiting')
            if now - self._stop_since > self.safety_timeout:
                self.fail(f'safety stop longer than {self.safety_timeout:.0f} s')
                return STOPPED
            if stage.drives_forward:
                return STOPPED
        elif self._stop_since is not None:
            self._stop_since = None
            self._events.append(f'{stage.name}: safety clear, resuming')

        cmd, status, message = stage.step(pose, heading, elapsed)
        self.progress = f'{stage.name}: {message}'
        if status == FAILED:
            self.fail(f'{stage.name}: {message}')
            return STOPPED
        if status == DONE:
            seen = sorted(self.seen.get(stage.name, ()))
            missed = sorted(set(stage.watch) - set(seen))
            report = f' | seen: {seen}' + (f', not seen: {missed}' if missed else '') if stage.watch else ''
            detail = f' ({message})' if message else ''
            self._events.append(f'STAGE {self.index + 1}/{len(self.stages)} DONE: {stage.name}{detail}{report}')
            self.index += 1
            self._stage_start = None
            if self.index == len(self.stages):
                self.status = DONE
                self._events.append('MISSION COMPLETE')
            return STOPPED
        return cmd


def load_mission(config, spawn=(0.0, 0.0, 0.0)):
    """(StartCheck, stages) from the parsed mission.yaml."""
    return _params(StartCheck, config.get('start_check')), build_stages(config['stages'], spawn)
