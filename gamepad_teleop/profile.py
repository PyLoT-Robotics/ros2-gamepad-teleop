"""Joint configuration independent of ROS and of the original robot."""
from dataclasses import dataclass, field, fields
from pathlib import Path
import math
import yaml
from .core import TeleopConfig


def finite(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass
class RobotProfile:
    arm_joints: tuple = ()
    gripper_joint: str = ''
    head_joints: tuple = ()
    head_scale: tuple = (0.001, 0.001)
    head_offset: tuple = (0.0, 0.0)
    joint_limits: dict = field(default_factory=dict)
    head_duration_sec: float = 0.1

    def __post_init__(self):
        for name in ('arm_joints', 'head_joints'):
            raw = getattr(self, name)
            if isinstance(raw, str):
                raise ValueError(f"{name} must be a sequence of names")
            values = tuple(raw)
            if any(not isinstance(j, str) or not j for j in values) or len(set(values)) != len(values):
                raise ValueError(f"{name} must contain unique nonempty names")
            setattr(self, name, values)
        if self.head_joints and len(self.head_joints) != 2:
            raise ValueError('head_joints must be empty or [pan_joint, tilt_joint]')
        if not isinstance(self.gripper_joint, str):
            raise ValueError('gripper_joint must be a string')
        all_joints = self.arm_joints + self.head_joints + ((self.gripper_joint,) if self.gripper_joint else ())
        if len(set(all_joints)) != len(all_joints):
            raise ValueError('joint groups must not overlap')
        for name in ('head_scale', 'head_offset'):
            values = tuple(finite(v, name) for v in getattr(self, name))
            if len(values) != 2:
                raise ValueError(f'{name} requires two values')
            setattr(self, name, values)
        if not isinstance(self.joint_limits, dict):
            raise ValueError('joint_limits must be a mapping')
        limits = {}
        for joint, pair in self.joint_limits.items():
            if len(pair) != 2:
                raise ValueError(f'{joint}: limits require [lower, upper]')
            lower, upper = (finite(v, joint) for v in pair)
            if lower > upper:
                raise ValueError(f'{joint}: reversed limits')
            limits[joint] = (lower, upper)
        self.joint_limits = limits
        for joint in all_joints:
            if joint not in limits:
                raise ValueError(f'missing joint_limits for {joint}')
        self.head_duration_sec = finite(self.head_duration_sec, 'head_duration_sec')
        if self.head_duration_sec <= 0:
            raise ValueError('head_duration_sec must be positive')

    def bounded(self, joint, value):
        value = finite(value, joint)
        lower, upper = self.joint_limits[joint]
        if not lower <= value <= upper:
            raise ValueError(f'{joint}: {value} outside [{lower}, {upper}]')
        return value

    def pose_positions(self, pose):
        return [self.bounded(joint, pose[joint]) for joint in self.arm_joints]

    def head_positions(self, pan, tilt):
        return [self.bounded(joint, value * scale + offset)
                for joint, value, scale, offset in
                zip(self.head_joints, (pan, tilt), self.head_scale, self.head_offset)]


def load_config(path):
    """Read a strict YAML config, resolving pose files relative to that config."""
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or set(data) - {'teleop', 'robot', 'topics'}:
        raise ValueError('configuration needs teleop, robot, and/or topics mappings')
    values = data.get('teleop', {})
    if not isinstance(values, dict):
        raise ValueError('teleop must be a mapping')
    allowed = {f.name for f in fields(TeleopConfig) if f.init}
    if set(values) - allowed:
        raise ValueError(f'unknown teleop options: {set(values) - allowed}')
    config = TeleopConfig(**values)
    if config.arm_poses_file:
        config.arm_poses_file = str((Path(path).resolve().parent / config.arm_poses_file).resolve())
    robot = data.get('robot', {})
    if not isinstance(robot, dict):
        raise ValueError('robot must be a mapping')
    profile = RobotProfile(**robot)
    poses = {}
    if config.arm_poses_file:
        with Path(config.arm_poses_file).open(encoding='utf-8') as stream:
            raw_poses = yaml.safe_load(stream)
        if not isinstance(raw_poses, dict):
            raise ValueError('pose file must contain a mapping')
        for name, values in raw_poses.items():
            if not isinstance(name, str) or not name or not isinstance(values, dict):
                raise ValueError('poses need a nonempty name and joint mapping')
            poses[name] = {joint: finite(value, joint) for joint, value in values.items()}

    for pose in poses.values():
        profile.pose_positions(pose)
        if profile.gripper_joint and profile.gripper_joint in pose:
            profile.bounded(profile.gripper_joint, pose[profile.gripper_joint])
    if profile.head_joints:
        for pan in (config.head_pan_min, config.head_pan_max):
            for tilt in (config.head_tilt_min, config.head_tilt_max):
                profile.head_positions(pan, tilt)
    if profile.gripper_joint:
        profile.bounded(profile.gripper_joint, config.gripper_open_m)
        profile.bounded(profile.gripper_joint, config.gripper_close_m)
    topics = {'joy': '/joy', 'cmd_vel': '/gamepad/cmd_vel', 'head': '/gamepad/head_trajectory',
              'arm': '/gamepad/arm_trajectory', 'gripper': '/gamepad/gripper_trajectory',
              'speech': '/gamepad/speech'}
    overrides = data.get('topics', {})
    if not isinstance(overrides, dict) or set(overrides) - set(topics):
        raise ValueError('unknown topic configuration')
    topics.update(overrides)
    if any(not isinstance(t, str) or not t.strip() for t in topics.values()):
        raise ValueError('topic names must be nonempty strings')
    return config, profile, poses, topics
