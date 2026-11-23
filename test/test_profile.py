import pytest
from gamepad_teleop.core import TeleopConfig
from gamepad_teleop.profile import RobotProfile


def test_machine_speed_limit_is_configurable():
    config = TeleopConfig(hard_max_linear=0.8, max_linear=1, turbo_max_linear=2)
    assert config.max_linear == 0.8
    assert config.turbo_max_linear == 0.8


def test_profile_rejects_out_of_range_named_pose():
    profile = RobotProfile(arm_joints=('shoulder',), joint_limits={'shoulder': (-1, 1)})
    with pytest.raises(ValueError):
        profile.pose_positions({'shoulder': 1.1})


def test_default_does_not_search_robot_workspace(monkeypatch):
    from gamepad_teleop.core import default_arm_poses_file
    monkeypatch.delenv('GAMEPAD_ARM_POSES', raising=False)
    monkeypatch.setenv('PYLOT_WS', '/nonexistent')
    assert default_arm_poses_file() == ''


def test_example_loads_relative_pose_file():
    from pathlib import Path
    from gamepad_teleop.profile import load_config
    _, robot, poses, topics = load_config(Path(__file__).parents[1] / 'config/gamepad.yaml')
    assert robot.pose_positions(poses['hold']) == [0, 0.3]
    assert robot.head_positions(1000, -200) == [1, -0.2]
    assert topics['cmd_vel'] == '/gamepad/cmd_vel'


@pytest.mark.parametrize('text', ['teleop: {typo: 2}', 'robot: {arm_joints: [x]}',
                                 'topics: {bad: /x}', 'robot: null', '[]',
                                 'teleop: {arm_poses_file: missing.yaml}'])
def test_bad_configs_are_rejected(tmp_path, text):
    from gamepad_teleop.profile import load_config
    path = tmp_path / 'config.yaml'
    path.write_text(text)
    with pytest.raises((ValueError, FileNotFoundError)):
        load_config(path)


def test_nonfinite_pose_rejected(tmp_path):
    from gamepad_teleop.profile import load_config
    (tmp_path / 'poses.yaml').write_text('home: {x: .nan}')
    path = tmp_path / 'config.yaml'
    path.write_text('teleop: {arm_poses_file: poses.yaml}')
    with pytest.raises(ValueError):
        load_config(path)


def test_head_conversion_must_respect_declared_joint_limits(tmp_path):
    from gamepad_teleop.profile import load_config
    path = tmp_path / 'config.yaml'
    path.write_text('robot: {head_joints: [pan, tilt], joint_limits: {pan: [-1, 1], tilt: [-1, 1]}}')
    with pytest.raises(ValueError):
        load_config(path)


def test_stop_always_publishes_at_least_one_zero():
    assert TeleopConfig(stop_publish_cycles=0).stop_publish_cycles == 1
