"""mft_teleop/core.py の pytest。ROS (rclpy) は使わない。"""

from __future__ import annotations

import math

import pytest

from gamepad_teleop.core import (
    DEFAULT_SPEECH_PHRASES,
    HARD_MAX_LINEAR,
    TeleopConfig,
    TeleopCore,
    default_arm_poses_file,
    load_arm_poses,
    merge_buttons,
)


# Xbox Linux のボタン番号。buttons 配列は「押している=1」のリスト。
DEADMAN = 4
TURBO = 5
ESTOP = 1
START = 7
HEAD_HOME = 10
GRIPPER_OPEN = 2
GRIPPER_CLOSE = 3
ARM_POSE_A = 0


def buttons(*pressed: int) -> list[int]:
    """押下ボタン番号から Joy.buttons 相当の配列を作る。"""

    result = [0] * 11
    for index in pressed:
        result[index] = 1
    return result


def axes(
    linear: float = 0.0,
    angular: float = 0.0,
    pan: float = 0.0,
    tilt: float = 0.0,
    hat_x: float = 0.0,
    hat_y: float = 0.0,
) -> list[float]:
    """Xbox Linux の軸配列（axis 0..7）を作る。"""

    return [angular, linear, 0.0, pan, tilt, 0.0, hat_x, hat_y]


class Driver:
    """20 Hz で core.update を呼ぶテスト用ドライバ。"""

    def __init__(self, config: TeleopConfig | None = None, **kwargs):
        self.core = TeleopCore(config or TeleopConfig(), **kwargs)
        self.time = 0.0
        self.period = 1.0 / self.core.config.control_rate_hz

    def step(
        self,
        joy_axes: list[float] | None = None,
        joy_buttons: list[int] | None = None,
        joy_received: bool = True,
        dt: float | None = None,
    ):
        self.time += self.period if dt is None else dt
        return self.core.update(
            joy_axes if joy_axes is not None else [],
            joy_buttons if joy_buttons is not None else [],
            self.time,
            joy_received=joy_received,
        )

    def hold(self, joy_axes, joy_buttons, cycles: int = 40):
        output = None
        for _ in range(cycles):
            output = self.step(joy_axes, joy_buttons)
        return output


# ----------------------------------------------------------------------
# 台車
# ----------------------------------------------------------------------
def test_no_deadman_never_moves():
    driver = Driver()
    output = driver.step(axes(linear=1.0, angular=1.0), buttons())
    assert output.vx == 0.0
    assert output.wz == 0.0
    assert output.publish_cmd_vel is False


def test_deadman_low_speed_limits():
    driver = Driver()
    output = driver.hold(
        axes(linear=1.0, angular=1.0), buttons(DEADMAN)
    )
    assert output.vx == pytest.approx(0.25, abs=1e-9)
    assert output.wz == pytest.approx(0.6, abs=1e-9)
    assert output.active is True
    assert output.publish_cmd_vel is True


def test_turbo_limits():
    driver = Driver()
    output = driver.hold(
        axes(linear=1.0, angular=1.0), buttons(DEADMAN, TURBO)
    )
    assert output.vx == pytest.approx(0.4, abs=1e-9)
    assert output.wz == pytest.approx(0.8, abs=1e-9)


def test_turbo_requires_deadman():
    driver = Driver()
    output = driver.hold(
        axes(linear=1.0, angular=1.0), buttons(TURBO)
    )
    assert output.vx == 0.0
    assert output.publish_cmd_vel is False


def test_max_linear_is_clamped_to_hard_limit():
    config = TeleopConfig(max_linear=0.9, turbo_max_linear=0.9)
    assert config.max_linear == HARD_MAX_LINEAR
    assert config.turbo_max_linear == HARD_MAX_LINEAR
    assert any('clamp' in warning for warning in config.warnings)

    driver = Driver(config)
    output = driver.hold(
        axes(linear=1.0, angular=0.0), buttons(DEADMAN, TURBO)
    )
    assert output.vx == pytest.approx(0.4, abs=1e-9)


def test_deadzone_ignores_small_input_and_scales_large_input():
    driver = Driver()
    stopped = driver.hold(axes(linear=0.05), buttons(DEADMAN), cycles=10)
    assert stopped.vx == 0.0

    moving = driver.hold(axes(linear=0.2), buttons(DEADMAN), cycles=20)
    expected = 0.25 * (0.2 - 0.1) / 0.9
    assert moving.vx == pytest.approx(expected, abs=1e-6)


def test_acceleration_limit_and_immediate_stop():
    driver = Driver()
    first = driver.step(axes(linear=1.0), buttons(DEADMAN))
    # 初回 dt = 0.05 s、linear_accel = 0.5 m/s^2。
    assert first.vx == pytest.approx(0.025, abs=1e-9)

    second = driver.step(axes(linear=1.0), buttons(DEADMAN))
    assert second.vx == pytest.approx(0.05, abs=1e-9)

    # デッドマンを離した瞬間は加速度制限をかけず 0。
    stopped = driver.step(axes(linear=1.0), buttons())
    assert stopped.vx == 0.0
    assert stopped.wz == 0.0


def test_zero_twist_is_published_for_several_cycles_then_stops():
    driver = Driver(TeleopConfig(stop_publish_cycles=5))
    driver.hold(axes(linear=1.0), buttons(DEADMAN), cycles=20)

    flags = []
    for _ in range(7):
        output = driver.step(axes(linear=1.0), buttons())
        assert output.vx == 0.0
        flags.append(output.publish_cmd_vel)

    assert flags == [True, True, True, True, True, False, False]


def test_joy_timeout_stops_immediately_and_stops_publishing():
    core = TeleopCore(TeleopConfig(stop_publish_cycles=5), arm_poses={})
    core.mark_joy_received(0.0)
    active = core.update(
        axes(linear=1.0), buttons(DEADMAN), 0.0, joy_received=False
    )
    assert active.active is True

    # joy_timeout_sec = 0.5。0.5 s ちょうどはまだ有効。
    still_fresh = core.update(
        axes(linear=1.0), buttons(DEADMAN), 0.5, joy_received=False
    )
    assert still_fresh.joy_fresh is True
    assert still_fresh.vx > 0.0

    timed_out = core.update(
        axes(linear=1.0), buttons(DEADMAN), 0.51, joy_received=False
    )
    assert timed_out.joy_fresh is False
    assert timed_out.vx == 0.0
    assert timed_out.publish_cmd_vel is True

    flags = [timed_out.publish_cmd_vel]
    for index in range(1, 6):
        output = core.update(
            axes(linear=1.0),
            buttons(DEADMAN),
            0.51 + index * 0.05,
            joy_received=False,
        )
        flags.append(output.publish_cmd_vel)
    assert flags == [True, True, True, True, True, False]


# ----------------------------------------------------------------------
# ロック
# ----------------------------------------------------------------------
def test_merge_buttons_ors_every_joy_since_last_timer():
    # タイマー周期の間に押して離した B も、OR した値なら残る。
    pending = merge_buttons((), buttons(ESTOP))
    pending = merge_buttons(pending, buttons())
    assert pending[ESTOP] == 1

    # 押しっぱなしは残り、別のボタンは足される。
    assert merge_buttons(buttons(DEADMAN), buttons(GRIPPER_OPEN)) == tuple(
        buttons(DEADMAN, GRIPPER_OPEN)
    )
    # 長さが違っても短い方を 0 として扱う。
    assert merge_buttons((1,), (0, 1, 0)) == (1, 1, 0)
    assert merge_buttons(None, None) == ()


def test_short_estop_press_in_one_cycle_locks():
    core = TeleopCore(TeleopConfig(), arm_poses={})
    core.update(axes(), buttons(), 0.0)

    # 20 Hz のタイマーの間に B を押して離した、という状況。
    pending = merge_buttons((), buttons(ESTOP))
    pending = merge_buttons(pending, buttons())
    output = core.update(axes(linear=1.0), pending, 0.05)

    assert output.locked is True
    assert output.events.lock_event == 'locked'
    assert output.vx == 0.0


def test_emergency_stop_locks_and_start_unlocks():
    driver = Driver()
    locked = driver.step(axes(linear=1.0), buttons(DEADMAN, ESTOP))
    assert locked.locked is True
    assert locked.vx == 0.0
    assert locked.events.lock_event == 'locked'
    assert locked.publish_cmd_vel is True

    # ロック中はデッドマンを押しても動かない。
    still_locked = driver.step(axes(linear=1.0), buttons(DEADMAN))
    assert still_locked.locked is True
    assert still_locked.vx == 0.0

    unlocked = driver.step(axes(linear=1.0), buttons(START))
    assert unlocked.locked is False
    assert unlocked.events.lock_event == 'unlocked'


def test_short_gripper_press_in_one_cycle_is_not_lost():
    core = TeleopCore(TeleopConfig(), arm_poses={})
    core.update(axes(), buttons(), 0.0)
    pending = merge_buttons((), buttons(GRIPPER_OPEN))
    pending = merge_buttons(pending, buttons())
    output = core.update(axes(), pending, 0.05)
    assert output.events.gripper == 'open'


def test_lock_ignores_arm_gripper_and_speech_but_allows_head():
    config = TeleopConfig()
    pose = {name: 0.0 for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')}
    core = TeleopCore(config, arm_poses={'hold': pose})
    core.update([], [], 0.0)

    # B でロック。同時に A / X / 十字キー上を押しても無視。
    locked = core.update(
        axes(hat_y=1.0),
        buttons(ESTOP, ARM_POSE_A, GRIPPER_OPEN),
        0.05,
    )
    assert locked.locked is True
    assert locked.events.arm_pose_name is None
    assert locked.events.gripper is None
    assert locked.events.speech is None

    # ロック中も首は動かせる。A は押しっぱなしのまま。
    moved = core.update(axes(pan=1.0), buttons(ARM_POSE_A), 0.10)
    assert moved.locked is True
    assert moved.pan > config.head_pan_home

    # ロック中でもホームは効く。
    homed = core.update(axes(), buttons(HEAD_HOME, ARM_POSE_A), 0.15)
    assert homed.events.head_home is True
    assert homed.pan == config.head_pan_home
    assert homed.tilt == config.head_tilt_home

    # 解除後、押しっぱなしだった A は立ち上がり扱いにならない。
    unlocked = core.update(axes(), buttons(START, ARM_POSE_A), 0.20)
    assert unlocked.events.lock_event == 'unlocked'
    assert unlocked.events.arm_pose_name is None

    # 一度離して押し直せばポーズが出る。
    core.update(axes(), buttons(), 0.25)
    pressed = core.update(axes(), buttons(ARM_POSE_A), 0.30)
    assert pressed.events.arm_pose_name == 'hold'


# ----------------------------------------------------------------------
# 入力の異常系
# ----------------------------------------------------------------------
def test_nan_and_inf_axes_are_treated_as_zero():
    driver = Driver()
    output = driver.hold(
        [math.nan, math.inf, -math.inf, math.nan, math.inf, 0.0, math.nan, 0.0],
        buttons(DEADMAN),
        cycles=10,
    )
    assert output.vx == 0.0
    assert output.wz == 0.0
    assert math.isfinite(output.pan)
    assert math.isfinite(output.tilt)


def test_short_axes_and_buttons_do_not_raise():
    driver = Driver()
    output = driver.step([1.0], [1])
    assert output.vx == 0.0
    # 首も軸が足りないのでホームのまま。
    assert output.pan == 1900
    assert output.tilt == 1200

    # ボタンが空でも落ちない。
    empty = driver.step([], [])
    assert empty.vx == 0.0


# ----------------------------------------------------------------------
# 首
# ----------------------------------------------------------------------
def test_head_moves_without_deadman_and_clamps():
    config = TeleopConfig()
    driver = Driver(config)
    output = driver.hold(axes(pan=1.0, tilt=-1.0), buttons(), cycles=100)
    assert output.pan == config.head_pan_max == 4095
    assert output.tilt == config.head_tilt_min == 800


def test_head_invert_and_home_button():
    config = TeleopConfig(invert_pan=True, invert_tilt=True)
    driver = Driver(config)
    output = driver.hold(axes(pan=-1.0, tilt=1.0), buttons(), cycles=100)
    assert output.pan == config.head_pan_max
    assert output.tilt == config.head_tilt_min

    homed = driver.step(axes(), buttons(HEAD_HOME))
    assert homed.events.head_home is True
    assert homed.head_changed is True
    assert homed.pan == 1900
    assert homed.tilt == 1200


def test_head_commands_wait_until_subscriber_is_ready():
    core = TeleopCore(TeleopConfig(), arm_poses={})
    hidden = core.update(
        axes(pan=1.0),
        buttons(),
        0.0,
        head_ready=False,
    )
    assert hidden.head_changed is False
    assert (hidden.pan, hidden.tilt) == (1900, 1200)

    ready = core.update(
        axes(pan=1.0),
        buttons(),
        0.05,
        head_ready=True,
    )
    # 購読者が見えた最初の周期は、スティックより起動時ホームを優先。
    assert ready.head_changed is True
    assert (ready.pan, ready.tilt) == (1900, 1200)

    moved = core.update(
        axes(pan=1.0),
        buttons(),
        0.10,
        head_ready=True,
    )
    assert moved.pan > 1900


def test_head_home_is_sent_once_on_startup_then_only_on_change():
    driver = Driver()
    first = driver.step(axes(), buttons())
    assert first.head_changed is True
    assert (first.pan, first.tilt) == (1900, 1200)

    second = driver.step(axes(), buttons())
    assert second.head_changed is False

    # 丸めても tick が変わらない微小入力では publish しない。
    tiny = driver.step(axes(pan=0.001), buttons())
    assert tiny.head_changed is False


def test_head_slow_input_accumulates_instead_of_being_rounded_away():
    driver = Driver()
    driver.step(axes(), buttons())
    changed = False
    for _ in range(10):
        output = driver.step(axes(pan=0.01), buttons())
        changed = changed or output.head_changed
    # 0.01 * 600 * 0.05 = 0.3 tick/周期。10 周期で 3 tick 進む。
    assert output.pan == 1903
    assert changed is True


def test_head_home_button_publishes_even_when_already_home():
    driver = Driver()
    driver.step(axes(), buttons())
    homed = driver.step(axes(), buttons(HEAD_HOME))
    assert homed.head_changed is True
    assert homed.events.head_home is True


# ----------------------------------------------------------------------
# ボタンの立ち上がり
# ----------------------------------------------------------------------
def test_buttons_fire_once_per_press():
    pose = {name: 0.0 for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')}
    core = TeleopCore(TeleopConfig(), arm_poses={'hold': pose})
    core.update([], [], 0.0)

    first = core.update([], buttons(ARM_POSE_A), 0.05)
    assert first.events.arm_pose_name == 'hold'
    held = core.update([], buttons(ARM_POSE_A), 0.10)
    assert held.events.arm_pose_name is None

    core.update([], buttons(), 0.15)
    again = core.update([], buttons(ARM_POSE_A), 0.20)
    assert again.events.arm_pose_name == 'hold'

    opened = core.update([], buttons(GRIPPER_OPEN), 0.25)
    assert opened.events.gripper == 'open'
    assert core.update([], buttons(GRIPPER_OPEN), 0.30).events.gripper is None

    core.update([], buttons(), 0.35)
    closed = core.update([], buttons(GRIPPER_CLOSE), 0.40)
    assert closed.events.gripper == 'close'


def test_unknown_arm_pose_is_ignored_with_warning():
    core = TeleopCore(
        TeleopConfig(arm_pose_button_ids=(0,), arm_pose_names=('missing',)),
        arm_poses={'hold': {'joint1': 0.0}},
    )
    core.update([], [], 0.0)
    output = core.update([], buttons(ARM_POSE_A), 0.05)
    assert output.events.arm_pose_name is None
    assert any('unknown arm pose' in warning for warning in output.warnings)


# ----------------------------------------------------------------------
# 発話
# ----------------------------------------------------------------------
def test_hat_triggers_one_phrase_per_press():
    driver = Driver()
    driver.step(axes(), buttons())

    up = driver.step(axes(hat_y=1.0), buttons())
    assert up.events.speech == DEFAULT_SPEECH_PHRASES[0]
    assert driver.step(axes(hat_y=1.0), buttons()).events.speech is None

    driver.step(axes(), buttons())
    down = driver.step(axes(hat_y=-1.0), buttons())
    assert down.events.speech == DEFAULT_SPEECH_PHRASES[1]

    driver.step(axes(), buttons())
    left = driver.step(axes(hat_x=1.0), buttons())
    assert left.events.speech == DEFAULT_SPEECH_PHRASES[2]

    driver.step(axes(), buttons())
    right = driver.step(axes(hat_x=-1.0), buttons())
    assert right.events.speech == DEFAULT_SPEECH_PHRASES[3]


def test_hat_deadzone_does_not_trigger():
    driver = Driver()
    driver.step(axes(), buttons())
    output = driver.step(axes(hat_y=0.3), buttons())
    assert output.events.speech is None


# ----------------------------------------------------------------------
# arm_poses.yaml
# ----------------------------------------------------------------------
def test_load_arm_poses_from_tmp_yaml(tmp_path):
    path = tmp_path / 'arm_poses.yaml'
    path.write_text(
        'hold:\n'
        '  joint1: 0.1\n'
        '  joint2: 0.2\n'
        '  joint3: 0.3\n'
        '  joint4: 0.4\n'
        '  joint5: 0.5\n'
        '  gripper_left_joint: 0.025\n'
        'broken: not-a-mapping\n'
        'bad_value:\n'
        '  joint1: .nan\n'
        '  joint2: 1.0\n',
        encoding='utf-8',
    )
    poses = load_arm_poses(str(path))
    assert poses['hold']['joint1'] == pytest.approx(0.1)
    assert poses['hold']['gripper_left_joint'] == pytest.approx(0.025)
    assert 'broken' not in poses
    assert 'bad_value' in poses
    assert 'joint1' not in poses['bad_value']  # NaN は捨てる
    assert poses['bad_value']['joint2'] == pytest.approx(1.0)


def test_load_arm_poses_missing_or_invalid_file(tmp_path):
    assert load_arm_poses(str(tmp_path / 'missing.yaml')) == {}

    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text('- not\n- a\n- mapping\n', encoding='utf-8')
    assert load_arm_poses(str(invalid)) == {}

    broken = tmp_path / 'broken.yaml'
    broken.write_text('foo: [unclosed\n', encoding='utf-8')
    assert load_arm_poses(str(broken)) == {}

    assert load_arm_poses(None) == {}


def test_default_arm_poses_file_uses_explicit_environment(monkeypatch, tmp_path):
    monkeypatch.setenv('GAMEPAD_ARM_POSES', str(tmp_path / 'poses.yaml'))
    assert default_arm_poses_file() == str(
        tmp_path / 'poses.yaml'
    )


def test_config_keeps_arm_pose_arrays_aligned():
    config = TeleopConfig(
        arm_pose_button_ids=(0, 6, 8),
        arm_pose_names=('hold', 'receive'),
    )
    assert config.arm_pose_button_ids == (0, 6)
    assert config.arm_pose_names == ('hold', 'receive')
    assert any('長さが違う' in warning for warning in config.warnings)


def test_config_coerces_dynamic_parameter_types():
    config = TeleopConfig(
        max_linear=1,                 # yaml で整数
        max_angular=1,
        deadman_button=4.0,           # yaml で float
        axis_linear=1.0,
        invert_pan=1,
        invert_tilt='false',
        arm_duration_sec=2,
        gripper_open_m=1,
        head_pan_min=0.0,
        stop_publish_cycles=5.0,
    )
    # 0.4 を超える max_linear は clamp される。
    assert config.max_linear == HARD_MAX_LINEAR
    assert config.max_angular == pytest.approx(1.0)
    assert config.deadman_button == 4
    assert config.axis_linear == 1
    assert config.invert_pan is True
    assert config.invert_tilt is False
    assert config.arm_duration_sec == pytest.approx(2.0)
    assert config.gripper_open_m == pytest.approx(1.0)
    assert config.head_pan_min == 0
    assert config.stop_publish_cycles == 5
