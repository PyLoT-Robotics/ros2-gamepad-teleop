"""MFT テレオペレーションの純粋ロジック。

このモジュールは rclpy / ROS メッセージに依存しない。``TeleopCore.update()`` に
``sensor_msgs/msg/Joy`` の ``axes`` / ``buttons`` と時刻を渡すと、台車速度・首の
目標値・ボタンの立ち上がりで発生したイベントを返す。ROS 側の都合（publish する
タイミングやメッセージの組み立て）は ``teleop_node.py`` に閉じ込める。

座標・符号の既定値は Xbox 系コントローラを Linux の ``joy`` で読んだ場合を
想定している。DualShock など別のコントローラでは config/mft_teleop.yaml の
ボタン番号・軸番号・反転を差し替える。
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Optional

try:  # PyYAML は ROS 環境にも入っているが、core 単体のテストでも動くようにする。
    import yaml
except Exception:  # pragma: no cover - yaml が無い環境向け
    yaml = None


# 機体の上限。max_linear にこれより大きい値を設定しても 0.4 に丸める。
HARD_MAX_LINEAR = 0.4

ARM_JOINT_NAMES: tuple[str, ...] = (
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
)
GRIPPER_JOINT_NAME = 'gripper_left_joint'

# 十字キー（hat）の方向。speech_phrases の並びと対応する。
HAT_UP = 0
HAT_DOWN = 1
HAT_LEFT = 2
HAT_RIGHT = 3
HAT_THRESHOLD = 0.5

DEFAULT_SPEECH_PHRASES: tuple[str, ...] = (
    'Hello!', 'Welcome!', 'How can I help?', 'Thank you!',
)


def default_arm_poses_file() -> str:
    """Optional pose file; no robot-specific path is searched."""
    return os.environ.get('GAMEPAD_ARM_POSES', '')


# 呼び名の揺れを吸収するための別名。
default_arm_poses_path = default_arm_poses_file


def load_arm_poses(path: Optional[str]) -> dict[str, dict[str, float]]:
    """arm_poses.yaml を読み込む。

    pylot_utils.robot.Robot._load_arm_poses と同じ形式
    ``{名前: {joint1..joint5: rad, gripper_left_joint: m}}`` を想定する。
    ファイルが無い・読めない・YAML が壊れている・根が mapping でない場合は
    例外を出さず空辞書を返す（ノードは落とさない）。値が数値でない項目は
    その項目だけ捨てる。
    """

    if not path or yaml is None or not os.path.isfile(path):
        return {}

    try:
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError):
        return {}

    if not isinstance(data, Mapping):
        return {}

    poses: dict[str, dict[str, float]] = {}
    for name, entry in data.items():
        if not isinstance(entry, Mapping):
            continue
        values: dict[str, float] = {}
        for key, value in entry.items():
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(number):
                values[str(key)] = number
        # 空でも名前自体は残す。ノード側で joint1..5 の不足を WARN にできる。
        poses[str(name)] = values
    return poses


def _finite_float(value: Any, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _int_value(value: Any, default: int) -> int:
    """ROS の dynamic_typing で int のはずの値が float/str でも int にする。"""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(number):
        return default
    return int(number)


def _bool_value(value: Any, default: bool) -> bool:
    """ROS の dynamic_typing で bool のはずの値の型揺れを吸収する。"""

    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ('true', '1', 'yes', 'on'):
            return True
        if lowered in ('false', '0', 'no', 'off', ''):
            return False
        return default
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(number):
        return default
    return number != 0.0


def _button_at(buttons: Optional[Sequence[Any]], index: int) -> bool:
    """Joy.buttons の 1 要素を安全に bool にする。短い配列・NaN は False。"""

    if buttons is None or index is None or index < 0:
        return False
    try:
        value = buttons[index]
    except (IndexError, KeyError, TypeError):
        return False
    try:
        number = float(value)
    except (TypeError, ValueError):
        return bool(value)
    if not math.isfinite(number):
        return False
    return number != 0.0


def merge_buttons(
    accumulated: Optional[Sequence[Any]],
    latest: Optional[Sequence[Any]],
) -> tuple[int, ...]:
    """前回のタイマー以降に届いたボタン状態を OR して溜める。

    ``_on_joy`` は 20 Hz のタイマーより速く呼ばれることがある。タイマーで
    最新値だけを見ると、50 ms 未満で押して離した B（緊急停止）や腕ポーズ、
    グリッパー、発話の短い押下を取りこぼす。ここで OR を取っておき、
    タイマーが 1 回処理したら ``latest``（現在の生の状態）に戻す。

    配列の長さが違う場合は長い方に合わせ、足りない要素は 0 とする。
    """

    old = _as_sequence(accumulated)
    new = _as_sequence(latest)
    length = max(len(old), len(new))
    return tuple(
        1 if _button_at(old, index) or _button_at(new, index) else 0
        for index in range(length)
    )


def _as_sequence(value: Any) -> tuple[Any, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, Sequence):
        return tuple(value)
    try:
        return tuple(value)
    except TypeError:
        return (value,)


@dataclass
class TeleopEvents:
    """1 周期のうちに発生した「1 回だけ」のイベント。"""

    arm_pose_name: Optional[str] = None
    gripper: Optional[str] = None  # 'open' / 'close'
    speech: Optional[str] = None
    head_home: bool = False
    lock_event: Optional[str] = None  # 'locked' / 'unlocked'
    warnings: list[str] = field(default_factory=list)

    @property
    def arm_pose(self) -> Optional[str]:
        return self.arm_pose_name

    @property
    def speak(self) -> Optional[str]:
        return self.speech

    @property
    def gripper_open(self) -> bool:
        return self.gripper == 'open'

    @property
    def gripper_close(self) -> bool:
        return self.gripper == 'close'

    @property
    def lock_changed(self) -> bool:
        return self.lock_event is not None

    @property
    def locked(self) -> bool:
        return self.lock_event == 'locked'

    @property
    def unlocked(self) -> bool:
        return self.lock_event == 'unlocked'


@dataclass
class TeleopOutput:
    """``TeleopCore.update()`` の結果。"""

    vx: float
    wz: float
    pan: int
    tilt: int
    head_changed: bool
    publish_cmd_vel: bool
    locked: bool
    active: bool
    joy_fresh: bool
    dt: float
    events: TeleopEvents

    # 仕様書の呼び方・実装で読みやすい呼び方の両方を用意する。
    @property
    def linear_x(self) -> float:
        return self.vx

    @property
    def angular_z(self) -> float:
        return self.wz

    @property
    def head_pan(self) -> int:
        return self.pan

    @property
    def head_tilt(self) -> int:
        return self.tilt

    @property
    def cmd_vel_publish(self) -> bool:
        return self.publish_cmd_vel

    @property
    def arm_pose(self) -> Optional[str]:
        return self.events.arm_pose_name

    @property
    def gripper(self) -> Optional[str]:
        return self.events.gripper

    @property
    def speak(self) -> Optional[str]:
        return self.events.speech

    @property
    def head_home(self) -> bool:
        return self.events.head_home

    @property
    def lock_event(self) -> Optional[str]:
        return self.events.lock_event

    @property
    def warnings(self) -> list[str]:
        return self.events.warnings


@dataclass
class TeleopConfig:
    """MFT テレオペレーションの設定。

    既定値は Xbox 系コントローラ（Linux の joy）を想定する。ROS の
    ``ros__parameters`` と同じ名前で、yaml から差し替えられる。
    """

    # --- ボタン（Xbox Linux: A=0, B=1, X=2, Y=3, LB=4, RB=5,
    #     Back=6, Start=7, Guide=8, LS=9, RS=10） ---
    deadman_button: int = 4          # LB
    turbo_button: int = 5            # RB
    emergency_stop_button: int = 1   # B
    start_button: int = 7            # Start
    head_home_button: int = 10       # 右スティック押し込み
    gripper_open_button: int = 2     # X
    gripper_close_button: int = 3    # Y
    # 2 本の配列で「ボタン番号 -> arm_poses.yaml のポーズ名」を持つ。
    arm_pose_button_ids: Sequence[int] = field(
        default_factory=lambda: (0, 6)
    )
    arm_pose_names: Sequence[str] = field(
        default_factory=lambda: ('hold', 'receive')
    )

    # --- 軸（Xbox Linux） ---
    axis_linear: int = 1
    axis_angular: int = 0
    axis_pan: int = 3
    axis_tilt: int = 4
    axis_hat_x: int = 6
    axis_hat_y: int = 7
    invert_linear: bool = False
    invert_angular: bool = False

    # --- 台車 ---
    hard_max_linear: float = HARD_MAX_LINEAR
    deadzone: float = 0.1
    max_linear: float = 0.25
    max_angular: float = 0.6
    turbo_max_linear: float = 0.4
    turbo_max_angular: float = 0.8
    linear_accel: float = 0.5        # m/s^2
    angular_accel: float = 1.5       # rad/s^2
    joy_timeout_sec: float = 0.5
    # デッドマンを離した / タイムアウト / ロックした後、0 の Twist を出す周期数。
    stop_publish_cycles: int = 5
    control_rate_hz: float = 20.0

    # --- 首（Dynamixel tick） ---
    head_pan_min: int = 0
    head_pan_max: int = 4095
    head_tilt_min: int = 800
    head_tilt_max: int = 1400
    head_pan_home: int = 1900
    head_tilt_home: int = 1200
    head_speed_ticks_per_sec: float = 600.0
    invert_pan: bool = False
    invert_tilt: bool = False

    # --- 腕・グリッパー ---
    arm_duration_sec: float = 2.0
    gripper_duration_sec: float = 1.0
    # URDF (manipulation/luna_arm/urdf/luna_arm.urdf.xacro) の
    # gripper_left_joint は prismatic 0.0〜0.025 m。robot.py の
    # set_gripper / arm_poses.yaml の open も 0.025 m なので全開は 0.025。
    gripper_open_m: float = 0.025
    gripper_close_m: float = 0.0
    arm_poses_file: str = ''

    # --- 発話 ---
    # axis_hat_y / axis_hat_x の順。上 / 下 / 左 / 右。
    speech_phrases: Sequence[str] = field(
        default_factory=lambda: DEFAULT_SPEECH_PHRASES
    )

    # 設定の読み込み時に出た警告。ROS 側で WARN として出す。
    warnings: list[str] = field(
        default_factory=list, init=False, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        # ROS の dynamic_typing では yaml の整数/浮動小数点/真偽値の型が
        # そのまま来る。ボタン・軸の添字が float のままだと配列参照で
        # TypeError になるので、ここで型をそろえる。
        self.deadman_button = _int_value(self.deadman_button, 4)
        self.turbo_button = _int_value(self.turbo_button, 5)
        self.emergency_stop_button = _int_value(
            self.emergency_stop_button, 1
        )
        self.start_button = _int_value(self.start_button, 7)
        self.head_home_button = _int_value(self.head_home_button, 10)
        self.gripper_open_button = _int_value(
            self.gripper_open_button, 2
        )
        self.gripper_close_button = _int_value(
            self.gripper_close_button, 3
        )
        self.axis_linear = _int_value(self.axis_linear, 1)
        self.axis_angular = _int_value(self.axis_angular, 0)
        self.axis_pan = _int_value(self.axis_pan, 3)
        self.axis_tilt = _int_value(self.axis_tilt, 4)
        self.axis_hat_x = _int_value(self.axis_hat_x, 6)
        self.axis_hat_y = _int_value(self.axis_hat_y, 7)
        self.invert_linear = _bool_value(self.invert_linear, False)
        self.invert_angular = _bool_value(self.invert_angular, False)
        self.invert_pan = _bool_value(self.invert_pan, False)
        self.invert_tilt = _bool_value(self.invert_tilt, False)

        self.hard_max_linear = max(0.0, _finite_float(self.hard_max_linear, HARD_MAX_LINEAR))
        # 機体の上限 0.4 m/s を超える max_linear は安全側に丸める。
        max_linear = _finite_float(self.max_linear, 0.25)
        if max_linear < 0.0:
            self.warnings.append(
                f'max_linear={max_linear} が負なので 0.0 にする'
            )
            max_linear = 0.0
        if max_linear > self.hard_max_linear:
            self.warnings.append(
                f'max_linear={max_linear} は機体上限 {self.hard_max_linear} m/s を'
                f'超えるため {self.hard_max_linear} に clamp する'
            )
            max_linear = self.hard_max_linear
        self.max_linear = max_linear

        turbo_linear = _finite_float(
            self.turbo_max_linear, self.hard_max_linear
        )
        if turbo_linear < 0.0:
            self.warnings.append(
                f'turbo_max_linear={turbo_linear} が負なので 0.0 にする'
            )
            turbo_linear = 0.0
        if turbo_linear > self.hard_max_linear:
            self.warnings.append(
                f'turbo_max_linear={turbo_linear} は機体上限 '
                f'{self.hard_max_linear} m/s を超えるため '
                f'{self.hard_max_linear} に clamp する'
            )
            turbo_linear = self.hard_max_linear
        self.turbo_max_linear = turbo_linear

        self.max_angular = max(0.0, _finite_float(self.max_angular, 0.6))
        self.turbo_max_angular = max(
            0.0, _finite_float(self.turbo_max_angular, 0.8)
        )
        self.linear_accel = max(
            0.0, _finite_float(self.linear_accel, 0.5)
        )
        self.angular_accel = max(
            0.0, _finite_float(self.angular_accel, 1.5)
        )
        self.joy_timeout_sec = max(
            0.0, _finite_float(self.joy_timeout_sec, 0.5)
        )

        deadzone = _finite_float(self.deadzone, 0.1)
        if deadzone < 0.0:
            self.warnings.append(f'deadzone={deadzone} が負なので 0.0 にする')
            deadzone = 0.0
        if deadzone >= 1.0:
            self.warnings.append(
                f'deadzone={deadzone} は 1 以上なので 0.99 に clamp する'
            )
            deadzone = 0.99
        self.deadzone = deadzone

        cycles = _int_value(self.stop_publish_cycles, 5)
        if cycles < 1:
            self.warnings.append(
                f'stop_publish_cycles={cycles} must be at least 1; clamped to 1'
            )
            cycles = 1
        self.stop_publish_cycles = cycles

        control_rate = _finite_float(self.control_rate_hz, 20.0)
        if control_rate <= 0.0:
            self.warnings.append(
                f'control_rate_hz={control_rate} は 0 以下なので 20.0 にする'
            )
            control_rate = 20.0
        self.control_rate_hz = control_rate

        # ボタン配列は長さを揃える。片方だけ長い場合は短い方に合わせる。
        button_ids = _as_sequence(self.arm_pose_button_ids)
        names = _as_sequence(self.arm_pose_names)
        converted_ids: list[int] = []
        for button_id in button_ids:
            try:
                converted_ids.append(int(button_id))
            except (TypeError, ValueError):
                self.warnings.append(
                    f'arm_pose_button_ids の {button_id!r} は整数でないので無視する'
                )
        converted_names = [str(name) for name in names]
        if len(converted_ids) != len(converted_names):
            self.warnings.append(
                'arm_pose_button_ids と arm_pose_names の長さが違うため、'
                '短い方に合わせる '
                f'({len(converted_ids)} != {len(converted_names)})'
            )
        count = min(len(converted_ids), len(converted_names))
        self.arm_pose_button_ids = tuple(converted_ids[:count])
        self.arm_pose_names = tuple(converted_names[:count])

        self.speech_phrases = tuple(
            str(phrase) for phrase in _as_sequence(self.speech_phrases)
        )

        # 首の範囲・ホームの整合性を取る。
        pan_min = _int_value(self.head_pan_min, 0)
        pan_max = _int_value(self.head_pan_max, 4095)
        tilt_min = _int_value(self.head_tilt_min, 800)
        tilt_max = _int_value(self.head_tilt_max, 1400)
        if pan_min > pan_max:
            self.warnings.append(
                f'head_pan_min > head_pan_max ({pan_min} > {pan_max}) '
                'なので入れ替える'
            )
            pan_min, pan_max = pan_max, pan_min
        if tilt_min > tilt_max:
            self.warnings.append(
                f'head_tilt_min > head_tilt_max ({tilt_min} > {tilt_max}) '
                'なので入れ替える'
            )
            tilt_min, tilt_max = tilt_max, tilt_min
        self.head_pan_min, self.head_pan_max = pan_min, pan_max
        self.head_tilt_min, self.head_tilt_max = tilt_min, tilt_max
        self.head_pan_home = min(
            pan_max,
            max(pan_min, _int_value(self.head_pan_home, 1900)),
        )
        self.head_tilt_home = min(
            tilt_max,
            max(tilt_min, _int_value(self.head_tilt_home, 1200)),
        )

        self.head_speed_ticks_per_sec = max(
            0.0, _finite_float(self.head_speed_ticks_per_sec, 600.0)
        )

        # 腕・グリッパーの時間と位置も型をそろえる。
        arm_duration = _finite_float(self.arm_duration_sec, 2.0)
        if arm_duration <= 0.0:
            self.warnings.append(
                f'arm_duration_sec={arm_duration} は 0 以下なので 2.0 にする'
            )
            arm_duration = 2.0
        self.arm_duration_sec = arm_duration

        gripper_duration = _finite_float(self.gripper_duration_sec, 1.0)
        if gripper_duration <= 0.0:
            self.warnings.append(
                f'gripper_duration_sec={gripper_duration} は 0 以下なので'
                ' 1.0 にする'
            )
            gripper_duration = 1.0
        self.gripper_duration_sec = gripper_duration
        self.gripper_open_m = _finite_float(self.gripper_open_m, 0.025)
        self.gripper_close_m = _finite_float(self.gripper_close_m, 0.0)

        if self.arm_poses_file is None:
            self.arm_poses_file = ''
        elif not isinstance(self.arm_poses_file, str):
            self.arm_poses_file = str(self.arm_poses_file)
        if not self.arm_poses_file:
            self.arm_poses_file = default_arm_poses_file()


# 短い別名。テストやノードから読みやすいように。
Config = TeleopConfig
Events = TeleopEvents
Output = TeleopOutput


class TeleopCore:
    """Joy 入力から台車・首・イベントを作る状態機械。"""

    def __init__(
        self,
        config: Optional[TeleopConfig] = None,
        arm_poses: Optional[Mapping[str, Mapping[str, float]]] = None,
    ) -> None:
        self.config = config or TeleopConfig()

        if arm_poses is None:
            self.arm_poses = load_arm_poses(self.config.arm_poses_file)
            if not self.arm_poses:
                if not os.path.isfile(self.config.arm_poses_file):
                    self.arm_poses_warning = (
                        'arm_poses file not found: '
                        f'{self.config.arm_poses_file}; '
                        'arm pose buttons will be ignored'
                    )
                else:
                    self.arm_poses_warning = (
                        'arm_poses file is empty or invalid: '
                        f'{self.config.arm_poses_file}; '
                        'arm pose buttons will be ignored'
                    )
            else:
                self.arm_poses_warning = None
        else:
            self.arm_poses = {
                str(name): dict(values)
                for name, values in arm_poses.items()
                if isinstance(values, Mapping)
            }
            self.arm_poses_warning = None

        # 首は起動時にホームを 1 回送る（_last_published_* = None）。
        self._pan = float(self.config.head_pan_home)
        self._tilt = float(self.config.head_tilt_home)
        self._last_published_pan: Optional[int] = None
        self._last_published_tilt: Optional[int] = None
        self._head_force = False

        self._vx = 0.0
        self._wz = 0.0
        self._was_active = False
        self._stop_cycles_left = 0
        self._locked = False

        self._last_update_time: Optional[float] = None
        self._last_joy_time: Optional[float] = None
        self._prev_buttons: tuple[Any, ...] = ()
        self._prev_hat_direction: Optional[int] = None

    # ------------------------------------------------------------------
    # 状態の読み出し（テスト・デバッグ用）
    # ------------------------------------------------------------------
    @property
    def locked(self) -> bool:
        return self._locked

    @property
    def pan(self) -> int:
        return self._clamp_pan(self._pan)

    @property
    def tilt(self) -> int:
        return self._clamp_tilt(self._tilt)

    @property
    def head_target(self) -> tuple[int, int]:
        return self.pan, self.tilt

    # ------------------------------------------------------------------
    # 入力
    # ------------------------------------------------------------------
    def mark_joy_received(self, now: float) -> None:
        """``/joy`` を 1 通受け取ったことを伝える。

        ノードは Joy コールバックでこれを呼び、20 Hz のタイマーでは
        ``update(..., joy_received=False)`` を呼ぶ。これで「最後に Joy が
        届いてから joy_timeout_sec 経ったか」を判定できる。
        """

        now_value = _finite_float(now, 0.0)
        self._last_joy_time = now_value

    def update(
        self,
        axes: Optional[Sequence[float]],
        buttons: Optional[Sequence[int]],
        now: float,
        *,
        joy_received: Optional[bool] = None,
        dt: Optional[float] = None,
        head_ready: bool = True,
    ) -> TeleopOutput:
        """1 周期分の状態を更新して出力を返す。

        Args:
            axes: Joy.axes。短い・None・NaN/inf でも例外にしない。
            buttons: Joy.buttons。短い・None でも例外にしない。
            now: 単調増加時刻 [s]。通常は ``time.monotonic()``。
            joy_received: True ならこの入力が新しい Joy。None の場合は
                axes/buttons のどちらかが空でなければ新しい Joy とみなす。
                タイマーから「新しい Joy は無い」と呼ぶときは False。
            dt: 制御周期 [s]。None なら now の差分（初回は制御周期）を使う。
            head_ready: False の間は首の入力・publish 判定を保留する。
                ノードが /set_position の購読者を確認するまで True にしない。
                起動時ホームは購読者が見えた最初の周期に 1 回だけ出す。

        Returns:
            TeleopOutput。イベントは 1 回だけ立ち上がりで入る。
        """

        now_value = _finite_float(now, 0.0)
        if dt is not None:
            dt_value = max(0.0, _finite_float(dt, 0.0))
        elif self._last_update_time is None:
            dt_value = 1.0 / self.config.control_rate_hz
        else:
            dt_value = max(0.0, now_value - self._last_update_time)

        dt_value = min(dt_value, 1.0 / self.config.control_rate_hz)

        if joy_received is None:
            # 空の axes/buttons は「Joy が来ていない」とみなす。実際の Joy は
            # スティックが中央でも axes 配列を持つので、これでタイムアウトを
            # テストしやすい。
            joy_received = bool(axes) or bool(buttons)
        if joy_received:
            self._last_joy_time = now_value

        joy_fresh = (
            self._last_joy_time is not None
            and (now_value - self._last_joy_time)
            <= self.config.joy_timeout_sec + 1e-9
        )

        axes_seq: tuple[Any, ...] = _as_sequence(axes)
        buttons_seq: tuple[Any, ...] = _as_sequence(buttons)

        events = TeleopEvents()

        if joy_fresh:
            self._process_fresh_input(
                axes_seq, buttons_seq, dt_value, events, head_ready
            )
        else:
            # 古いボタン状態を残さない。次に Joy が来たときは押しっぱなしでも
            # 新しい押下として扱う（安全側でなく「操作したら動く」を優先）。
            self._prev_buttons = ()
            self._prev_hat_direction = None

        vx, wz, active = self._update_base(
            axes_seq, buttons_seq, dt_value, joy_fresh
        )

        pan = self._clamp_pan(self._pan)
        tilt = self._clamp_tilt(self._tilt)
        if head_ready:
            head_changed = (
                self._head_force
                or self._last_published_pan is None
                or self._last_published_tilt is None
                or pan != self._last_published_pan
                or tilt != self._last_published_tilt
            )
            if head_changed:
                self._last_published_pan = pan
                self._last_published_tilt = tilt
                self._head_force = False
        else:
            # 購読者が見えるまでは「送った」ことにしない。見えた最初の
            # 周期に起動時ホームを改めて送る。
            head_changed = False

        # 台車の publish を止めるタイミング。デッドマンを離した・
        # タイムアウト・ロックのときは 0 を stop_publish_cycles 周期出して
        # から publish をやめる。停止自体は即 0（加速度制限をかけない）。
        if active:
            self._stop_cycles_left = 0
            publish_cmd_vel = True
        else:
            if self._was_active:
                self._stop_cycles_left = self.config.stop_publish_cycles
            # 緊急停止は「元々デッドマンを押していなかった」場合でも
            # 他ノードの cmd_vel を止めるため 0 を出す。
            if events.lock_event == 'locked':
                self._stop_cycles_left = max(
                    self._stop_cycles_left,
                    self.config.stop_publish_cycles,
                )
            if self._stop_cycles_left > 0:
                publish_cmd_vel = True
                self._stop_cycles_left -= 1
            else:
                publish_cmd_vel = False
        self._was_active = active

        self._last_update_time = now_value

        return TeleopOutput(
            vx=0.0 if vx == 0.0 else vx,
            wz=0.0 if wz == 0.0 else wz,
            pan=pan,
            tilt=tilt,
            head_changed=head_changed,
            publish_cmd_vel=publish_cmd_vel,
            locked=self._locked,
            active=active,
            joy_fresh=joy_fresh,
            dt=dt_value,
            events=events,
        )

    def update_from_joy(
        self,
        joy: Any,
        now: float,
        *,
        dt: Optional[float] = None,
        head_ready: bool = True,
    ) -> TeleopOutput:
        """``sensor_msgs/msg/Joy`` 風オブジェクトから 1 周期分を更新する。

        rclpy には依存しない。``joy.axes`` / ``joy.buttons`` を読むだけ。
        """

        return self.update(
            getattr(joy, 'axes', None),
            getattr(joy, 'buttons', None),
            now,
            joy_received=True,
            dt=dt,
            head_ready=head_ready,
        )

    # ------------------------------------------------------------------
    # 個別処理
    # ------------------------------------------------------------------
    def _process_fresh_input(
        self,
        axes: Sequence[Any],
        buttons: Sequence[Any],
        dt: float,
        events: TeleopEvents,
        head_ready: bool,
    ) -> None:
        # 解除 -> 停止 の順に見る。B と Start が同時なら停止を優先する。
        if self._rising(buttons, self.config.start_button):
            if self._locked:
                self._locked = False
                events.lock_event = 'unlocked'

        if self._rising(buttons, self.config.emergency_stop_button):
            if not self._locked:
                self._locked = True
                events.lock_event = 'locked'

        # 腕・グリッパー・発話はロック中は無視する。ただしボタンの
        # 押下状態は最後に更新するので、ロック解除後に押しっぱなしでも
        # 勝手にイベントは出ない。
        if not self._locked:
            for button_id, pose_name in zip(
                self.config.arm_pose_button_ids,
                self.config.arm_pose_names,
            ):
                if self._rising(buttons, button_id):
                    if pose_name in self.arm_poses:
                        events.arm_pose_name = pose_name
                    else:
                        events.warnings.append(
                            f'unknown arm pose {pose_name!r} '
                            f'(button {button_id}); ignored. '
                            f'available: {sorted(self.arm_poses)}'
                        )
                    break

            if self._rising(buttons, self.config.gripper_open_button):
                events.gripper = 'open'
            elif self._rising(buttons, self.config.gripper_close_button):
                events.gripper = 'close'

        # 発話はロック中も方向の立ち上がりだけ追跡し、イベントは出さない。
        hat_direction = self._hat_direction(axes)
        if (
            not self._locked
            and hat_direction is not None
            and hat_direction != self._prev_hat_direction
        ):
            if hat_direction < len(self.config.speech_phrases):
                events.speech = self.config.speech_phrases[hat_direction]
            else:
                events.warnings.append(
                    f'speech_phrases が足りないため hat 方向 '
                    f'{hat_direction} の発話を無視する'
                )
        self._prev_hat_direction = hat_direction

        # 首はデッドマン不要。ロック中も動かせる。ただし /set_position の
        # 購読者が見えるまでは、起動時ホームを送れるように入力を保留する。
        if head_ready:
            startup_home_pending = (
                self._last_published_pan is None
                or self._last_published_tilt is None
            )
            home_pressed = self._rising(
                buttons, self.config.head_home_button
            )
            if home_pressed or startup_home_pending:
                # 起動直後・ホームボタンの周期は、スティック入力より
                # ホームを優先する。次の周期から通常の首操作に戻る。
                self._pan = float(self.config.head_pan_home)
                self._tilt = float(self.config.head_tilt_home)
                if home_pressed:
                    self._head_force = True
                    events.head_home = True
            else:
                pan_axis = self._axis(axes, self.config.axis_pan)
                tilt_axis = self._axis(axes, self.config.axis_tilt)
                if self.config.invert_pan:
                    pan_axis = -pan_axis
                if self.config.invert_tilt:
                    tilt_axis = -tilt_axis

                speed = self.config.head_speed_ticks_per_sec
                if pan_axis:
                    self._pan += pan_axis * speed * dt
                if tilt_axis:
                    self._tilt += tilt_axis * speed * dt
            # ここでは整数に丸めない。ゆっくり倒したときに 1 tick 未満の
            # 変化が毎周期捨てられないよう、float のまま保持する。
            self._pan = min(
                float(self.config.head_pan_max),
                max(float(self.config.head_pan_min), self._pan),
            )
            self._tilt = min(
                float(self.config.head_tilt_max),
                max(float(self.config.head_tilt_min), self._tilt),
            )

        self._prev_buttons = tuple(buttons)

    def _update_base(
        self,
        axes: Sequence[Any],
        buttons: Sequence[Any],
        dt: float,
        joy_fresh: bool,
    ) -> tuple[float, float, bool]:
        deadman = self._pressed(buttons, self.config.deadman_button)
        turbo = self._pressed(buttons, self.config.turbo_button)
        active = bool(joy_fresh and deadman and not self._locked)

        if not active:
            # デッドマンを離した / タイムアウト / ロックは加速度制限を
            # かけずに即 0。
            self._vx = 0.0
            self._wz = 0.0
            return 0.0, 0.0, False

        linear_axis = self._axis(axes, self.config.axis_linear)
        angular_axis = self._axis(axes, self.config.axis_angular)
        if self.config.invert_linear:
            linear_axis = -linear_axis
        if self.config.invert_angular:
            angular_axis = -angular_axis

        linear_axis = self._apply_deadzone(linear_axis)
        angular_axis = self._apply_deadzone(angular_axis)

        max_linear = (
            self.config.turbo_max_linear
            if turbo
            else self.config.max_linear
        )
        max_angular = (
            self.config.turbo_max_angular
            if turbo
            else self.config.max_angular
        )
        target_vx = linear_axis * max_linear
        target_wz = angular_axis * max_angular

        self._vx = self._approach(
            self._vx, target_vx, self.config.linear_accel * dt
        )
        self._wz = self._approach(
            self._wz, target_wz, self.config.angular_accel * dt
        )
        return self._vx, self._wz, True

    # ------------------------------------------------------------------
    # 小物
    # ------------------------------------------------------------------
    def _axis(self, axes: Sequence[Any], index: int) -> float:
        if index is None or index < 0:
            return 0.0
        try:
            value = axes[index]
        except (IndexError, KeyError, TypeError):
            return 0.0
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0
        return number if math.isfinite(number) else 0.0

    def _pressed(self, buttons: Sequence[Any], index: int) -> bool:
        return _button_at(buttons, index)

    def _rising(self, buttons: Sequence[Any], index: int) -> bool:
        return self._pressed(buttons, index) and not self._pressed(
            self._prev_buttons, index
        )

    def _hat_direction(self, axes: Sequence[Any]) -> Optional[int]:
        x = self._axis(axes, self.config.axis_hat_x)
        y = self._axis(axes, self.config.axis_hat_y)
        # 斜め入力は絶対値の大きい方へ寄せる。
        if abs(y) >= abs(x):
            if y >= HAT_THRESHOLD:
                return HAT_UP
            if y <= -HAT_THRESHOLD:
                return HAT_DOWN
        if abs(x) >= abs(y):
            if x >= HAT_THRESHOLD:
                return HAT_LEFT
            if x <= -HAT_THRESHOLD:
                return HAT_RIGHT
        return None

    def _apply_deadzone(self, value: float) -> float:
        deadzone = self.config.deadzone
        magnitude = abs(value)
        if magnitude <= deadzone:
            return 0.0
        # デッドゾーン直後を 0 から滑らかにつなぐ。
        scaled = (magnitude - deadzone) / (1.0 - deadzone)
        scaled = min(1.0, scaled)
        return math.copysign(scaled, value)

    @staticmethod
    def _approach(current: float, target: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return current
        difference = target - current
        if abs(difference) <= max_delta:
            return target
        return current + math.copysign(max_delta, difference)

    def _clamp_pan(self, value: float) -> int:
        return int(
            min(
                self.config.head_pan_max,
                max(self.config.head_pan_min, int(round(value))),
            )
        )

    def _clamp_tilt(self, value: float) -> int:
        return int(
            min(
                self.config.head_tilt_max,
                max(self.config.head_tilt_min, int(round(value))),
            )
        )


# 短い別名。
Core = TeleopCore

__all__ = [
    'ARM_JOINT_NAMES',
    'GRIPPER_JOINT_NAME',
    'HARD_MAX_LINEAR',
    'DEFAULT_SPEECH_PHRASES',
    'Config',
    'Core',
    'Events',
    'Output',
    'TeleopConfig',
    'TeleopCore',
    'TeleopEvents',
    'TeleopOutput',
    'default_arm_poses_file',
    'default_arm_poses_path',
    'load_arm_poses',
    'merge_buttons',
]
