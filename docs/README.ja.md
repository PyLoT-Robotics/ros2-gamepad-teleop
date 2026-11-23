# ROS 2 Gamepad Teleop

[English](../README.md) | 日本語

ゲームパッド1台で台車・首・腕・グリッパー・定型発話要求を操作するROS 2パッケージです。展示用ロボットの状態機械をベースに、機体固有の関節名・速度上限・独自メッセージへの依存を外しました。ROS非依存のPython APIと、標準メッセージを使うROSアダプタで構成しています。

## ROSなしでインストール・設定確認

Python 3.10以上とPyYAMLを使用します。リポジトリ直下で実行してください。

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e '.[test]'
python -m gamepad_teleop.cli config/gamepad.yaml
python -m pytest -q
```

`rclpy`がなければROS統合テストだけスキップします。[設定例](../config/gamepad.yaml)は架空の機体です。実機に合わせて関節名・可動域・ポーズ・接続先を変更してください。

## ROS 2での起動

ROS Jazzy / Ubuntu 24.04を対象にしています。ワークスペースの`src`内に配置し、ROS標準メッセージ、`joy`、`launch_ros`、PyYAMLを用意します。依存一覧は[package.xml](../package.xml)にあります。

```bash
source /opt/ros/jazzy/setup.bash
# ROSワークスペース直下で実行
colcon build --packages-select gamepad_teleop --symlink-install
source install/setup.bash
ros2 launch gamepad_teleop gamepad.launch.py
# 設定ファイルの差し替え
ros2 launch gamepad_teleop gamepad.launch.py config_file:=/absolute/path/to/gamepad.yaml
```

launchは20 Hzの`joy_node`も起動します。既存のJoy入力を使う場合は、次のようにノードだけ起動できます。

```bash
ros2 run gamepad_teleop gamepad_teleop_node --ros-args -p config_file:=/absolute/path/to/gamepad.yaml
```

## 操作とメッセージ

既定はLinuxのXbox系配列です。LBで台車のデッドマン、RBでターボ、左スティックで前後・旋回、Bで停止ロック、Startで解除します。右スティックで首、押し込みでホーム、A/Backで`hold`/`receive`、X/Yでグリッパー開閉、十字キーで4種類の定型発話要求を送ります。首・腕・グリッパー・発話要求にはデッドマンを要求しません。

入力は`/joy`の`Joy`。既定出力は`/gamepad/cmd_vel`の`Twist`、`/gamepad/head_trajectory`・`arm_trajectory`・`gripper_trajectory`の`JointTrajectory`、`/gamepad/speech`の`String`です。実機には自動接続しません。発話は別のTTS購読ノードに渡す文字列で、TTSエンジンやアクションクライアントは含みません。

## 機体設定

- `teleop`：ボタン番号、軸、反転、速度、加速度、タイムアウト、定型文。全項目は[TeleopConfig](../gamepad_teleop/core.py)を参照。負のボタン・軸番号でその割り当てを無効にできます。
- `robot`：腕の関節順、グリッパー関節、首の`[pan, tilt]`関節名、全関節の上下限。無効にする関節群は`[]`、グリッパーは空文字列にします。
- `topics`：入力・出力先。実機への接続前に設定してください。

首は仮想整数tickを`tick * head_scale + head_offset`でradに変換します。例は0.001 rad/tickです。首の可動域の両端も関節制限内である必要があります。起動時ホームは新しいJoy入力と購読者が揃うまで保留し、入力断・停止ロック中は首の新規指令を出しません。

腕のポーズファイルは設定ファイルからの相対パスで解決します。[poses.yaml](../config/poses.yaml)に例があります。明示指定したファイルの欠落・不正な値・関節制限外の値はエラーになります。空なら読み込みません。既定パスは任意の`GAMEPAD_ARM_POSES`で指定でき、元のPyLoTワークスペースは探索しません。腕ポーズとグリッパーボタンが同時ならボタンを優先します。

既定は通常0.25 m/s・0.6 rad/s、ターボ0.4 m/s・0.8 rad/s、入力タイムアウト0.5秒、20 Hzです。`hard_max_linear`で機体の直進上限を指定でき、通常・ターボの設定値を制限します。周期処理が遅れた場合の積分時間は1周期分に制限します。

## 停止と検証範囲

デッドマン解放・入力断・停止ロック時は次の制御周期で加速度制限を介さずゼロ速度を出します。既定では5周期送ってから無送信になります。入力監視は単調時刻とsteady timerを使うため、シミュレーション時刻の停止に依存しません。短い停止・ポーズボタン入力は保持しますが、デッドマンとターボは最新の物理状態を使います。

停止ロックは首・腕・グリッパー・発話の新規指令を抑止します。受信側が開始済みの軌道や発話はキャンセルしません。ハードウェア非常停止の代わりではなく、受信側のウォッチドッグや他ノードとの指令仲裁も必要です。

単体テストに加え、ROS利用時は独立ドメインの実DDSで合成Joyを流し、停止・入力断・関節名・首・腕・グリッパー・発話要求を確認します。実機や実ゲームパッドでの性能検証は含みません。

## ライセンス

Apache-2.0。[LICENSE](../LICENSE)、[出典と変更範囲](../PROVENANCE.md)を参照してください。モデル・録音・専用ドライバは含みません。
