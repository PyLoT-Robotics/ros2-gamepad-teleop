# ROS 2 Gamepad Teleop

English | [日本語](docs/README.ja.md)

Control a mobile base, pan/tilt head, arm, gripper, and canned speech requests from one gamepad. The ROS-independent state machine is adapted from PyLoT's exhibition robot; the ROS adapter uses **standard messages and configurable joint names/limits**, without the original robot's drivers or interfaces.

## Features

- Deadman-controlled base velocity, turbo, scaled deadzone, acceleration limits, and a configurable hard linear-speed ceiling.
- Monotonic input watchdog and steady-clock timer, including when simulated ROS time is paused.
- Latched software stop and explicit unlock. Short stop/pose taps are retained between timer ticks; deadman/turbo use the latest physical state.
- Incremental head movement, named arm poses, gripper open/close, and four configurable speech phrases.
- Joint-limit validation, relative pose-file paths, and isolated output topics by default.
- Pure Python API and configuration checker work without ROS. ROS integration targets Jazzy / Ubuntu 24.04.

## Install and check a profile without ROS

Python 3.10+ and PyYAML are required. From the repository root:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e '.[test]'
python -m gamepad_teleop.cli config/gamepad.yaml
python -m pytest -q
```

The ROS integration test is skipped when `rclpy` is absent. No controller or robot is needed for the other tests. The sample profile describes a fictitious robot; replace its names, limits, poses, and output topics before connecting hardware.

## Build and run with ROS 2

Place this repository inside a ROS workspace's `src` directory. Install `rclpy`, the standard message packages, `joy`, `launch_ros`, and PyYAML (the dependencies are listed in [package.xml](package.xml)). Use ROS's system Python.

```bash
source /opt/ros/jazzy/setup.bash
# From the ROS workspace root:
colcon build --packages-select gamepad_teleop --symlink-install
source install/setup.bash
ros2 launch gamepad_teleop gamepad.launch.py
# Use a different configuration:
ros2 launch gamepad_teleop gamepad.launch.py config_file:=/absolute/path/to/gamepad.yaml
```

The launch file starts `joy_node` with a 20 Hz autorepeat rate and the teleop node. To use an existing Joy source, start only the node:

```bash
ros2 run gamepad_teleop gamepad_teleop_node --ros-args -p config_file:=/absolute/path/to/gamepad.yaml
```

## Messages and defaults

| Role | Message | Default topic |
|---|---|---|
| Gamepad input | `sensor_msgs/Joy` | `/joy` |
| Base output | `geometry_msgs/Twist` | `/gamepad/cmd_vel` |
| Head output | `trajectory_msgs/JointTrajectory` | `/gamepad/head_trajectory` |
| Arm output | `trajectory_msgs/JointTrajectory` | `/gamepad/arm_trajectory` |
| Gripper output | `trajectory_msgs/JointTrajectory` | `/gamepad/gripper_trajectory` |
| Speech request | `std_msgs/String` | `/gamepad/speech` |

Speech output is text for a separate TTS subscriber, not an embedded TTS engine or action client. The node does not cancel previously queued speech or trajectories. Connect trajectory topics to appropriate controllers and route base commands through your command arbitration layer.

## Default controls (Xbox-style Linux Joy)

| Input | Function |
|---|---|
| LB / RB | Base deadman / turbo (turbo requires deadman) |
| Left stick | Forward and turn |
| B / Start | Latch software stop / unlock |
| Right stick / right-stick click | Pan/tilt / head home |
| A / Back | Named poses `hold` / `receive` |
| X / Y | Gripper open / close |
| D-pad up/down/left/right | Four speech phrases |

Indices, axes, inversions, phrases, timings, speeds, and head ranges are fields under `teleop` in [config/gamepad.yaml](config/gamepad.yaml). See `TeleopConfig` in [core.py](gamepad_teleop/core.py) for all options. A negative button/axis index disables that binding. Controllers such as DualSense use different mappings; inspect `/joy` before changing the profile. The software stop does not require a deadman; head and one-shot arm/gripper/speech controls also do not require it.

## Robot profiles

`robot.arm_joints` sets ordered arm names; `robot.gripper_joint` selects the prismatic gripper joint. Set them to `[]` / `""` to disable those outputs. `robot.head_joints` is `[]` or `[pan_joint, tilt_joint]`.

Every configured joint requires `robot.joint_limits: {name: [lower, upper]}` in controller units (rad for revolute joints, m for a prismatic gripper). Invalid/out-of-range poses are rejected. An explicit gripper button overrides the gripper value inside a simultaneous named pose.

Head core positions use integer virtual ticks; they are converted to radians by `position = tick * head_scale + head_offset`, one value per pan/tilt joint. The example uses 0.001 rad/tick and zero offset. All converted range endpoints must fit the joint limits. Startup head home is held until fresh Joy input and a head subscriber are present. No head commands are emitted while input is stale or the software stop is locked.

`teleop.arm_poses_file` resolves relative to the configuration file, as shown in [config/poses.yaml](config/poses.yaml). Empty means no pose file. An optional `GAMEPAD_ARM_POSES` environment variable sets the default; original PyLoT workspace paths are never searched. Missing/malformed explicitly configured pose files fail configuration validation.

Defaults are 0.25 m/s linear, 0.6 rad/s angular, 0.4 m/s turbo, 0.8 rad/s turbo, 0.5 s input timeout, and 20 Hz control. `hard_max_linear` is configurable; normal and turbo linear limits are clamped to it. Timer integration is capped at one nominal period so a stalled callback does not produce a large head/velocity step.

## Stop behavior and verification

Releasing deadman, losing Joy input, or latching stop makes the base output zero immediately on the next control tick, bypassing acceleration limits. The default is five zero messages followed by silence; configure the receiving base controller with its own watchdog. The software stop suppresses new head/arm/gripper/speech commands and does not cancel an already accepted controller trajectory. It is a software control state, not a hardware emergency-stop system. Configure a command mux when another node also controls the same actuators.

Tests cover state transitions, input timeouts, invalid values, short button taps, configuration and pose limits. The optional ROS test uses real DDS on a separate domain with synthetic Joy input and standard output messages; it exercises stop, timeout, head/arm/gripper, and speech behavior. No physical robot or gamepad performance is claimed.

## License and origin

Apache-2.0. See [LICENSE](LICENSE) and [PROVENANCE.md](PROVENANCE.md). No models, recordings, robot drivers, or vendor SDKs are bundled.
