"""ROS 2 adapter using standard Joy, Twist, JointTrajectory, and String messages."""
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from .core import TeleopCore, merge_buttons
from .profile import load_config


class GamepadTeleopNode(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')
        self.declare_parameter('config_file', '')
        path = self.get_parameter('config_file').value
        if not path:
            raise ValueError('set the config_file ROS parameter to a YAML configuration')
        config, self.profile, poses, topics = load_config(path)
        self.core = TeleopCore(config, arm_poses=poses)
        self.received_at = None
        self.axes = ()
        self.latest_buttons = self.pending_buttons = ()
        self.cmd = self.create_publisher(Twist, topics['cmd_vel'], 10)
        self.head = self.create_publisher(JointTrajectory, topics['head'], 10)
        self.arm = self.create_publisher(JointTrajectory, topics['arm'], 10)
        self.gripper = self.create_publisher(JointTrajectory, topics['gripper'], 10)
        self.speech = self.create_publisher(String, topics['speech'], 10)
        self.subscription = self.create_subscription(Joy, topics['joy'], self.on_joy, 10)
        # A steady timer keeps the input watchdog running even if /clock pauses.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1/config.control_rate_hz, self.on_timer, clock=self.steady_clock)
        for warning in config.warnings:
            self.get_logger().warning(warning)
        self.get_logger().info(f'Gamepad teleop ready; output topics: {topics}')

    def on_joy(self, message):
        self.axes = tuple(message.axes)
        self.latest_buttons = tuple(message.buttons)
        # Preserve short stop/pose button taps, but do not latch deadman or unlock:
        # the base must always use the latest physical enable button state.
        self.pending_buttons = merge_buttons(self.pending_buttons, self.latest_buttons)
        self.received_at = time.monotonic()
        self.core.mark_joy_received(self.received_at)

    def on_timer(self):
        buttons = list(self.pending_buttons)
        for index in (self.core.config.deadman_button, self.core.config.turbo_button,
                      self.core.config.start_button):
            if 0 <= index < len(buttons):
                buttons[index] = self.latest_buttons[index] if index < len(self.latest_buttons) else 0
        now = time.monotonic()
        fresh = self.received_at is not None and now - self.received_at <= self.core.config.joy_timeout_sec
        stop_index = self.core.config.emergency_stop_button
        stop_pressed = 0 <= stop_index < len(buttons) and bool(buttons[stop_index])
        head_ready = (fresh and not self.core.locked and not stop_pressed
                      and bool(self.profile.head_joints) and self.head.get_subscription_count() > 0)
        # The standalone adapter suppresses head commands while stale or locked.
        output = self.core.update(self.axes, buttons, now, joy_received=False, head_ready=head_ready)
        self.pending_buttons = self.latest_buttons
        if output.publish_cmd_vel:
            message = Twist()
            message.linear.x = output.vx
            message.angular.z = output.wz
            self.cmd.publish(message)
        if output.head_changed and output.joy_fresh and not output.locked:
            self.publish_trajectory(self.head, self.profile.head_joints,
                                    self.profile.head_positions(output.pan, output.tilt), self.profile.head_duration_sec)
        try:
            if output.arm_pose and self.profile.arm_joints:
                pose = self.core.arm_poses[output.arm_pose]
                self.publish_trajectory(self.arm, self.profile.arm_joints,
                                        self.profile.pose_positions(pose), self.core.config.arm_duration_sec)
                if not output.gripper and self.profile.gripper_joint and self.profile.gripper_joint in pose:
                    self.publish_gripper(pose[self.profile.gripper_joint])
            if output.gripper and self.profile.gripper_joint:
                self.publish_gripper(self.core.config.gripper_open_m if output.gripper == 'open'
                                     else self.core.config.gripper_close_m)
        except (ValueError, KeyError) as error:
            self.get_logger().error(f'Joint command rejected: {error}')
        if output.speak:
            self.speech.publish(String(data=output.speak))
        for warning in output.warnings:
            self.get_logger().warning(warning)
        if output.lock_event:
            self.get_logger().warning(f'Software stop {output.lock_event}')

    def publish_gripper(self, value):
        self.publish_trajectory(self.gripper, [self.profile.gripper_joint],
                                [self.profile.bounded(self.profile.gripper_joint, value)],
                                self.core.config.gripper_duration_sec)

    def publish_trajectory(self, publisher, names, positions, seconds):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(seconds=seconds).to_msg()
        message.points = [point]
        publisher.publish(message)

    def stop(self):
        self.cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GamepadTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
