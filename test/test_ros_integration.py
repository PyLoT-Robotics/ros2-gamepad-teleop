"""Real ROS messages and DDS on an isolated test domain; no robot connected."""
import time
import types
from pathlib import Path
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import gamepad_teleop.node as adapter


def test_dds_base_watchdog_short_taps_joint_profiles_and_speech(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '187')
    now = [100.0]
    monkeypatch.setattr(adapter, 'time', types.SimpleNamespace(monotonic=lambda: now[0]))
    config = Path(__file__).parents[1] / 'config/gamepad.yaml'
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config}'])
    node = observer = None
    executor = SingleThreadedExecutor()
    try:
        node = adapter.GamepadTeleopNode()
        node.timer.cancel()
        observer = Node('gamepad_test_observer')
        received = {key: [] for key in ['cmd_vel', 'head', 'arm', 'gripper', 'speech']}
        subscriptions = []
        for key, msg_type, topic in [
            ('cmd_vel', Twist, '/gamepad/cmd_vel'),
            ('head', JointTrajectory, '/gamepad/head_trajectory'),
            ('arm', JointTrajectory, '/gamepad/arm_trajectory'),
            ('gripper', JointTrajectory, '/gamepad/gripper_trajectory'),
            ('speech', String, '/gamepad/speech')]:
            subscriptions.append(observer.create_subscription(msg_type, topic,
                lambda msg, key=key: received[key].append(msg), 10))
        publisher = observer.create_publisher(Joy, '/joy', 10)
        executor.add_node(node)
        executor.add_node(observer)

        def wait(predicate):
            deadline = time.monotonic() + 3
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert predicate(), 'DDS condition timed out'

        def send(pressed=(), axes=None):
            now[0] += .05
            message = Joy()
            message.axes = axes or [0., 1., 0., 0., 0., 0., 0., 0.]
            message.buttons = [int(i in pressed) for i in range(11)]
            publisher.publish(message)
            wait(lambda: node.received_at == now[0])

        wait(lambda: publisher.get_subscription_count() > 0 and
             node.head.get_subscription_count() > 0 and node.speech.get_subscription_count() > 0)
        node.on_timer()
        assert not received['head']  # Discovery alone never homes the head.
        send([4])
        node.on_timer()
        wait(lambda: received['cmd_vel'] and received['head'])
        assert 0 < received['cmd_vel'][-1].linear.x <= .25
        assert received['head'][-1].joint_names == ['pan_joint', 'tilt_joint']
        assert list(received['head'][-1].points[0].positions) == [0., 0.]

        # Both press and release arrive before the timer: never latch deadman.
        send([4])
        send([])
        node.on_timer()
        count = len(received['cmd_vel'])
        wait(lambda: len(received['cmd_vel']) > count)
        assert received['cmd_vel'][-1].linear.x == 0

        send([0, 2], axes=[0., 0., 0., 0., 0., 0., 0., 1.])
        node.on_timer()
        wait(lambda: received['arm'] and received['gripper'] and received['speech'] and len(received['cmd_vel']) >= 3)
        assert received['arm'][-1].joint_names == ['shoulder_joint', 'elbow_joint']
        assert list(received['arm'][-1].points[0].positions) == [0., .3]
        assert received['gripper'][-1].points[0].positions[0] == .025
        assert received['speech'][-1].data == 'Hello!'

        send([])
        node.on_timer()
        wait(lambda: len(received['cmd_vel']) >= 4)
        send([4])
        node.on_timer()
        wait(lambda: len(received['cmd_vel']) >= 5)
        assert received['cmd_vel'][-1].linear.x > 0
        now[0] += 1  # Input timeout is monotonic, independent of /clock.
        count = len(received['cmd_vel'])
        node.on_timer()
        wait(lambda: len(received['cmd_vel']) > count)
        assert received['cmd_vel'][-1].linear.x == 0

        # A stop tap released within a timer interval is still latched.
        send([1])
        send([])
        node.on_timer()
        assert node.core.locked
        wait(lambda: len(received['cmd_vel']) >= 7)
        send([0, 2, 4], axes=[0., 1., 0., 1., 1., 0., 0., 1.])
        counts = {key: len(values) for key, values in received.items()}
        node.on_timer()
        wait(lambda: len(received['cmd_vel']) >= 8)
        for _ in range(8):
            executor.spin_once(timeout_sec=.01)
        for key in ('head', 'arm', 'gripper', 'speech'):
            assert len(received[key]) == counts[key]
        send([7])
        node.on_timer()
        assert not node.core.locked
        wait(lambda: len(received['cmd_vel']) >= 9)
        count = len(received['cmd_vel'])
        node.stop()
        wait(lambda: len(received['cmd_vel']) > count)
        assert received['cmd_vel'][-1].linear.x == 0
    finally:
        executor.shutdown()
        if node:
            node.destroy_node()
        if observer:
            observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_steady_timer_and_input_watchdog_work_with_paused_ros_time(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '188')
    config = tmp_path / 'base-only.yaml'
    config.write_text('teleop: {joy_timeout_sec: 0.15}\n')
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config}', '-p', 'use_sim_time:=true'])
    node = observer = None
    executor = SingleThreadedExecutor()
    try:
        node = adapter.GamepadTeleopNode()
        observer = Node('paused_clock_test')
        received = []
        subscription = observer.create_subscription(Twist, '/gamepad/cmd_vel', received.append, 10)
        publisher = observer.create_publisher(Joy, '/joy', 10)
        executor.add_node(node)
        executor.add_node(observer)

        def wait(predicate):
            end = time.monotonic() + 3
            while not predicate() and time.monotonic() < end:
                executor.spin_once(timeout_sec=.005)
            assert predicate()

        wait(lambda: publisher.get_subscription_count() > 0 and node.cmd.get_subscription_count() > 0)
        assert node.get_clock().now().nanoseconds == 0
        message = Joy()
        message.axes = [0., 1.]
        message.buttons = [0, 0, 0, 0, 1]
        publisher.publish(message)
        wait(lambda: any(m.linear.x > 0 for m in received))
        wait(lambda: received[-1].linear.x == 0)
        assert node.get_clock().now().nanoseconds == 0
    finally:
        executor.shutdown()
        if node:
            node.destroy_node()
        if observer:
            observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
