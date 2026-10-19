"""RViz pose topics through the real GUI worker, using simulated ROS endpoints."""
import json
import math
import time
from types import SimpleNamespace

from PyQt5 import QtWidgets
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from match_mur_navigation import navigation_gui
from match_mur_navigation.session import LATCHED


def test_rviz_pose_topics_route_to_simulated_native_and_nav2_endpoints(monkeypatch, tmp_path):
    # Explicitly isolate this test even when the shell inherited robot DDS peers.
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    for key in ('FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_DISCOVERY_SERVER'):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setattr(navigation_gui, 'GUI_LOG_DIR', str(tmp_path/'gui'))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    context = rclpy.context.Context()
    rclpy.init(args=[], context=context, domain_id=179)
    node = rclpy.create_node('click_goal_simulator', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    robot = 'mur620a'
    ns = '/'+robot+'/navigation'
    native, nav2, logs = [], [], []
    node.create_subscription(PoseStamped, '/'+robot+'/move_base_simple/goal', native.append, 10)
    status = node.create_publisher(String, ns+'/status', LATCHED)
    static = node.create_publisher(TFMessage, '/tf_static', LATCHED)
    clicks = {mode: node.create_publisher(PoseStamped, ns+'/'+topic, 10)
              for mode, topic in [('mir','mir_goal_pose'), ('nav2','goal_pose')]}

    def execute(handle):
        nav2.append(handle.request.pose)
        handle.succeed()
        return NavigateToPose.Result()

    server = ActionServer(node, NavigateToPose, ns+'/navigate_to_pose', execute_callback=execute)
    processes = set()
    module = SimpleNamespace(
        context=SimpleNamespace(checked_robots=lambda:[robot], append_log=logs.append,
                                window=SimpleNamespace(open_mir_goals=lambda:None)),
        running=lambda name:name in processes,
        terminate_owned=processes.discard)
    panel = navigation_gui.DrivingPanel(module)
    calibration = TransformStamped(child_frame_id='mocap')
    calibration.header.frame_id = 'map'
    calibration.transform.translation.x, calibration.transform.translation.y = 10., 20.
    calibration.transform.rotation.z = math.sin(math.pi/4)
    calibration.transform.rotation.w = math.cos(math.pi/4)

    def pump(duration=.1):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            status.publish(String(data=json.dumps(dict(ready=True, busy=False, state='Simuliert bereit'))))
            static.publish(TFMessage(transforms=[calibration]))
            app.processEvents()
            executor.spin_once(timeout_sec=.01)

    def until(predicate, timeout=5.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            pump()
        assert predicate(), logs

    def pose():
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = 'mocap', node.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = 1., 2.
        message.pose.orientation.z, message.pose.orientation.w = math.sin(.3), math.cos(.3)
        return message

    try:
        until(lambda: all(p.get_subscription_count() for p in clicks.values()) and panel.state.get('ready'))
        # A transient-local replay or queued old click must never become a drive.
        stale = pose()
        stale.header.stamp.sec -= 5
        clicks['mir'].publish(stale)
        pump(.3)
        assert not native and not nav2
        clicks['mir'].publish(pose())
        until(lambda: bool(native))
        assert len(native) == 1 and not nav2
        assert native[0].header.frame_id == 'map'
        assert native[0].pose.position.x == pytest.approx(8.)
        assert native[0].pose.position.y == pytest.approx(21.)
        assert navigation_gui.yaw(native[0].pose.orientation) == pytest.approx(math.pi/2+.6)

        panel.backend.setCurrentIndex(1)
        panel.session_robot = robot
        processes.add('navigation_driving')  # Simulated ownership; no stack is launched.
        until(lambda: panel.state.get('states', {}).get(robot, {}).get('ready'))
        clicks['mir'].publish(pose())  # An old native RViz window cannot target Nav2.
        pump(.3)
        assert len(native) == 1 and not nav2
        clicks['nav2'].publish(pose())
        until(lambda: bool(nav2) and not panel.goal_pending)
        assert len(nav2) == 1 and len(native) == 1
        assert nav2[0].header.frame_id == 'mocap'
        assert nav2[0].pose.position.x == pytest.approx(1.)
        assert nav2[0].pose.position.y == pytest.approx(2.)
        assert navigation_gui.yaw(nav2[0].pose.orientation) == pytest.approx(.6)
        assert node.count_publishers('/'+robot+'/cmd_vel_stamped') == 0
    finally:
        panel.shutdown()
        panel.close()
        app.processEvents()
        server.destroy()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
