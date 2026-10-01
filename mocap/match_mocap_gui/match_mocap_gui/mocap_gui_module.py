"""Qualisys Mocap panel for the shared MuR GUI."""

import math
import time
from collections import deque
from functools import partial

from PyQt5 import QtCore, QtWidgets

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from match_mur_gui.base_gui import MurGuiModule, ROBOTS, setup_prefix


PROCESS_NAME = 'mocap_driver'
RATE_WINDOW_SEC = 2.0
STALE_AFTER_SEC = 0.5


class MocapMonitor(QtCore.QThread):
    snapshot = QtCore.pyqtSignal(object)
    error = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._stop = False
        self._context = None
        self._node = None
        self._times = {name: deque() for name in ROBOTS}
        self._ever_seen = set()
        self._last_smoothed = {}
        self._last_map = {}

    def shutdown(self):
        self._stop = True

    def _on_raw(self, robot, msg):
        if msg.header.frame_id != 'mocap':
            return
        now = time.monotonic()
        times = self._times[robot]
        times.append(now)
        self._ever_seen.add(robot)
        while times and now - times[0] > RATE_WINDOW_SEC:
            times.popleft()

    def _on_smoothed(self, robot, msg):
        if msg.header.frame_id != 'mocap':
            return
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._last_smoothed[robot] = (
            time.monotonic(), msg.pose.position.x, msg.pose.position.y, math.degrees(yaw),
        )

    def _on_map(self, robot, msg):
        if msg.header.frame_id == 'map':
            self._last_map[robot] = time.monotonic()

    def _emit_snapshot(self):
        now = time.monotonic()
        result = {}
        for robot, times in self._times.items():
            while times and now - times[0] > RATE_WINDOW_SEC:
                times.popleft()
            live = bool(times) and now - times[-1] <= STALE_AFTER_SEC
            hz = (len(times) - 1) / (times[-1] - times[0]) if live and len(times) > 1 and times[-1] > times[0] else 0.0
            pose = self._last_smoothed.get(robot)
            result[robot] = {
                'live': live,
                'seen': robot in self._ever_seen,
                'hz': hz,
                'pose': pose[1:] if live and pose and now - pose[0] <= STALE_AFTER_SEC else None,
                'map_live': live and now - self._last_map.get(robot, 0.0) <= STALE_AFTER_SEC,
            }
        self.snapshot.emit(result)

    def run(self):
        context = rclpy.context.Context()
        self._context = context
        executor = None
        try:
            rclpy.init(args=None, context=context)
            node = rclpy.create_node('mocap_gui_monitor', context=context)
            self._node = node
            for robot in ROBOTS:
                node.create_subscription(
                    PoseStamped, f'/qualisys/{robot}/pose', partial(self._on_raw, robot), 10
                )
                node.create_subscription(
                    PoseStamped, f'/qualisys/{robot}/pose_smoothed',
                    partial(self._on_smoothed, robot), 10
                )
                node.create_subscription(
                    PoseStamped, f'/qualisys_map/{robot}/pose_smoothed',
                    partial(self._on_map, robot), 10
                )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            next_display = 0.0
            while context.ok() and not self._stop:
                executor.spin_once(timeout_sec=0.01)
                if time.monotonic() >= next_display:
                    self._emit_snapshot()
                    next_display = time.monotonic() + 0.2
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as exc:
            self.error.emit(f'Mocap monitor stopped: {exc}')
        finally:
            if executor is not None:
                executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
                self._node = None
            if context.ok():
                rclpy.shutdown(context=context)
            self._context = None


class MocapGuiModule(MurGuiModule):
    def __init__(self):
        self.context = None
        self.monitor = None
        self.last_snapshot = {}

    def setup_ui(self, context):
        self.context = context
        self.start_button = context.add_action_button('Start Mocap', self.start_driver, section='Mocap')
        self.stop_button = context.add_action_button('Stop Mocap', self.stop_driver, section='Mocap')
        self.driver_status = QtWidgets.QLabel('driver stopped')
        context.add_status_row('Mocap', self.driver_status)

        panel = QtWidgets.QGroupBox('Qualisys Mocap — selected MuRs')
        layout = QtWidgets.QVBoxLayout(panel)
        self.map_check = QtWidgets.QCheckBox(
            'Map-Posen und Roboter-TF aus Qualisys publishen'
        )
        self.map_check.setChecked(True)
        self.map_check.setToolTip(
            'Liest map → mocap beim Start aus dem ROS1-Repo auf roscore. '
            'Roboter-TF nutzt /qualisys_map/<mur>/pose (100 Hz, ungemittelt). '
            'Qualisys-Starrkörper-Frame entspricht dem jeweiligen base_link. '
            'Roh- und Mittelwert-Posen unter /qualisys bleiben im mocap-Frame.'
        )
        self.map_check.toggled.connect(self.on_map_output_toggled)
        layout.addWidget(self.map_check)
        self.table = QtWidgets.QTableWidget(len(ROBOTS), 7)
        self.table.setHorizontalHeaderLabels(['MuR', 'Pose', 'Raw Hz', 'x [m]', 'y [m]', 'φ [°]', 'Map'])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setMaximumHeight(170)
        for row, robot in enumerate(ROBOTS):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(robot))
            for col in range(1, 7):
                self.table.setItem(row, col, QtWidgets.QTableWidgetItem('—'))
        layout.addWidget(self.table)
        hint = QtWidgets.QLabel(
            'Qualisys-Pose im Frame mocap (Starrkörper = base_link). '
            'Anzeige: gleitender Mittelwert über 0,2 s, max. 5 Hz. '
            'Map-Posen werden nur auf /qualisys_map publiziert. '
            'Roboter-TF nutzt die ungemittelte 6D-Pose ohne zusätzlichen Versatz.'
        )
        layout.addWidget(hint)
        context.add_panel(panel)
        self.on_robot_selection_changed()
        self.monitor = MocapMonitor()
        self.monitor.snapshot.connect(self._update_snapshot)
        self.monitor.error.connect(context.append_log)
        self.monitor.start()
        context.append_log('[mocap] GUI module loaded; monitoring Qualisys topics')

    def on_robot_selection_changed(self):
        if self.context is None or not hasattr(self, 'table'):
            return
        selected = set(self.context.selected_robots())
        for row, robot in enumerate(ROBOTS):
            self.table.setRowHidden(row, robot not in selected)

    def on_map_output_toggled(self, enabled):
        process = self.context.window.processes.get(PROCESS_NAME)
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.context.append_log(
                f"[mocap] Map output {'enabled' if enabled else 'disabled'}; restarting bridge"
            )
            self.stop_driver()
            self.start_driver()
        elif self.context is not None:
            self.context.append_log(
                f"[mocap] Map output {'enabled' if enabled else 'disabled'} for next start"
            )

    def _update_snapshot(self, snapshot):
        self.last_snapshot = snapshot
        for row, robot in enumerate(ROBOTS):
            state = snapshot[robot]
            if state['live']:
                status = 'Live' if state['pose'] is not None else 'Warte auf Mittelwert'
            else:
                status = 'Keine Pose' if not state['seen'] else 'Veraltet'
            values = [status, f"{state['hz']:.1f}" if state['live'] else '—']
            if state['pose'] is not None:
                x, y, yaw_deg = state['pose']
                values.extend([f'{x:.3f}', f'{y:.3f}', f'{yaw_deg:.1f}'])
            else:
                values.extend(['—'] * 3)
            values.append(
                ('Extern live' if state['map_live'] else 'Aus')
                if not self.map_check.isChecked()
                else 'Live' if state['map_live'] else 'Keine Pose'
            )
            for col, value in enumerate(values, start=1):
                item = self.table.item(row, col)
                item.setText(value)
                if col == 1:
                    item.setForeground(QtCore.Qt.darkGreen if state['live'] else QtCore.Qt.darkRed)

    def start_driver(self):
        process = self.context.window.processes.get(PROCESS_NAME)
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.context.append_log('[mocap] QTM bridge is already running')
            return
        map_enabled = 'true' if self.map_check.isChecked() else 'false'
        robot_tf_enabled = map_enabled
        command = (
            setup_prefix() + 'exec python3 -m match_mocap_ros2.qualisys_ssh_bridge '
            + f'--ros-args -p publish_map_pose:={map_enabled} -p publish_robot_tf:={robot_tf_enabled}'
        )
        self.driver_status.setText('starting…')
        self.context.append_log(
            f'[mocap] Starting QTM bridge at 100 Hz; map output {map_enabled}; '
            f'Qualisys raw-pose robot TF {robot_tf_enabled} (rigid body = base_link)'
        )
        self.context.start_process(PROCESS_NAME, command, on_finished=self._driver_finished)
        self.context.window.processes[PROCESS_NAME].started.connect(
            lambda: self.driver_status.setText('driver running')
        )

    def _driver_finished(self, code, _status):
        self.driver_status.setText(f'driver stopped (exit {code})')

    def stop_driver(self):
        process = self.context.window.processes.get(PROCESS_NAME)
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            self.context.append_log('[mocap] No GUI-managed bridge is running')
            self.driver_status.setText('driver stopped')
            return
        self.context.append_log('[mocap] Stopping QTM bridge')
        process.terminate()
        if not process.waitForFinished(2000):
            process.kill()
            process.waitForFinished(2000)
        self.driver_status.setText('driver stopped')

    def on_shutdown(self):
        if self.monitor is not None:
            self.monitor.shutdown()
            self.monitor.wait(2000)
            self.monitor = None
