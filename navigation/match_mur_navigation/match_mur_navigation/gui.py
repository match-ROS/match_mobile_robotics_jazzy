"""Navigation module for the shared PyQt MuR application."""
import hashlib
import json
import os
from pathlib import Path
import queue
import shlex
import signal
import sys
import tempfile
import threading
import time
import uuid

from PyQt5 import QtCore, QtWidgets
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from std_msgs.msg import String
from std_srvs.srv import SetBool
import yaml

from match_mur_gui.base_gui import MurBaseGui, MurGuiModule, setup_prefix
from match_mur_gui.app_icon import configure_gui_icon
from .session import LATCHED, valid_pose
from .settings import DEFAULTS, ROBOTS, validate


class NavigationMonitor(QtCore.QThread):
    snapshot = QtCore.pyqtSignal(object)
    map_changed = QtCore.pyqtSignal(str, str, int)
    log = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.stop_event = threading.Event()
        self.commands = queue.Queue()
        self.identifier = uuid.uuid4().hex[:8]
        self.sources = {}
        self.statuses = {}
        self.revisions = {robot: 0 for robot in ROBOTS}
        self.digests = {}
        self.publishers = {}
        self.expected_sessions = {}

    def cache_topic(self, robot):
        return f'/{robot}/mapping/gui_{self.identifier}/map'

    def request_stop(self, robot):
        self.commands.put(('stop', robot))

    def reset(self, robot, session_id):
        self.commands.put(('reset', (robot, session_id)))

    def shutdown(self):
        self.stop_event.set()

    def run(self):
        context = rclpy.context.Context()
        node = executor = None
        clients = {}
        pending = []
        try:
            rclpy.init(args=[], context=context)
            node = rclpy.create_node('mur_navigation_monitor_' + self.identifier, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            def raw(robot, msg):
                if valid_pose(msg):
                    self.sources[robot] = time.monotonic()
            def status(robot, msg):
                try:
                    self.statuses[robot] = (time.monotonic(), json.loads(msg.data))
                except ValueError:
                    pass
            def map_received(robot, msg):
                if not msg.data or len(msg.data) != msg.info.width * msg.info.height:
                    return
                session_id = self.expected_sessions.get(robot)
                status = self.statuses.get(robot, (0, {}))[1]
                stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                if (not session_id or status.get('session_id') != session_id or
                        not status.get('has_map') or stamp < status.get('map_stamp', 0)):
                    return
                geometry = (msg.header.frame_id, msg.info.resolution, msg.info.width, msg.info.height,
                            msg.info.origin.position.x, msg.info.origin.position.y)
                digest = (geometry, hashlib.blake2b(msg.data.tobytes(), digest_size=16).digest())
                self.publishers[robot].publish(msg)
                if digest != self.digests.get(robot):
                    self.digests[robot] = digest
                    self.revisions[robot] += 1
                    self.map_changed.emit(robot, session_id, self.revisions[robot])
            for robot in ROBOTS:
                self.publishers[robot] = node.create_publisher(OccupancyGrid, self.cache_topic(robot), LATCHED)
                node.create_subscription(PoseStamped, f'/qualisys/{robot}/pose',
                                         lambda m, r=robot: raw(r, m), 100)
                node.create_subscription(String, f'/{robot}/mapping/status',
                                         lambda m, r=robot: status(r, m), LATCHED)
                node.create_subscription(OccupancyGrid, f'/{robot}/mapping/map',
                                         lambda m, r=robot: map_received(r, m), LATCHED)
                clients[robot] = node.create_client(SetBool, f'/{robot}/mapping/set_recording')
            next_snapshot = 0
            discovery_started = time.monotonic()
            while context.ok() and not self.stop_event.is_set():
                executor.spin_once(timeout_sec=0.02)
                while not self.commands.empty():
                    action, value = self.commands.get_nowait()
                    if action == 'reset':
                        robot, session_id = value
                        self.expected_sessions[robot] = session_id
                        self.digests.pop(robot, None)
                        self.statuses.pop(robot, None)
                        node.destroy_publisher(self.publishers[robot])
                        self.publishers[robot] = node.create_publisher(OccupancyGrid, self.cache_topic(robot), LATCHED)
                    elif clients[value].service_is_ready():
                        robot = value
                        pending.append((clients[robot].call_async(SetBool.Request(data=False)), time.monotonic()))
                    else:
                        self.log.emit('[mapping] Stop-Dienst nicht erreichbar; Backendstatus prüfen.')
                for future, started in list(pending):
                    if future.done():
                        try:
                            result = future.result()
                            self.log.emit('[mapping] ' + result.message)
                        except Exception as exc:
                            self.log.emit(f'[mapping] Stop fehlgeschlagen: {exc}')
                        pending.remove((future, started))
                    elif time.monotonic() - started > 3:
                        future.cancel()
                        pending.remove((future, started))
                        self.log.emit('[mapping] Stop-Zeitüberschreitung; Backendstatus prüfen.')
                if time.monotonic() >= next_snapshot:
                    now = time.monotonic()
                    names = node.get_node_names()
                    self.snapshot.emit(dict(
                        discovery_ready=now-discovery_started >= 2,
                        raw={r: now - self.sources.get(r, -100) < 0.5 for r in ROBOTS},
                        bridge_present='qualisys_ssh_bridge' in names,
                        owners={r: node.count_publishers(f'/{r}/mapping/status') > 0 for r in ROBOTS},
                        statuses={r: value for r, (stamp, value) in self.statuses.items() if now-stamp < 2}))
                    next_snapshot = now + 0.2
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        except Exception as exc:
            self.log.emit(f'[mapping] Monitorfehler: {exc}')
        finally:
            if executor:
                executor.shutdown()
            if node:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)


class CloseGuard(QtCore.QObject):
    def __init__(self, module, parent):
        super().__init__(parent)
        self.module = module

    def eventFilter(self, watched, event):
        if event.type() == QtCore.QEvent.Close:
            if self.module.saving:
                self.module.context.append_log('[mapping] Bitte den Kartenexport vor dem Schließen abwarten.')
                return True
            if self.module.dirty and not self.module.confirm_discard():
                return True
        return False


class NavigationModule(MurGuiModule):
    def __init__(self):
        self.context = None
        self.session = None
        self.active = False
        self.saving = False
        self.dirty = False
        self.has_map = False
        self.revision = 0
        self.last_snapshot = {}
        self.stopping = False
        self.temp = tempfile.TemporaryDirectory(prefix='mur_navigation_')

    def setup_ui(self, context):
        self.context = context
        self.panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.panel)
        form = QtWidgets.QFormLayout()
        self.robot = QtWidgets.QComboBox()
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem('Mocap-Kartierung', 'mocap')
        self.mode.addItem('SLAM ohne Mocap', 'slam')
        form.addRow('Kartierungsroboter', self.robot)
        form.addRow('Betriebsart', self.mode)
        layout.addLayout(form)
        self.hint = QtWidgets.QLabel('Mocap: Ursprung und Achsen entsprechen Qualisys. '
                                    'SLAM: eigener Kartenursprung ohne zugesicherten Mocap-Bezug.')
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        buttons = QtWidgets.QGridLayout()
        def button(text, callback, row, col):
            widget = QtWidgets.QPushButton(text)
            widget.clicked.connect(callback)
            buttons.addWidget(widget, row, col)
            return widget
        self.mocap_start = button('Mocap starten', self.start_mocap, 0, 0)
        self.mocap_stop = button('Mocap stoppen', self.stop_mocap, 0, 1)
        self.start_button = button('Kartierung starten', self.start_mapping, 1, 0)
        self.stop_button = button('Kartierung stoppen', self.stop_mapping, 1, 1)
        self.save_button = button('Karte speichern', self.save_map, 2, 0)
        self.rviz_button = button('RViz öffnen', self.open_rviz, 2, 1)
        layout.addLayout(buttons)
        self.auto_rviz = QtWidgets.QCheckBox('RViz beim Kartierungsstart öffnen')
        layout.addWidget(self.auto_rviz)
        toggle = QtWidgets.QToolButton()
        toggle.setText('Kartierungseinstellungen')
        toggle.setCheckable(True)
        toggle.setArrowType(QtCore.Qt.RightArrow)
        toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        layout.addWidget(toggle)
        self.settings_panel = QtWidgets.QWidget()
        settings_layout = QtWidgets.QFormLayout(self.settings_panel)
        self.fields = {}
        for key, label, low, high, step in (
            ('resolution', 'Auflösung [m]', 0.005, 0.2, 0.005),
            ('min_range', 'Minimale Reichweite [m]', 0.0, 99.0, 0.1),
            ('max_range', 'Maximale Reichweite [m]', 0.1, 100.0, 0.5),
            ('map_update_interval', 'Kartenaktualisierung [s]', 0.1, 30.0, 0.1)):
            field = QtWidgets.QDoubleSpinBox()
            field.setDecimals(3)
            field.setRange(low, high)
            field.setSingleStep(step)
            field.setValue(DEFAULTS[key])
            self.fields[key] = field
            settings_layout.addRow(label, field)
        advanced_toggle = QtWidgets.QCheckBox('Erweiterte Zeiteinstellungen')
        settings_layout.addRow(advanced_toggle)
        advanced = QtWidgets.QWidget()
        advanced_form = QtWidgets.QFormLayout(advanced)
        for key, label in (('mocap_time_offset', 'Mocap-Zeitkorrektur [s]'),
                           ('front_time_offset', 'Vorderscanner-Zeitkorrektur [s]'),
                           ('rear_time_offset', 'Rückscanner-Zeitkorrektur [s]')):
            field = QtWidgets.QDoubleSpinBox()
            field.setDecimals(3)
            field.setRange(-1.0, 1.0)
            field.setSingleStep(0.005)
            self.fields[key] = field
            advanced_form.addRow(label, field)
        advanced_form.addRow(QtWidgets.QLabel('Korrektur wird zum Empfangszeitstempel addiert.'))
        advanced.hide()
        advanced_toggle.toggled.connect(advanced.setVisible)
        settings_layout.addRow(advanced)
        self.sim = QtWidgets.QCheckBox('Simulationszeit verwenden')
        settings_layout.addRow(self.sim)
        self.settings_panel.hide()
        toggle.toggled.connect(self.settings_panel.setVisible)
        toggle.toggled.connect(lambda checked: toggle.setArrowType(QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow))
        layout.addWidget(self.settings_panel)
        self.status = QtWidgets.QLabel('Bereit – einen markierten MuR auswählen.')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.inputs = QtWidgets.QLabel('Keine aktuellen Sitzungsdaten')
        self.inputs.setWordWrap(True)
        layout.addWidget(self.inputs)
        layout.addStretch()
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidget(self.panel)
        context.add_module_tab(scroll, 'Kartierung')
        self.guard = CloseGuard(self, context.window)
        context.window.installEventFilter(self.guard)
        self.monitor = NavigationMonitor()
        self.monitor.snapshot.connect(self.update_snapshot)
        self.monitor.map_changed.connect(self.on_map_changed)
        self.monitor.log.connect(context.append_log)
        self.monitor.start()
        self.robot.currentIndexChanged.connect(self.update_buttons)
        self.on_robot_selection_changed()

    def on_robot_selection_changed(self):
        if self.context is None or self.active:
            return
        previous = self.robot.currentText()
        self.robot.blockSignals(True)
        self.robot.clear()
        self.robot.addItems(self.context.checked_robots())
        if previous in self.context.checked_robots():
            self.robot.setCurrentText(previous)
        self.robot.blockSignals(False)
        self.update_buttons()

    def update_buttons(self):
        self.start_button.setEnabled(bool(self.robot.currentText()) and not self.active and not self.saving)
        self.stop_button.setEnabled(self.active and not self.stopping)
        self.save_button.setEnabled(self.has_map and not self.saving)
        self.rviz_button.setEnabled(self.session is not None)
        for widget in (self.robot, self.mode, self.settings_panel, self.auto_rviz):
            widget.setEnabled(not self.active and not self.saving)
        self.mocap_stop.setEnabled(self.running('navigation_mocap'))
        self.mocap_start.setEnabled(bool(self.last_snapshot.get('discovery_ready')) and
                                   not self.running('navigation_mocap'))

    def running(self, name):
        process = self.context.window.processes.get(name)
        return process is not None and process.state() != QtCore.QProcess.NotRunning

    def command(self, module, arguments):
        return setup_prefix() + 'exec python3 -m ' + module + ' ' + shlex.join(arguments)

    def start_mocap(self):
        if not self.last_snapshot.get('discovery_ready'):
            self.context.append_log('[mapping] Warte auf ROS-Erkennung vorhandener Qualisys-Quellen.')
            return
        if self.running('navigation_mocap'):
            return
        if self.last_snapshot.get('bridge_present') or any(self.last_snapshot.get('raw', {}).values()):
            self.context.append_log('[mapping] Vorhandene Qualisys-Bridge wird verwendet.')
            return
        self.context.start_process('navigation_mocap', self.command('match_mocap_ros2.qualisys_ssh_bridge',
            ['--ros-args', '-p', 'publish_map_pose:=false', '-p', 'publish_robot_tf:=false']),
            on_finished=lambda *_: self.update_buttons())
        QtCore.QTimer.singleShot(300, self.update_buttons)

    def terminate_owned(self, name):
        process = self.context.window.processes.get(name)
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            return
        try:
            os.kill(int(process.processId()), signal.SIGINT)
        except ProcessLookupError:
            return
        if not process.waitForFinished(4000):
            process.terminate()
            if not process.waitForFinished(2000):
                process.kill()
                process.waitForFinished(1000)

    def stop_mocap(self):
        self.terminate_owned('navigation_mocap')
        self.update_buttons()

    def confirm_discard(self):
        return QtWidgets.QMessageBox.question(self.panel, 'Ungespeicherte Karte',
            'Die aktuelle Karte enthält ungespeicherte Änderungen. Verwerfen?',
            QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel,
            QtWidgets.QMessageBox.Cancel) == QtWidgets.QMessageBox.Discard

    def start_mapping(self):
        robot = self.robot.currentText()
        if not robot or self.active or self.saving:
            return
        try:
            settings = validate(robot, self.mode.currentData(), {k: f.value() for k, f in self.fields.items()})
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        old_robot = self.session['robot'] if self.session else None
        if self.last_snapshot.get('owners', {}).get(robot) and not (
                old_robot == robot and self.running('navigation_mapping')):
            self.status.setText('Für diesen MuR läuft bereits eine externe Kartierung.')
            return
        if self.dirty and not self.confirm_discard():
            return
        self.terminate_owned('navigation_rviz')
        self.terminate_owned('navigation_mapping')
        self.session = dict(robot=robot, mode=self.mode.currentData(), settings=settings,
                            use_sim_time=self.sim.isChecked(), session_id=uuid.uuid4().hex)
        self.active = True
        self.stopping = False
        self.has_map = self.dirty = False
        self.revision = 0
        self.monitor.reset(robot, self.session['session_id'])
        directory = Path(self.temp.name)
        config = directory / 'parameters.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}), encoding='utf-8')
        (directory / 'session.yaml').write_text(yaml.safe_dump(self.session), encoding='utf-8')
        arguments = ['ros2', 'launch', 'match_mur_navigation', 'mapping.launch.py', f'robot:={robot}',
                     f"mode:={self.session['mode']}", 'use_rviz:=false',
                     f"session_id:={self.session['session_id']}",
                     f"use_sim_time:={str(self.sim.isChecked()).lower()}", f'params_file:={config}']
        self.status.setText('Kartierung startet; warte auf passende Sensordaten …')
        self.context.start_process('navigation_mapping', setup_prefix() + 'exec ' + shlex.join(arguments),
                                   on_finished=self.backend_finished)
        self.update_buttons()
        if self.auto_rviz.isChecked():
            self.open_rviz()

    def stop_mapping(self):
        if not self.active or not self.session:
            return
        self.stopping = True
        self.monitor.request_stop(self.session['robot'])
        self.update_buttons()
        QtCore.QTimer.singleShot(3500, self.stop_timeout)

    def stop_timeout(self):
        if self.stopping:
            self.stopping = False
            self.status.setText('Stop nicht bestätigt. Erneut versuchen oder verwaltete Prozesse stoppen.')
            self.update_buttons()

    def backend_finished(self, code, _status):
        self.active = self.stopping = False
        self.status.setText(f'Kartierungsprozess beendet (Code {code}). Empfangene Karte bleibt speicherbar.')
        self.on_robot_selection_changed()

    def update_snapshot(self, snapshot):
        self.last_snapshot = snapshot
        if self.session and self.running('navigation_mapping'):
            status = snapshot.get('statuses', {}).get(self.session['robot'])
            if status and status.get('session_id') == self.session['session_id']:
                self.active = status['recording']
                if not self.active:
                    self.stopping = False
                labels = {'recording': 'Aufnahme läuft', 'waiting_for_input': 'Warte auf Sensordaten',
                          'waiting_for_pose': 'Warte auf zeitlich passende Mocap-Pose',
                          'waiting_for_mocap': 'Aufnahme pausiert – warte auf Mocap',
                          'waiting_for_scanner_tf': 'Warte auf Scannertransformation',
                          'waiting_for_mapper': 'Warte auf Kartierungskern',
                          'waiting_for_map': 'SLAM startet – warte auf erste Karte',
                          'scan_expired': 'Scans ohne passende Zeitzuordnung verworfen',
                          'grid_limit': 'Kartengrößenlimit erreicht', 'stopped': 'Aufnahme gestoppt'}
                mode = 'Mocap' if self.session['mode'] == 'mocap' else 'SLAM'
                self.status.setText(f"{self.session['robot']} · {mode} · {labels.get(status['state'], status['state'])}")
                live = status['live']
                self.inputs.setText(' | '.join(f"{name}: {'live' if live.get(key) else 'fehlt'}" for key, name in
                    (('front_scan', 'Vorne'), ('rear_scan', 'Hinten'), ('mocap', 'Mocap'), ('odom', 'Odometrie'))) +
                    f" | Scans eingetragen: {status['integrated']} · verworfen: {status['discarded']}" +
                    (' (Zähler nur für Mocap)' if self.session['mode'] == 'slam' else ''))
        self.update_buttons()

    def on_map_changed(self, robot, session_id, revision):
        if self.session and robot == self.session['robot'] and session_id == self.session['session_id']:
            self.revision = revision
            self.has_map = self.dirty = True
            self.update_buttons()

    def open_rviz(self):
        if not self.session or self.running('navigation_rviz'):
            return
        arguments = ['--robot', self.session['robot'], '--mode', self.session['mode']]
        if self.session['use_sim_time']:
            arguments.append('--sim')
        self.context.start_process('navigation_rviz', self.command('match_mur_navigation.rviz', arguments))

    def save_map(self):
        if not self.session or not self.has_map or self.saving:
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self.panel, 'Karte speichern',
            str(Path.home() / 'maps' / (self.session['robot'] + '_' + self.session['mode'])),
            'Karten-YAML (*.yaml)', options=QtWidgets.QFileDialog.DontConfirmOverwrite)
        if not filename:
            return
        base = Path(filename)
        if base.suffix == '.yaml':
            base = base.with_suffix('')
        overwrite = any(Path(str(base) + suffix).exists() for suffix in ('.yaml', '.pgm', '.mapping.yaml'))
        if overwrite and QtWidgets.QMessageBox.question(self.panel, 'Karte überschreiben',
                'Vorhandene Kartendateien mit diesem Namen ersetzen?',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Cancel) != QtWidgets.QMessageBox.Yes:
            return
        self.saving = True
        revision = self.revision
        outputs = [Path(str(base) + suffix) for suffix in ('.yaml', '.pgm', '.mapping.yaml')]
        def finished(code, _status):
            self.saving = False
            if code == 0 and all(p.is_file() and p.stat().st_size for p in outputs):
                self.dirty = self.revision != revision
                self.context.append_log(f'[mapping] Karte gespeichert: {outputs[0]}')
                self.status.setText('Karte gespeichert: ' + str(outputs[0]))
            else:
                self.context.append_log('[mapping] Kartenexport fehlgeschlagen; Details im Prozesslog.')
                self.status.setText('Kartenexport fehlgeschlagen. Karte bleibt verfügbar.')
            self.update_buttons()
        args = ['--session', str(Path(self.temp.name) / 'session.yaml'), '--output', str(base),
                '--topic', self.monitor.cache_topic(self.session['robot'])]
        if overwrite:
            args.append('--overwrite')
        self.context.start_process('navigation_save', self.command('match_mur_navigation.save_map', args), on_finished=finished)
        self.update_buttons()

    def on_shutdown(self):
        self.monitor.shutdown()
        self.monitor.wait(3000)
        self.temp.cleanup()


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '62')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    icon = configure_gui_icon(app, 'mur-navigation-gui')
    window = MurBaseGui(modules=[NavigationModule()], window_title='MuR Navigation')
    window.setWindowIcon(icon)
    window.resize(1280, 1000)
    window.show()
    sys.exit(app.exec_())
