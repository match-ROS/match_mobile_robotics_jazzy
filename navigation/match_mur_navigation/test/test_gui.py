from PyQt5 import QtCore, QtWidgets
import pytest
from match_mur_navigation import gui
from match_mur_navigation import navigation_gui
from match_mur_gui import base_gui


class Monitor(QtCore.QObject):
    snapshot = QtCore.pyqtSignal(object)
    map_changed = QtCore.pyqtSignal(str, str, int)
    log = QtCore.pyqtSignal(str)
    session_end_result = QtCore.pyqtSignal(str, bool, str)
    def start(self): pass
    def shutdown(self): pass
    def wait(self, _timeout): pass
    def reset(self, robot, session_id): self.reset_robot = robot
    def request_stop(self, robot): self.stop_robot = robot
    def request_end(self, robot): self.end_robot = robot
    def request_stationary(self, robot, enabled): self.stationary_request = (robot, enabled)
    def cache_topic(self, robot): return '/test/cache/' + robot


class DrivingWorker(QtCore.QObject):
    snapshot = QtCore.pyqtSignal(object)
    event = QtCore.pyqtSignal(str,str,bool)
    clicked_goal = QtCore.pyqtSignal(str,str,object,float)
    def start(self): pass
    def shutdown(self): pass
    def wait(self,_timeout): pass
    def request(self,*_args): pass


@pytest.fixture(autouse=True)
def mock_driving_monitor(monkeypatch):
    monkeypatch.setattr(navigation_gui,'DrivingMonitor',DrivingWorker)


class Context:
    def __init__(self):
        self.window = QtWidgets.QWidget()
        self.window.open_mir_goals = lambda:None
        self.window.processes = {}
        self.selected = []
        self.commands = []
        self.logs = []
    def checked_robots(self): return self.selected
    def append_log(self, text): self.logs.append(text)
    def add_module_tab(self, panel, title): self.tab = panel
    def start_process(self, name, command, **kwargs): self.commands.append((name, command, kwargs))


def test_explicit_selection_locks_session_and_confirms_replacement(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'NavigationMonitor', Monitor)
    context = Context()
    module = gui.NavigationModule()
    module.setup_ui(context)
    assert not module.start_button.isEnabled()
    module.start_mapping()
    assert not context.commands
    context.selected = ['mur620a', 'mur620d']
    module.on_robot_selection_changed()
    module.robot.setCurrentText('mur620a')
    module.start_mapping()
    assert module.session['robot'] == 'mur620a'
    assert module.session['mode'] == 'mocap'
    assert not module.robot.isEnabled()
    assert not module.settings_panel.isEnabled()
    assert 'robot:=mur620a' in context.commands[-1][1]
    assert 'use_rviz:=false' in context.commands[-1][1]
    context.selected = ['mur620d']
    module.on_robot_selection_changed()
    assert module.robot.currentText() == 'mur620a'
    module.on_map_changed('mur620a', module.session['session_id'], 1)
    module.backend_finished(1, 0)
    assert module.has_map and module.save_button.isEnabled()
    assert module.robot.currentText() == 'mur620d'
    monkeypatch.setattr(module, 'confirm_discard', lambda: False)
    module.start_mapping()
    assert len(context.commands) == 1
    assert module.session['robot'] == 'mur620a'
    module.dirty = False
    module.on_shutdown()
    context.window.close()
    app.processEvents()


def test_external_mocap_and_slam_selection(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'NavigationMonitor', Monitor)
    context = Context()
    context.selected = ['mur620d']
    module = gui.NavigationModule()
    module.setup_ui(context)
    module.last_snapshot = {'bridge_present': True, 'discovery_ready': True}
    module.start_mocap()
    assert not context.commands
    assert 'Vorhandene Qualisys-Bridge' in context.logs[-1]
    module.mode.setCurrentIndex(1)
    module.start_mapping()
    assert module.session['mode'] == 'slam'
    module.stop_mapping()
    assert module.monitor.stop_robot == 'mur620d'
    module.on_shutdown()
    context.window.close()
    app.processEvents()


def test_external_mapping_can_be_ended_for_only_the_selected_robot(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'NavigationMonitor', Monitor)
    context = Context()
    context.selected = ['mur620a', 'mur620c']
    module = gui.NavigationModule()
    module.setup_ui(context)
    module.robot.setCurrentText('mur620c')
    module.update_snapshot({'owners': {'mur620a': True, 'mur620c': True}})
    assert not module.start_button.isEnabled()
    assert module.end_button.isEnabled()
    assert module.end_button.text() == 'Externe Kartierung beenden'
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, 'question', lambda *_args: QtWidgets.QMessageBox.Cancel)
    module.end_mapping()
    assert not module.ending_robot
    assert not hasattr(module.monitor, 'end_robot')
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, 'question', lambda *_args: QtWidgets.QMessageBox.Yes)
    module.end_mapping()
    assert module.monitor.end_robot == 'mur620c'
    assert not module.robot.isEnabled()
    assert not module.end_button.isEnabled()
    module.end_result('mur620a', False, 'service_unavailable')
    assert not context.commands
    module.end_result('mur620c', False, 'service_unavailable')
    name, command, options = context.commands[-1]
    assert name == 'navigation_end_mur620c'
    assert 'match_mur_navigation.stop_session --robot mur620c' in command
    assert 'mur620a' not in command
    options['on_finished'](0, 0)
    assert module.ending_robot == 'mur620c'  # Wait for ROS ownership to disappear.
    module.update_snapshot({'owners': {'mur620a': True, 'mur620c': False}})
    assert not module.ending_robot
    assert module.start_button.isEnabled()
    assert not module.end_button.isEnabled()
    module.on_shutdown()
    context.window.close()
    app.processEvents()


def test_failed_external_end_and_old_timeout_do_not_unlock_a_new_request(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'NavigationMonitor', Monitor)
    monkeypatch.setattr(gui.QtWidgets.QMessageBox, 'question', lambda *_args: QtWidgets.QMessageBox.Yes)
    context = Context()
    context.selected = ['mur620c']
    module = gui.NavigationModule()
    module.setup_ui(context)
    module.update_snapshot({'owners': {'mur620c': True}})
    module.end_mapping()
    previous = module.end_generation
    module.end_result('mur620c', False, 'Backend nicht erreichbar')
    assert module.ending_robot is None
    assert module.end_button.isEnabled()
    module.end_mapping()
    module.end_timeout(previous)
    assert module.ending_robot == 'mur620c'
    module.end_timeout(module.end_generation)
    assert module.ending_robot is None
    module.on_shutdown()
    context.window.close()
    app.processEvents()


def test_module_in_actual_base_window(monkeypatch, tmp_path):
    class Signal:
        def connect(self, _callback): pass
    class RosWorker:
        def __init__(self, _robots):
            self.log = Signal()
            self.freedrive_status = Signal()
            self.battery_status = Signal()
            self.battery_details = Signal()
        def start(self): pass
        def shutdown(self): pass
        def wait(self, _timeout): pass
        def set_robot_names(self, _robots): pass
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'NavigationMonitor', Monitor)
    monkeypatch.setattr(base_gui, 'RosWorker', RosWorker)
    monkeypatch.setattr(base_gui, 'configure_ros_discovery', lambda *_args: 'DDS test configuration')
    monkeypatch.setattr(base_gui.MurBaseGui, '_create_gui_log_file', lambda self: str(tmp_path/'gui.log'))
    module = gui.NavigationModule()
    window = base_gui.MurBaseGui(modules=[module], window_title='MuR Navigation')
    window.stop_managed_processes = lambda: None
    window.resize(1280, 1000)
    window.show()
    app.processEvents()
    assert [window.module_tabs.tabText(i) for i in range(3)] == ['Navigation','Lokalisierung','Kartierung']
    assert module.robot.currentText() == 'mur620d'
    assert module.start_button.isEnabled()
    captured = []
    monkeypatch.setattr(window, 'start_process', lambda name, command, **kwargs: captured.append((name, command)))
    window.module_tabs.setCurrentWidget(module.localization_scroll)
    window.open_rviz()
    assert captured[-1][0] == 'navigation_localization_rviz'
    assert '--robot mur620c --mode localization' in captured[-1][1]
    assert 'moveit' not in captured[-1][1]
    window.modules = [base_gui.MurGuiModule()]
    window.open_rviz()
    assert captured[-1][0] == 'rviz_moveit'  # Other applications keep the base view.
    window.modules = [module]
    module.settings_panel.show()
    app.processEvents()
    assert window.height() == 1000
    for check in window.robot_checks.values():
        check.setChecked(False)
    assert not module.start_button.isEnabled()
    window.robot_checks['mur620a'].setChecked(True)
    assert module.robot.currentText() == 'mur620a'
    window.grab().save(str(tmp_path/'navigation.png'))
    window.close()
    app.processEvents()


def test_default_map_load_and_amcl_are_separate_from_mapping(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'NavigationMonitor', Monitor)
    context = Context()
    module = gui.NavigationModule()
    module.setup_ui(context)
    panel = module.localization_panel
    assert panel.robot.currentText() == 'mur620c'
    assert panel.path.name == 'mur620c_mocap.yaml'
    assert panel.scene.items()
    assert panel.stationary.isChecked()
    assert not context.commands
    module.update_snapshot({'discovery_ready': True})
    name, command, _ = context.commands[-1]
    assert name == 'navigation_localization'
    assert 'localize:=false' in command
    assert 'mur620c_mocap.yaml' in command
    assert module.session is None and not module.active
    panel.stationary.setChecked(True)
    panel.start_amcl()
    assert 'localize:=true' in context.commands[-1][1]
    assert 'stationary_updates:=true' in context.commands[-1][1]
    assert not panel.robot.isEnabled()
    assert not panel.start_button.isEnabled()
    assert panel.stationary.isEnabled()
    panel.stationary.setChecked(False)
    assert module.monitor.stationary_request == ('mur620c', False)
    module.update_snapshot(dict(comparisons={'mur620c': dict(
        live=dict(encoder_odom=True, front_scan=True, mocap=True, amcl=True, amcl_updates=False),
        initialized_from_mocap_once=True, stationary_updates=False, samples=0)}))
    assert 'AMCL lokalisiert' in panel.status.text()
    assert 'Standmessung' in panel.metrics.text()
    panel.stop()
    count = len(context.commands)
    module.update_snapshot({'discovery_ready': True})
    assert len(context.commands) == count  # User stop must not auto-restart.
    panel.started = False
    panel.snapshot = {'discovery_ready': True, 'localization_owners': {'mur620c': True}}
    panel.start_amcl()
    assert len(context.commands) == count  # Never terminate/reconfigure an external AMCL.
    module.on_shutdown()
    context.window.close()
    app.processEvents()


def test_localization_distinguishes_orphan_adapter_from_reusable_map(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui, 'NavigationMonitor', Monitor)
    context = Context()
    module = gui.NavigationModule()
    module.setup_ui(context)
    panel = module.localization_panel
    state = dict(discovery_ready=True, localization_owners={'mur620c': True},
                 localization_backends={'mur620c': dict(map=True, adapter=True, amcl=False)},
                 localization_map_paths={'mur620c': str(panel.path.resolve())})
    module.update_snapshot(state)
    assert not panel.start_button.isEnabled()
    assert 'AMCL-Prozess fehlt' in panel.status.text()
    panel.start_amcl()
    assert not context.commands
    state['localization_backends']['mur620c']['adapter'] = False
    state['localization_map_paths']['mur620c'] = '/tmp/different-map.yaml'
    module.update_snapshot(state)
    assert not panel.start_button.isEnabled()
    panel.start_amcl()
    assert not context.commands
    state['localization_map_paths']['mur620c'] = str(panel.path.resolve())
    module.update_snapshot(state)
    assert panel.start_button.isEnabled()
    assert 'AMCL ist nicht gestartet' in panel.status.text()
    panel.start_amcl()
    assert 'load_map:=false' in context.commands[-1][1]
    assert 'localize:=true' in context.commands[-1][1]
    assert panel.localizing
    panel.stop()
    module.on_shutdown()
    context.window.close()
    app.processEvents()
