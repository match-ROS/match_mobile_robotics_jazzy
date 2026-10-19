"""GUI shutdown must reap real ROS launch children instead of orphaning them."""
import time
from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.executors import SingleThreadedExecutor
from match_mur_gui.base_gui import MurBaseGui, setup_prefix


def test_gui_shutdown_reaps_map_server_and_lifecycle_manager():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    class Owner(QtWidgets.QWidget):
        start_process = MurBaseGui.start_process
        stop_managed_processes = MurBaseGui.stop_managed_processes
        def __init__(self):
            super().__init__()
            self.processes = {}
            self.lines = []
        def append_log(self, line): self.lines.append(line)
        def _read_process_output(self, name, process):
            self.lines.append(bytes(process.readAllStandardOutput()).decode(errors='replace'))
        def stop_module_motion_like_actions(self): pass
        def selected_robots(self): return []

    owner = Owner()
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('gui_shutdown_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    owner.start_process('navigation_localization', setup_prefix() +
        'exec ros2 launch match_mur_navigation localization.launch.py robot:=mur620a localize:=false')
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            app.processEvents()
            executor.spin_once(timeout_sec=.05)
            if {('map_server', '/mur620a/localization'),
                ('localization_manager', '/mur620a/localization')} <= set(node.get_node_names_and_namespaces()):
                break
        else:
            raise AssertionError('\n'.join(owner.lines))
        owner.stop_managed_processes()
        assert owner.processes['navigation_localization'].state() == QtCore.QProcess.NotRunning
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if not any(ns == '/mur620a/localization' for name, ns in node.get_node_names_and_namespaces()):
                break
        else:
            raise AssertionError('GUI left ROS launch children running: ' + '\n'.join(owner.lines))
        assert any('SIGINT' in line for line in owner.lines)
    finally:
        owner.stop_managed_processes()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        owner.close()
        app.processEvents()
