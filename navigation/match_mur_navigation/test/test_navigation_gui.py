from pathlib import Path
import math
import time
from types import SimpleNamespace
from PyQt5 import QtCore, QtWidgets
import pytest

from match_mur_navigation import navigation_gui
from match_mur_gui import base_gui
from geometry_msgs.msg import PoseStamped


class Worker(QtCore.QObject):
    snapshot = QtCore.pyqtSignal(object)
    event = QtCore.pyqtSignal(str,str,bool)
    clicked_goal = QtCore.pyqtSignal(str,str,object,float)
    def __init__(self):
        super().__init__()
        self.requests = []
    def start(self): pass
    def shutdown(self): pass
    def wait(self,_timeout): pass
    def request(self,*args): self.requests.append(args)


@pytest.fixture
def panel(monkeypatch,tmp_path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(navigation_gui,'DrivingMonitor',Worker)
    monkeypatch.setattr(navigation_gui,'GUI_LOG_DIR',str(tmp_path/'gui'))
    processes = set()
    calls = []
    context = SimpleNamespace(checked_robots=lambda:['mur620c'],append_log=calls.append,
        window=SimpleNamespace(open_mir_goals=lambda:None),
        start_process=lambda *args,**kwargs:processes.add(args[0]))
    module = SimpleNamespace(context=context,temp=SimpleNamespace(name=str(tmp_path)),
        running=lambda name:name in processes,terminate_owned=processes.discard,start_mocap=lambda:None,
        command=lambda name,args:name+' '+' '.join(args))
    widget = navigation_gui.DrivingPanel(module)
    yield widget,processes,calls
    widget.shutdown()
    widget.close()
    app.processEvents()


def test_backends_share_goals_and_block_active_switches(panel):
    widget,processes,_ = panel
    assert widget.send('mur620c','map',1.,2.,.3) is None
    assert widget.send('mur620c','mocap',1.,2.,.3)[0]
    assert widget.worker.requests[-1][0] == 'native'
    widget.backend.setCurrentIndex(1)
    assert not widget.send('mur620c','map',1.,2.,.3)[0]
    processes.add('navigation_driving')
    widget.session_robot = 'mur620c'
    widget.update_snapshot(dict(ready=True,states={'mur620c':dict(ready=True,busy=False,state='Bereit')}))
    assert not widget.send('mur620d','map',1.,2.,.3)[0]
    assert widget.send('mur620c','map',1.,2.,.3)[0]
    assert widget.worker.requests[-1] == ('goal','mur620c',dict(frame='map',x=1.,y=2.,yaw=.3))
    assert not widget.backend.isEnabled()
    assert not widget.fields['max_speed'].isEnabled()
    assert not widget.send('mur620c','map',1.,2.,.3)[0]
    widget.cancel()
    assert widget.worker.requests[-1][:2] == ('cancel','mur620c')
    widget.stop()
    assert not widget.running()


def test_profiles_persist_per_robot(panel):
    widget,_,_ = panel
    widget.backend.setCurrentIndex(1)
    widget.fields['max_speed'].setValue(.3)
    widget.save_settings()
    assert widget.profiles_path.is_file()
    widget.fields['max_speed'].setValue(.4)
    widget.load_profile()
    assert widget.fields['max_speed'].value() == .3


def test_shared_goal_router_preserves_native_fallback():
    calls = []
    module = SimpleNamespace(on_mir_goal=lambda *args:(True,'Nav2'),
                             mir_navigation_description=lambda:'Nav2')
    window = SimpleNamespace(modules=[module],ros_worker=SimpleNamespace(
        publish_mir_goal=lambda *args:calls.append(args) or (True,'MiR')))
    assert base_gui.MurBaseGui.send_mir_navigation_goal(window,'mur620c','map',1,2,3) == (True,'Nav2')
    assert not calls
    module.on_mir_goal = lambda *args:None
    assert base_gui.MurBaseGui.send_mir_navigation_goal(window,'mur620c','map',1,2,3) == (True,'MiR')
    assert calls == [('mur620c','map',1,2,3)]


def click_pose(frame='mocap'):
    message = PoseStamped()
    message.header.frame_id = frame
    message.pose.position.x, message.pose.position.y = 1., 2.
    message.pose.orientation.z, message.pose.orientation.w = math.sin(.3), math.cos(.3)
    return message


def test_click_goals_follow_backend_robot_and_nav2_readiness(panel):
    widget, processes, logs = panel
    pose = click_pose('map')
    widget.receive_clicked_goal('mur620c', 'mir', pose, time.monotonic())
    operation, robot, goal = widget.worker.requests[-1]
    assert (operation, robot) == ('native', 'mur620c')
    assert goal == pytest.approx(dict(frame='map', x=1., y=2., yaw=.6))
    widget.worker.requests.clear()
    widget.receive_clicked_goal('mur620d', 'mir', pose, time.monotonic())
    widget.receive_clicked_goal('mur620c', 'nav2', pose, time.monotonic())
    assert not widget.worker.requests
    widget.backend.setCurrentIndex(1)
    widget.receive_clicked_goal('mur620c', 'nav2', pose, time.monotonic())
    assert not widget.worker.requests
    processes.add('navigation_driving')
    widget.session_robot = 'mur620c'
    widget.update_snapshot(dict(states={'mur620c': dict(ready=False, busy=False, state='Warte auf Scanner')}))
    widget.receive_clicked_goal('mur620c', 'nav2', pose, time.monotonic())
    assert not widget.worker.requests
    assert 'Warte auf Scanner' in widget.status.text()
    widget.update_snapshot(dict(states={'mur620c': dict(ready=True, busy=False, state='Bereit')}))
    widget.receive_clicked_goal('mur620c', 'nav2', pose, time.monotonic())
    assert widget.worker.requests[-1][0] == 'goal'
    assert widget.worker.requests[-1][2]['yaw'] == pytest.approx(.6)
    count = len(widget.worker.requests)
    widget.receive_clicked_goal('mur620c', 'nav2', pose, time.monotonic())
    assert len(widget.worker.requests) == count
    assert 'zuerst abbrechen' in widget.status.text()


@pytest.mark.parametrize('invalid', ['frame', 'nan', 'quaternion', 'delayed'])
def test_invalid_or_delayed_click_goals_never_send(panel, invalid):
    widget, _, _ = panel
    message = click_pose()
    received = time.monotonic()
    if invalid == 'frame': message.header.frame_id = 'base_link'
    if invalid == 'nan': message.pose.position.x = float('nan')
    if invalid == 'quaternion': message.pose.orientation.w = 0.
    if invalid == 'delayed': received -= 3.
    widget.receive_clicked_goal('mur620c', 'mir', message, received)
    assert not widget.worker.requests


def test_native_rviz_opens_without_nav2_and_closes_on_backend_change(panel):
    widget, processes, _ = panel
    assert widget.rviz_button.isEnabled()
    widget.open_rviz()
    assert 'navigation_driving_rviz' in processes
    assert widget.rviz_selection == ('mur620c', 'mir')
    widget.backend.setCurrentIndex(1)
    assert 'navigation_driving_rviz' not in processes
    assert not widget.rviz_button.isEnabled()


def test_qt_events_and_navigation_notifications(panel):
    widget, _, logs = panel
    # Qt dispatches one QEvent argument to QWidget.event().
    event = QtCore.QEvent(QtCore.QEvent.User)
    QtWidgets.QApplication.sendEvent(widget, event)
    widget.worker.event.emit('mur620c', 'Navigation bereit', False)
    assert widget.status.text() == 'Navigation bereit'
    assert logs[-1] == '[navigation] mur620c: Navigation bereit'
