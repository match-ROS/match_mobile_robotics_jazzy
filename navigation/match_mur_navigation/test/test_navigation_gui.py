from pathlib import Path
from types import SimpleNamespace
from PyQt5 import QtCore, QtWidgets
import pytest

from match_mur_navigation import navigation_gui
from match_mur_gui import base_gui


class Worker(QtCore.QObject):
    snapshot = QtCore.pyqtSignal(object)
    event = QtCore.pyqtSignal(str,str,bool)
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
