"""Driving controls, persistent limits and asynchronous goal routing."""
import copy
import json
import math
import queue
import threading
import time
import uuid
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_pose_stamped
import yaml

from match_mur_gui.base_gui import GUI_LOG_DIR, setup_prefix
from .navigation_config import DEFAULTS, BOUNDS, validate
from .saved_map import default_map, map_metadata
from .session import LATCHED
from .settings import ROBOTS
from .comparison import yaw
import shlex


def clicked_pose_values(message):
    """Validate an RViz map goal before reducing it to planar coordinates."""
    frame = message.header.frame_id.strip('/')
    p, q = message.pose.position, message.pose.orientation
    if frame not in ('map', 'mocap'):
        raise ValueError('Klickziel benötigt den Kartenrahmen map oder mocap')
    if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
            abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1.) > .01):
        raise ValueError('Ungültige Klickziel-Pose')
    return dict(frame=frame, x=p.x, y=p.y, yaw=yaw(q))


class DrivingMonitor(QtCore.QThread):
    snapshot = QtCore.pyqtSignal(object)
    event = QtCore.pyqtSignal(str, str, bool)
    clicked_goal = QtCore.pyqtSignal(str, str, object, float)

    def __init__(self):
        super().__init__()
        self.commands = queue.Queue()
        self.stopping = threading.Event()

    def request(self, operation, robot, pose=None):
        self.commands.put((operation, robot, pose))

    def shutdown(self):
        self.stopping.set()

    def run(self):
        context = rclpy.context.Context()
        node = executor = None
        statuses, clients, cancels, handles, generations, publishers = {}, {}, {}, {}, {}, {}
        try:
            rclpy.init(args=[], context=context)
            node = rclpy.create_node('mur_driving_gui_'+uuid.uuid4().hex[:8], context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            transforms = Buffer()
            def static(message):
                for t in message.transforms:
                    if (t.header.frame_id,t.child_frame_id) == ('map','mocap'):
                        transforms.set_transform_static(t, 'calibration')
            node.create_subscription(TFMessage, '/tf_static', static, LATCHED)
            def status(robot, message):
                try:
                    statuses[robot] = (time.monotonic(), json.loads(message.data))
                except ValueError:
                    pass
            def clicked(robot, backend, message):
                age = (node.get_clock().now().nanoseconds -
                       message.header.stamp.sec*1000000000-message.header.stamp.nanosec)*1e-9
                if not -.5 <= age <= 2.:
                    self.event.emit(robot, 'Veraltetes Klickziel verworfen; erneut in RViz setzen', False)
                    return
                self.clicked_goal.emit(robot, backend, message, time.monotonic())
            for robot in ROBOTS:
                ns = '/'+robot+'/navigation'
                clients[robot] = ActionClient(node, NavigateToPose, ns+'/navigate_to_pose')
                cancels[robot] = node.create_client(Trigger, ns+'/cancel')
                node.create_subscription(String, ns+'/status', lambda m, r=robot: status(r,m), LATCHED)
                node.create_subscription(PoseStamped, ns+'/goal_pose',
                    lambda m, r=robot: clicked(r, 'nav2', m), 10)
                node.create_subscription(PoseStamped, ns+'/mir_goal_pose',
                    lambda m, r=robot: clicked(r, 'mir', m), 10)
            def pose_message(pose):
                message = PoseStamped()
                message.header.frame_id = pose['frame']
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.position.x, message.pose.position.y = pose['x'], pose['y']
                message.pose.orientation.z = math.sin(pose['yaw']/2)
                message.pose.orientation.w = math.cos(pose['yaw']/2)
                return message
            def received(robot, generation, future):
                try:
                    handle = future.result()
                    if not handle.accepted:
                        raise RuntimeError('Ziel abgelehnt; Bereitschaft und Koordinatenrahmen prüfen')
                    handles[robot] = handle
                    if generation != generations.get(robot):
                        handle.cancel_goal_async()
                    self.event.emit(robot, 'Nav2 hat das Ziel angenommen', False)
                    def finished(result_future):
                        try:
                            result = result_future.result()
                            message = ('Ziel erreicht' if result.status == 4 and result.result.error_code == 0
                                       else result.result.error_msg or 'Fahrt beendet / abgebrochen')
                        except Exception as exc:
                            message = str(exc)
                        handles.pop(robot, None)
                        self.event.emit(robot, message, True)
                    handle.get_result_async().add_done_callback(finished)
                except Exception as exc:
                    self.event.emit(robot, str(exc), True)
            started = next_snapshot = time.monotonic()
            pending_sends = []
            while context.ok() and not self.stopping.is_set():
                executor.spin_once(timeout_sec=.02)
                while not self.commands.empty():
                    operation, robot, pose = self.commands.get_nowait()
                    if operation == 'cancel':
                        generations[robot] = generations.get(robot,0)+1
                        if robot in handles:
                            handles[robot].cancel_goal_async()
                        if cancels[robot].service_is_ready():
                            cancels[robot].call_async(Trigger.Request())
                        else:
                            self.event.emit(robot, 'Abbruchdienst nicht erreichbar; Stack stoppen', True)
                    elif operation == 'native':
                        try:
                            message = pose_message(pose)
                            t = transforms.lookup_transform('map', pose['frame'], rclpy.time.Time())
                            message = do_transform_pose_stamped(message,t)
                            message.header.stamp = node.get_clock().now().to_msg()
                            pub = publishers.get(robot)
                            if pub is None:
                                pub = node.create_publisher(PoseStamped, '/'+robot+'/move_base_simple/goal',10)
                                publishers[robot] = pub
                            # Wait for DDS matching before sending a single goal.
                            pending_sends.append((robot,pub,message,time.monotonic()))
                        except Exception as exc:
                            self.event.emit(robot, 'MiR-Ziel nicht gesendet: '+str(exc),True)
                    elif operation == 'goal':
                        if not clients[robot].server_is_ready():
                            self.event.emit(robot,'Nav2-Zielserver nicht erreichbar',True)
                            continue
                        generation = generations.get(robot,0)+1
                        generations[robot] = generation
                        future = clients[robot].send_goal_async(NavigateToPose.Goal(pose=pose_message(pose)))
                        future.add_done_callback(lambda f, r=robot, g=generation: received(r,g,f))
                for entry in list(pending_sends):
                    robot, pub, message, sent = entry
                    if pub.get_subscription_count():
                        pub.publish(message)
                        self.event.emit(robot,'Ziel nach map transformiert und an MiR gesendet',True)
                        pending_sends.remove(entry)
                    elif time.monotonic()-sent > 2:
                        self.event.emit(robot,'Kein Empfänger für das MiR-Ziel',True)
                        pending_sends.remove(entry)
                now = time.monotonic()
                if now >= next_snapshot:
                    self.snapshot.emit(dict(ready=now-started > 2.,
                        states={r:s for r,(t,s) in statuses.items() if now-t < 1.},
                        owners={r:node.count_publishers('/'+r+'/navigation/status') > 0 for r in ROBOTS}))
                    next_snapshot = now+.2
        except (KeyboardInterrupt,ExternalShutdownException):
            pass
        except Exception as exc:
            self.event.emit('', 'Navigationsmonitor: '+str(exc),True)
        finally:
            for handle in handles.values():
                handle.cancel_goal_async()
            if executor:
                executor.shutdown()
            for client in clients.values():
                client.destroy()
            if node:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)


class DrivingPanel(QtWidgets.QWidget):
    def __init__(self, module):
        super().__init__()
        self.module, self.context = module, module.context
        self.state = {}
        self.session_robot = None
        self.goal_pending = False
        self.rviz_selection = None
        self.profiles_path = Path(GUI_LOG_DIR).parent/'navigation_settings.yaml'
        self.profiles = {}
        try:
            loaded = yaml.safe_load(self.profiles_path.read_text())
            if isinstance(loaded, dict):
                self.profiles = loaded
        except (OSError,yaml.YAMLError):
            pass
        self.path = default_map()
        layout = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QFormLayout()
        self.backend = QtWidgets.QComboBox()
        self.backend.addItem('Interne MiR-Navigation','mir')
        self.backend.addItem('Eigener Stack · Nav2','nav2')
        self.robot = QtWidgets.QComboBox()
        self.source = QtWidgets.QComboBox()
        self.source.addItem('Mocap / Qualisys','mocap')
        self.source.addItem('Vorhandene AMCL-Lokalisierung','amcl')
        form.addRow('Zielbackend',self.backend)
        form.addRow('Navigationsroboter',self.robot)
        form.addRow('Nav2-Lokalisierung',self.source)
        layout.addLayout(form)
        self.map_label = QtWidgets.QLabel(self.path.name)
        self.map_label.setWordWrap(True)
        layout.addWidget(self.map_label)
        row = QtWidgets.QGridLayout()
        def button(text, action, r,c):
            item = QtWidgets.QPushButton(text)
            item.clicked.connect(action)
            row.addWidget(item,r,c)
            return item
        self.choose = button('Navigationskarte wählen',self.choose_map,0,0)
        self.start_button = button('Nav2 starten',self.start,0,1)
        self.stop_button = button('Nav2 stoppen',self.stop,1,1)
        self.cancel_button = button('Fahrt abbrechen',self.cancel,1,0)
        self.goals_button = button('MiR Goals öffnen',self.context.window.open_mir_goals,2,0)
        self.rviz_button = button('Navigation in RViz',self.open_rviz,2,1)
        layout.addLayout(row)
        self.status = QtWidgets.QLabel('Interne MiR-Navigation: Ziele wie bisher über MiR Goals senden.')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        click_hint = QtWidgets.QLabel('Klickziel: „Navigation in RViz“ öffnen, „2D Goal Pose“ wählen, '
            'Zielposition anklicken und bei gedrückter Maustaste die Ausrichtung ziehen. '
            'Loslassen sendet das Ziel an den gewählten MuR und das ausgewählte Backend.')
        click_hint.setWordWrap(True)
        layout.addWidget(click_hint)
        toggle = QtWidgets.QToolButton()
        toggle.setText('Navigationseinstellungen')
        toggle.setCheckable(True)
        toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        toggle.setArrowType(QtCore.Qt.RightArrow)
        layout.addWidget(toggle)
        self.settings_widget = QtWidgets.QWidget()
        settings_layout = QtWidgets.QFormLayout(self.settings_widget)
        self.fields = {}
        labels = [('max_speed','Max. Geschwindigkeit [m/s]'),('max_accel','Max. Beschleunigung [m/s²]'),
            ('max_decel','Max. Bremsverzögerung [m/s²]'),('max_angular_speed','Max. Drehrate [rad/s]'),
            ('max_angular_accel','Max. Winkelbeschleunigung [rad/s²]'),
            ('max_angular_decel','Max. Winkelverzögerung [rad/s²]'),
            ('xy_goal_tolerance','Positionstoleranz [m]'),('yaw_goal_tolerance','Winkeltoleranz [rad]'),
            ('inflation_radius','Hindernis-Inflationsradius [m]'),('cost_scaling_factor','Inflation: Kostenabfall'),
            ('obstacle_range','Hindernisreichweite [m]'),('raytrace_range','Freiraum-Raytracing [m]'),
            ('footprint_length','Footprint-Länge [m]'),('footprint_width','Footprint-Breite [m]'),
            ('footprint_padding','Zusätzlicher Sicherheitsabstand [m]'),
            ('max_reverse_speed','Max. Rückwärtsgeschwindigkeit [m/s]'),
            ('progress_distance','Minimaler Fortschritt [m]'),('progress_timeout','Fortschrittszeitlimit [s]'),
            ('goal_timeout','Zeitlimit pro Ziel [s]')]
        for key, label in labels:
            field = QtWidgets.QDoubleSpinBox()
            field.setDecimals(3)
            field.setRange(*BOUNDS[key])
            field.setValue(DEFAULTS[key])
            field.setSingleStep(.05 if 'timeout' not in key else 1.)
            self.fields[key] = field
            settings_layout.addRow(label,field)
        save = QtWidgets.QPushButton('Einstellungen für diesen MuR speichern')
        save.clicked.connect(self.save_settings)
        settings_layout.addRow(save)
        hint = QtWidgets.QLabel('Smac 2D (global) + MPPI (lokal). Einstellungen gelten beim nächsten Nav2-Start. '
            'Die interne MiR-Navigation wird im MiR-Webinterface eingestellt. '
            'Footprint muss MuR-Aufbauten und überstehende Lasten einschließen; Basismaß ist 1,35 × 0,91 m.')
        hint.setWordWrap(True)
        settings_layout.addRow(hint)
        self.settings_widget.hide()
        toggle.toggled.connect(self.settings_widget.setVisible)
        toggle.toggled.connect(lambda checked:toggle.setArrowType(QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow))
        layout.addWidget(self.settings_widget)
        layout.addStretch()
        self.worker = DrivingMonitor()
        self.worker.snapshot.connect(self.update_snapshot)
        self.worker.event.connect(self.handle_navigation_event)
        self.worker.clicked_goal.connect(self.receive_clicked_goal)
        self.worker.start()
        self.backend.currentIndexChanged.connect(self.selection_changed)
        self.robot.currentTextChanged.connect(self.load_profile)
        self.update_selection()

    def update_selection(self):
        if self.running() or self.goal_pending:
            return
        selected = self.robot.currentText()
        self.robot.blockSignals(True)
        self.robot.clear()
        self.robot.addItems(self.context.checked_robots())
        if selected in self.context.checked_robots():
            self.robot.setCurrentText(selected)
        self.robot.blockSignals(False)
        self.load_profile()

    def load_profile(self):
        try:
            settings = validate(self.profiles.get(self.robot.currentText(),{}))
        except ValueError:
            settings = DEFAULTS
        for key,field in self.fields.items():
            field.setValue(settings[key])
        self.selection_changed()

    def selection_changed(self):
        selection = (self.robot.currentText(), self.backend.currentData())
        if self.rviz_selection is not None and selection != self.rviz_selection:
            self.module.terminate_owned('navigation_driving_rviz')
            self.rviz_selection = None
        self.update_buttons()

    def settings(self):
        return validate({key:field.value() for key,field in self.fields.items()})

    def save_settings(self):
        if not self.robot.currentText():
            return
        try:
            self.profiles[self.robot.currentText()] = self.settings()
            self.profiles_path.parent.mkdir(parents=True,exist_ok=True)
            temporary = self.profiles_path.with_suffix('.tmp')
            temporary.write_text(yaml.safe_dump(self.profiles))
            temporary.replace(self.profiles_path)
            self.status.setText('Navigationseinstellungen gespeichert; wirksam beim nächsten Nav2-Start.')
        except (OSError,ValueError) as exc:
            self.status.setText(str(exc))

    def running(self):
        return self.module.running('navigation_driving')

    def choose_map(self):
        path,_ = QtWidgets.QFileDialog.getOpenFileName(self,'Navigationskarte wählen',str(self.path.parent),'Karte (*.yaml)')
        if path:
            try:
                self.path,_ = map_metadata(path)
                self.map_label.setText(self.path.name)
            except (OSError,ValueError,yaml.YAMLError) as exc:
                self.status.setText(str(exc))

    def start(self):
        robot = self.robot.currentText()
        if self.running() or self.backend.currentData() != 'nav2' or not robot:
            return
        if self.state.get('owners',{}).get(robot):
            self.status.setText('Externer Nav2-Stack läuft bereits; dort beenden, bevor diese GUI übernimmt.')
            return
        try:
            settings = self.settings()
            path,_ = map_metadata(self.path)
        except (OSError,ValueError,yaml.YAMLError) as exc:
            self.status.setText(str(exc))
            return
        if self.source.currentData() == 'mocap':
            self.module.start_mocap()
        else:
            self.context.append_log('[navigation] AMCL muss im Tab Lokalisierung auf derselben Karte laufen.')
        self.session_robot = robot
        directory = Path(self.module.temp.name)
        parameters = directory/'navigation_settings.yaml'
        parameters.write_text(yaml.safe_dump(settings))
        arguments = ['ros2','launch','match_mur_navigation','navigation.launch.py',
            'robot:='+robot,'map:='+str(path),'localization_source:='+self.source.currentData(),
            'params_file:='+str(parameters),'output_directory:='+str(directory/'nav2')]
        self.context.start_process('navigation_driving', setup_prefix()+'exec '+shlex.join(arguments),
                                   on_finished=self.finished)
        self.status.setText('Nav2 startet; Fahrbefehle erst nach gültigem Ziel und vollständigen Eingangsdaten.')
        QtCore.QTimer.singleShot(200,self.update_buttons)

    def stop(self):
        if not self.running():
            return
        if self.session_robot:
            self.worker.request('cancel',self.session_robot)
        self.module.terminate_owned('navigation_driving_rviz')
        self.module.terminate_owned('navigation_driving')
        self.goal_pending = False
        self.update_buttons()

    def cancel(self):
        robot = self.session_robot or self.robot.currentText()
        if self.backend.currentData() == 'nav2':
            if robot:
                self.worker.request('cancel',robot)
                self.status.setText('Fahrtabbruch angefordert …')
        else:
            ok,message = self.context.window.ros_worker.cancel_mir_navigation(robot)
            self.status.setText(message)
            self.context.append_log('[navigation] '+message)

    def send(self, namespace, frame, x,y,angle):
        robot = namespace.strip('/')
        if self.backend.currentData() == 'mir':
            if frame == 'map':
                return None
            if robot not in ROBOTS:
                return False,'Für die Rahmentransformation einen bekannten MuR wählen'
            self.worker.request('native',robot,dict(frame=frame,x=float(x),y=float(y),yaw=float(angle)))
            return True,'MiR-Ziel zur Transformation nach map angefordert'
        if not self.running() or robot != self.session_robot:
            return False,'Nav2 für diesen MuR zuerst im Navigationstab starten'
        status = self.state.get('states',{}).get(robot,{})
        if self.goal_pending or status.get('busy'):
            return False,'Aktuelle Fahrt zuerst abbrechen und deren Ende abwarten'
        if not status.get('ready'):
            return False,status.get('state','Nav2 noch nicht bereit')
        self.goal_pending = True
        self.worker.request('goal',robot,dict(frame=frame,x=float(x),y=float(y),yaw=float(angle)))
        self.status.setText('Ziel an Nav2 gesendet …')
        self.update_buttons()
        return True,'Nav2-Ziel angefordert'

    def receive_clicked_goal(self, robot, backend, message, received_at):
        # A stale RViz window must not redirect goals to another robot/backend.
        if robot != self.robot.currentText() or backend != self.backend.currentData():
            self.context.append_log('[navigation] Klickziel für einen anderen MuR / ein anderes Backend verworfen.')
            return
        if time.monotonic()-received_at > 2.:
            self.handle_navigation_event(robot, 'Verzögertes Klickziel verworfen; erneut in RViz setzen', False)
            return
        try:
            pose = clicked_pose_values(message)
        except ValueError as exc:
            self.handle_navigation_event(robot, str(exc), False)
            return
        result = self.send(robot, pose['frame'], pose['x'], pose['y'], pose['yaw'])
        if result is None:
            # Native map goals also wait for DDS discovery in the worker.
            self.worker.request('native', robot, pose)
            result = True, 'Klickziel für die interne MiR-Navigation angefordert'
        self.handle_navigation_event(robot, result[1], False)

    def handle_navigation_event(self,robot,message,finished):
        self.context.append_log('[navigation] '+(robot+': ' if robot else '')+message)
        if not robot or robot == self.session_robot or self.backend.currentData() == 'mir':
            self.status.setText(message)
        if finished and robot == self.session_robot:
            self.goal_pending = False
        self.update_buttons()

    def finished(self,code,_status):
        self.goal_pending = False
        self.status.setText(f'Nav2 beendet (Code {code}).')
        self.update_selection()

    def update_snapshot(self,state):
        self.state = state
        data = state.get('states',{}).get(self.session_robot or self.robot.currentText())
        if self.running() and data:
            distance = data.get('distance_remaining')
            self.status.setText(data['state']+(f' · Restweg {distance:.2f} m' if distance is not None else ''))
        self.update_buttons()

    def update_buttons(self):
        own = self.running()
        nav = self.backend.currentData() == 'nav2'
        busy = self.goal_pending or self.state.get('states',{}).get(self.session_robot,{}).get('busy',False)
        for widget in (self.backend,self.robot,self.source,self.choose,self.settings_widget):
            widget.setEnabled(not own and not busy)
        self.source.setEnabled(nav and not own and not busy)
        self.choose.setEnabled(nav and not own and not busy)
        self.settings_widget.setEnabled(nav and not own and not busy)
        self.start_button.setEnabled(nav and not own and not busy and bool(self.robot.currentText()) and self.state.get('ready',False))
        self.stop_button.setEnabled(own)
        self.rviz_button.setEnabled(bool(self.robot.currentText()) and (own if nav else True))
        self.cancel_button.setEnabled(bool(self.robot.currentText()) and (own if nav else True))

    def open_rviz(self):
        nav = self.backend.currentData() == 'nav2'
        robot = self.session_robot if nav else self.robot.currentText()
        if robot and (self.running() or not nav) and not self.module.running('navigation_driving_rviz'):
            self.rviz_selection = (robot, self.backend.currentData())
            self.context.start_process('navigation_driving_rviz',self.module.command('match_mur_navigation.rviz',
                ['--robot',robot,'--mode','navigation' if nav else 'mir']))

    def description(self):
        return ('Zielbackend: interne MiR-Navigation' if self.backend.currentData() == 'mir' else
                'Zielbackend: Nav2 · Smac + MPPI · '+(self.session_robot or self.robot.currentText() or 'kein MuR'))

    def shutdown(self):
        self.worker.shutdown()
        self.worker.wait(3000)
