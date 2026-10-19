"""Saved map view and controls for an independent AMCL experiment."""
from pathlib import Path
import shlex
from PyQt5 import QtCore, QtGui, QtWidgets
import yaml
from match_mur_gui.base_gui import GUI_LOG_DIR, setup_prefix
from .saved_map import default_map, map_metadata
from .settings import ROBOTS


class MapView(QtWidgets.QGraphicsView):
    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.scene():
            self.fitInView(self.scene().itemsBoundingRect(), QtCore.Qt.KeepAspectRatio)


class LocalizationPanel(QtWidgets.QWidget):
    def __init__(self, module):
        super().__init__()
        self.module = module
        self.context = module.context
        self.snapshot = {}
        self.started = False
        self.localizing = False
        self.using_external = False
        self.autoload = True
        self.path = default_map()
        self.output_directory = str(Path(GUI_LOG_DIR).parent / 'localization')
        layout = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QFormLayout()
        self.robot = QtWidgets.QComboBox()
        self.robot.addItems(ROBOTS)
        self.robot.setCurrentText('mur620c')
        self.path_label = QtWidgets.QLabel(self.path.name)
        self.path_label.setWordWrap(True)
        form.addRow('Lokalisierungsroboter', self.robot)
        form.addRow('Standardkarte', self.path_label)
        layout.addLayout(form)
        self.view = MapView()
        self.scene = QtWidgets.QGraphicsScene(self.view)
        self.view.setScene(self.scene)
        self.view.setMinimumHeight(140)
        self.view.setMaximumHeight(190)
        layout.addWidget(self.view, 1)
        buttons = QtWidgets.QGridLayout()
        def button(text, callback, row, col):
            result = QtWidgets.QPushButton(text)
            result.clicked.connect(callback)
            buttons.addWidget(result, row, col)
            return result
        self.choose_button = button('Andere Karte wählen', self.choose_map, 0, 0)
        self.load_button = button('Karte laden', self.load_map, 0, 1)
        self.start_button = button('AMCL starten · Startpose aus Mocap', self.start_amcl, 1, 0)
        self.stop_button = button('AMCL / Kartenserver stoppen', self.stop, 1, 1)
        self.rviz_button = button('Karte und Posen in RViz', self.open_rviz, 2, 0)
        layout.addLayout(buttons)
        self.stationary = QtWidgets.QCheckBox('Standmessung: AMCL auch ohne Bewegung aktualisieren (1 Hz)')
        self.stationary.setChecked(True)
        self.stationary.toggled.connect(self.change_stationary)
        layout.addWidget(self.stationary)
        hint = QtWidgets.QLabel('AMCL wird hier separat verglichen. Startpose einmal aus Mocap; '
            'anschließend Laser + Encoder-Odometrie. Die laufende Robotersteuerung bleibt unverändert. '
            'Die ersten 5 s werden nicht ausgewertet. Für Fahrgenauigkeit eine Messfahrt aufnehmen.')
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.status = QtWidgets.QLabel('Gespeicherte Karte wird geladen …')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.metrics = QtWidgets.QLabel('Noch keine AMCL-Vergleichsdaten.')
        self.metrics.setWordWrap(True)
        layout.addWidget(self.metrics)
        self.robot.currentTextChanged.connect(self.selection_changed)
        self.show_map()
        self.update_buttons()

    def show_map(self):
        try:
            self.path, _ = map_metadata(self.path)
            document = yaml.safe_load(self.path.read_text(encoding='utf-8'))
            image = QtGui.QPixmap(str(self.path.parent / document['image']))
            if image.isNull():
                raise ValueError('Kartenbild konnte nicht angezeigt werden')
            self.scene.clear()
            self.scene.addPixmap(image)
            self.view.setSceneRect(self.scene.itemsBoundingRect())
            self.view.fitInView(self.scene.itemsBoundingRect(), QtCore.Qt.KeepAspectRatio)
            self.path_label.setText(self.path.name)
            self.path_label.setToolTip(str(self.path))
        except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
            self.status.setText('Karte nicht geladen: '+str(exc))

    def selection_changed(self):
        self.using_external = False
        self.show_map()
        self.metrics.setText('Noch keine AMCL-Vergleichsdaten für diesen MuR.')
        self.autoload = True
        self.update_buttons()

    def choose_map(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Karte laden', str(self.path.parent),
                                                          'Karten-YAML (*.yaml)')
        if filename:
            try:
                path, _ = map_metadata(filename)
            except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
                self.status.setText(str(exc))
                return
            self.path = path
            self.show_map()
            self.load_map()

    def update_buttons(self):
        own = self.started or self.module.running('navigation_localization')
        external = self.snapshot.get('localization_owners', {}).get(self.robot.currentText(), False) and not own
        backend = self.snapshot.get('localization_backends', {}).get(self.robot.currentText())
        map_only = external and backend and backend['map'] and not backend['amcl'] and not backend['adapter'] and self.external_map_matches()
        ready = self.snapshot.get('discovery_ready', False)
        self.start_button.setEnabled(ready and not self.localizing and (not external or bool(map_only)))
        self.load_button.setEnabled(ready and not self.localizing and not external)
        self.choose_button.setEnabled(not own and not external)
        self.robot.setEnabled(not own)
        self.stationary.setEnabled(not external)
        self.stop_button.setEnabled(own)
        self.rviz_button.setEnabled(bool(own or external))

    def external_map_matches(self):
        path = self.snapshot.get('localization_map_paths', {}).get(self.robot.currentText())
        return bool(path and Path(path).resolve() == self.path.resolve())

    def launch(self, localize):
        robot = self.robot.currentText()
        external = self.snapshot.get('localization_owners', {}).get(robot, False) and not (
            self.started or self.module.running('navigation_localization'))
        backend = self.snapshot.get('localization_backends', {}).get(robot)
        reuse_map = bool(external and localize and backend and backend['map']
                        and not backend['amcl'] and not backend['adapter'] and self.external_map_matches())
        if external and not reuse_map:
            self.status.setText('Vorhandener externer Kartenserver / AMCL wird angezeigt. '
                                'Zum Neustart in der zugehörigen Anwendung beenden.')
            return
        try:
            path, _ = map_metadata(self.path)
        except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
            self.status.setText(str(exc))
            return
        if localize:
            # Start the receive-only bridge only after this attempt passed the
            # map and ownership checks; an external experiment stays untouched.
            self.module.start_mocap()
        self.module.terminate_owned('navigation_localization_rviz')
        self.module.terminate_owned('navigation_localization')
        self.started = True
        self.using_external = False
        self.localizing = localize
        self.autoload = False
        arguments = ['ros2', 'launch', 'match_mur_navigation', 'localization.launch.py',
            f'robot:={robot}', f'map:={path}', f'localize:={str(localize).lower()}',
            f'load_map:={str(not reuse_map).lower()}',
            f'stationary_updates:={str(self.stationary.isChecked()).lower()}',
            f'output_directory:={self.output_directory}']
        self.context.start_process('navigation_localization', setup_prefix()+'exec '+shlex.join(arguments),
                                   on_finished=self.finished)
        self.status.setText('AMCL startet; warte auf Mocap, Encoder-Odometrie und Frontlaser …' if localize
                            else 'Kartenserver startet …')
        self.update_buttons()

    def load_map(self):
        self.launch(False)

    def start_amcl(self):
        self.launch(True)

    def change_stationary(self, enabled):
        if self.localizing:
            self.module.monitor.request_stationary(self.robot.currentText(), enabled)

    def stop(self):
        self.autoload = False
        self.using_external = False
        self.module.terminate_owned('navigation_localization_rviz')
        self.module.terminate_owned('navigation_localization')
        self.started = self.localizing = False
        self.status.setText('AMCL / Kartenserver gestoppt. Messdateien bleiben gespeichert.')
        self.update_buttons()

    def finished(self, code, _status):
        self.started = self.localizing = False
        self.status.setText(f'Lokalisierungsprozess beendet (Code {code}); Details im Prozesslog.')
        self.update_buttons()

    def update_snapshot(self, snapshot):
        self.snapshot = snapshot
        robot = self.robot.currentText()
        if self.autoload and snapshot.get('discovery_ready'):
            self.autoload = False
            if not snapshot.get('localization_owners', {}).get(robot):
                self.load_map()
            else:
                self.using_external = True
                self.scene.clear()
                self.path_label.setText('Externe Karte · Anzeige in RViz')
                self.status.setText('Vorhandener externer Kartenserver / AMCL wird verwendet.')
        owner = snapshot.get('localization_owners', {}).get(robot)
        if owner and (self.started or self.using_external) and snapshot.get('localization_maps', {}).get(robot) and not self.localizing:
            self.status.setText('Karte geladen · Frame mocap · AMCL kann separat gestartet werden.')
        data = snapshot.get('comparisons', {}).get(robot)
        if data and (self.localizing or (owner and self.using_external)):
            self.stationary.blockSignals(True)
            self.stationary.setChecked(data['stationary_updates'])
            self.stationary.blockSignals(False)
            live = data['live']
            missing = [label for key, label in [('encoder_odom', 'Encoder-Odometrie'),
                       ('front_scan', 'Frontlaser'), ('mocap', 'Mocap')] if not live.get(key)]
            if missing:
                self.status.setText('Warte auf: '+', '.join(missing))
            elif not data['initialized_from_mocap_once']:
                self.status.setText('Warte auf AMCL-Aktivierung und einmalige Startpose aus Mocap …')
            elif not live.get('amcl'):
                self.status.setText('Keine aktuelle AMCL-Transformation; Eingangsdaten und TF prüfen.'
                    if data.get('amcl_pose_available') else
                    'Startpose gesendet; warte auf AMCL-Pose (Scanner-TF im Prozesslog prüfen).')
            elif not data['stationary_updates'] and not live.get('amcl_updates', live.get('amcl')):
                self.status.setText('AMCL lokalisiert · keine neuen Filterupdates im Stand. '
                                    'Für Messwerte „Standmessung“ aktivieren.')
            else:
                self.status.setText('AMCL unabhängig aktiv · Vergleich mit Mocap läuft')
            if not data['samples']:
                self.metrics.setText('Noch keine Messproben nach der Einlaufzeit.' +
                    (' Im Stand „Standmessung“ aktivieren; ohne diese Option aktualisiert AMCL erst bei Bewegung.'
                     if not data['stationary_updates'] else ''))
            if data['samples']:
                p, a = data['position_m'], data['yaw_deg']
                self.metrics.setText(f"{data['samples']} Posen · Position RMSE {p['rmse']*100:.1f} cm, "
                    f"P95 {p['p95']*100:.1f} cm · Gierwinkel RMSE {a['rmse']:.2f}° · "
                    f"Beobachtete Positionsspanne {data['reference_span_m']:.2f} m\n"
                    + ('Standmessung: korrelierte Stichproben; keine Aussage über Fahrgenauigkeit.\n'
                       if data['stationary_updates'] else '') + 'Dateien: '+data['output']+'.{csv,json}')
        backend = snapshot.get('localization_backends', {}).get(robot)
        if backend and not backend['amcl'] and not self.localizing:
            if backend['adapter']:
                self.status.setText('AMCL-Prozess fehlt; ein Lokalisierungsadapter aus einer älteren '
                                    'Sitzung läuft noch. Diese Sitzung auf ihrem Rechner beenden.')
            elif backend['map']:
                self.status.setText('Karte geladen · AMCL ist nicht gestartet. ' +
                    ('AMCL kann mit diesem Kartenserver gestartet werden.' if self.external_map_matches()
                     else 'Für AMCL muss die gewählte Karte der Karte des externen Servers entsprechen.'))
        self.update_buttons()

    def open_rviz(self):
        if not self.module.running('navigation_localization_rviz'):
            self.context.start_process('navigation_localization_rviz',
                self.module.command('match_mur_navigation.rviz',
                                    ['--robot', self.robot.currentText(), '--mode', 'localization']))
