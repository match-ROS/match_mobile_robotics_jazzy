# MuR Navigation und Kartierung

**Required: configure your workspace path in `~/.bashrc` before using these commands.**
Replace the example directory below with the actual name and location of your
workspace. `WS` must point to the workspace root containing `src/`, not to a
repository inside `src/`. There is no automatic workspace discovery.

```bash
# Add these lines to ~/.bashrc; adjust the workspace folder to match your setup.
export WS="$HOME/your_workspace_folder"
source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
```

Then reload and verify the configured path:

```bash
source ~/.bashrc
printf 'Workspace: %s\n' "$WS"
test -d "$WS/src" && echo "Workspace src directory found" || echo "Fix WS in ~/.bashrc"
```

Check that the printed path matches your actual workspace folder name. Update
`WS` whenever you move or rename the workspace. Remote workspaces can differ;
set `REMOTE_WS` or the GUI's Remote WS field to the path on the remote computer.


Die Anwendung erweitert `MurBaseGui` um den Tab **Kartierung**. Hardwarestart,
MiR-Jog und die übrigen Basisfunktionen bleiben in der gemeinsamen Oberfläche.
Eine Sitzung verwendet genau einen ausdrücklich markierten MuR. Die GUI und
Kartierung laufen auf dem Bedienrechner; dessen ROS-Domain muss der Hardware
entsprechen (GUI-Standard: 62).

## Bauen und starten

```bash
source /opt/ros/jazzy/setup.bash
cd "$WS"
rosdep install --from-paths src/match_mobile_robotics_jazzy/navigation/match_mur_navigation --ignore-src -r -y
colcon build --packages-select match_mur_navigation --symlink-install
source install/setup.bash
ros2 run match_mur_navigation navigation_gui
```

`match_mur_gui`, `match_mocap_ros2` und die Hardwarepakete müssen bereits im
Workspace verfügbar sein. ROS-Abhängigkeiten sind unter anderem `slam_toolbox`,
`nav2_map_server` und `rviz2`. Ein laufender Roboter muss seine lokalen TFs und
`/<robot>/robot_description` bereitstellen; im SLAM-Modus außerdem
`/<robot>/odom` und den zusammengeführten `/<robot>/scan`.

Für den Startmenüeintrag **MuR Navigation** mit passendem Fenster- und Karten-Icon:

```bash
python3 "$WS"/src/match_mur_gui/scripts/install_gui_desktop.py \
  --app navigation --no-desktop-shortcut
```

Ohne `--no-desktop-shortcut` wird zusätzlich eine Verknüpfung auf dem Desktop angelegt.

1. MuR in der Basis-GUI markieren und Hardware starten beziehungsweise verbinden.
2. Im Kartierungstab den Kartierungsroboter und die Betriebsart auswählen.
3. Für Mocap **Mocap starten** verwenden oder eine laufende Qualisys-Bridge nutzen.
4. Bei Bedarf Einstellungen aufklappen und **RViz beim Kartierungsstart öffnen** wählen.
5. **Kartierung starten**, den MuR mit den vorhandenen Bedienelementen bewegen,
   dann **Kartierung stoppen** und **Karte speichern**.

Stoppen beendet die Aufnahme, lässt das Backend zur Bereitstellung der letzten
Karte aber laufen. Eine neue Sitzung beendet ausschließlich die von dieser GUI
gestartete alte Sitzung. Die GUI behält zusätzlich einen eigenen Kartencache;
auch nach einem Backendabsturz kann die zuletzt empfangene Karte gespeichert
werden. Beim Ersetzen einer ungespeicherten Karte wird nachgefragt.

**Kartierung beenden** beendet zusätzlich den Kartierungsprozess und gibt den
ausgewählten MuR für eine neue Sitzung frei. Wird eine Sitzung aus einer anderen
GUI oder vom CLI erkannt, ist der Button als **Externe Kartierung beenden**
verfügbar. Eine ungespeicherte externe Karte vorher in der ursprünglichen GUI
speichern. Beendet wird ausschließlich die Kartierung dieses MuR; Hardware,
Fahrsteuerung und andere Roboter bleiben aktiv. Der Start ist erst wieder möglich,
wenn die Kartierungs-Backends aus der ROS-Erkennung verschwunden sind.

Neue Backends bieten dafür `/<robot>/mapping/end_session` an. Ältere, bereits
laufende Sitzungen auf dem Bedienrechner werden anhand ihrer exakten Prozessargumente
und des Roboter-Namens erkannt und regulär per SIGINT beendet. Der Fallback beendet
keine fremden Benutzerprozesse und erzwingt kein SIGKILL. Alte Backends auf einem
anderen Rechner müssen dort beendet oder auf den neuen Dienst aktualisiert werden.

**Mocap stoppen** beendet ausschließlich die GUI-eigene Bridge. Vorhandene
Bridges werden erkannt und verwendet; deren Einstellungen werden nicht verändert.
Die GUI-eigene Bridge läuft mit `publish_map_pose:=false` und
`publish_robot_tf:=false`. Es erfolgt kein automatischer Bridge-Start beim
Hardwarestart, damit die bisherige Hardwarelokalisierung unverändert bleibt.

## Betriebsarten und Koordinaten

**Mocap-Kartierung** verwendet ungemittelte `PoseStamped` auf
`/qualisys/<robot>/pose`, Frame `mocap`, und die getrennten LaserScans
`/<robot>/f_scan` und `/<robot>/b_scan`. Der Qualisys-Starrkörper entspricht laut
vorhandener MuR-Kalibrierung dem `base_link`; `base_footprint -> base_link` ist
identisch. Position und Quaternion werden zwischen zwei zeitlich passenden
Messungen interpoliert (SLERP für die Orientierung). Die vollständige 3D-Pose
und Scannerextrinsik werden vor der Projektion in die XY-Karte kombiniert.
Der Mapper extrapoliert nicht und nutzt niemals MiR-Posen als Ersatz.

Freie Zellen entstehen durch Raytracing, belegte durch gültige Endpunkte. Ein
Scan aktualisiert jede Zelle höchstens einmal; Endpunkte haben Vorrang vor
durchlaufenden Strahlen. Unbeobachtete Zellen bleiben unbekannt. Positive
Unendlich-Messungen und an der konfigurierten Reichweite abgeschnittene Strahlen
räumen freie Fläche, erzeugen aber keinen künstlichen Hindernisring. NaN,
negative/unterschrittene und ungültig über der Sensorgrenze liegende Werte
werden ignoriert. Log-Odds sind begrenzt; die Karte wächst rastertreu auch in
negative Koordinaten. Das Standardlimit von 25 Millionen Zellen verhindert
unbegrenztes Wachstum; abgelehnte Scans erscheinen im Status.

Fehlende Mocap-Posen, größere Posenlücken oder fehlende Scanner-TFs lassen
Scans bis zu 0,5 s auf passende Daten warten. Danach werden sie verworfen.
Mit gültigen Daten wird automatisch fortgesetzt. Die letzte Karte bleibt erhalten.

**SLAM ohne Mocap** verwendet den Jazzy-`async_slam_toolbox_node` mit
Scan-Matching und Schleifenschlüssen. Odometrie kommt aus `/<robot>/odom`,
der Scan aus `/<robot>/scan`. Die Aufnahme wird über die Weiterleitung der
Scans gesteuert. Beim Stoppen können bereits angenommene Scans und die letzte
Kartenaktualisierung noch fertig verarbeitet werden. Mocap-Zähler gelten nur
für den Mocap-Modus; im SLAM-Modus stehen dessen Meldungen im Prozesslog.
SLAM-Karten besitzen keinen zugesicherten Mocap-Bezug.

Beide Modi lesen globale TFs, veröffentlichen jedoch ausschließlich auf
`/<robot>/mapping/tf` und `tf_static`. Es werden nur roboterinterne Kanten
weitergeleitet. Die Wurzelpose wird für die Sitzung aus Mocap beziehungsweise
Odometrie aufgebaut; SLAM ergänzt seine eigene Map-Odom-Kante. Hardware-TFs und
deren Lokalisierung werden nicht verändert. RViz liest diese privaten Topics
und das Robotermodell des gewählten MuR.

## Einstellungen und CLI

Die GUI erstellt pro Sitzung eine Parameterdatei. CLI-Sitzungen können
`config/mapping.yaml` kopieren und anpassen:

```bash
ros2 launch match_mur_navigation mapping.launch.py \
  robot:=mur620d mode:=mocap use_rviz:=true use_sim_time:=false
# Alternative: mode:=slam; eigene Datei: params_file:=/absolut/mapping.yaml
ros2 service call /mur620d/mapping/set_recording std_srvs/srv/SetBool '{data: false}'
```

Eine gestoppte Sitzung wird nicht wieder aufgenommen; eine neue Sitzung startet
eine neue Karte. Externe CLI-Sitzungen können über **Externe Kartierung beenden**
freigegeben werden.

| Parameter | Standard | Bedeutung |
| --- | --- | --- |
| `resolution` | 0,02 m | Rasterauflösung (0,005–0,2 m) |
| `min_range`, `max_range` | 0,1 / 10 m | zusätzlich auf Sensorgrenzen beschränkt |
| `map_update_interval` | 1 s | Veröffentlichung der vollständigen Karte |
| `mocap_time_offset` | 0 s | zum Mocap-Zeitstempel addieren |
| `front_time_offset`, `rear_time_offset` | 0 s | zum jeweiligen Scan-Zeitstempel addieren |
| `pose_timeout` | 0,5 s | maximale Nachrichtenalterung und Scanwartezeit |
| `max_pose_gap` | 0,1 s | größte zulässige Lücke für Poseninterpolation |
| `max_cells` | 25000000 | Größenlimit der Mocap-Karte |

Zeitkorrekturen sind auf ±1 s begrenzt und betreffen die Mocap-Kartierung.
Mocap-Posen werden vollständig um ihren Offset verschoben. Scannerzeiten werden
um den jeweiligen Offset und pro Strahl um `i * time_increment` verschoben.
Ist `time_increment` null, gilt dieselbe Pose für alle Strahlen. Die vorhandene
MiR-Bridge und die Qualisys-Bridge stempeln beim Empfang auf dem ROS-Rechner;
die ursprüngliche Aufnahmezeit ist daraus nicht rekonstruierbar. Eine konstante
Korrektur kompensiert keine schwankende Transportlatenz. Kleine Rasterzellen
garantieren deshalb keine entsprechende Messgenauigkeit.

## ROS-Schnittstellen und Export

Alle sitzungsbezogenen Topics liegen unter `/<robot>/mapping/`:

| Name | Typ / Funktion |
| --- | --- |
| `map` | `nav_msgs/OccupancyGrid`, reliable/transient-local |
| `status` | `std_msgs/String`, JSON mit Modus, Aufnahmezustand, Eingangsstatus und Mocap-Scanzählern |
| `set_recording` | `std_srvs/SetBool`, `false` beendet die Aufnahme |
| `end_session` | `std_srvs/Trigger`, beendet die Sitzung samt Kartierungsbackend |
| `pose`, `robot_pose` | zeitkorrigierte Mocap-Pose / RViz-Anzeige |
| `front_scan`, `rear_scan`, `scan` | sitzungsbezogene Scans |
| `tf`, `tf_static` | privater TF-Datenstrom |

Frame der Karte: `mocap` im Mocap-Modus, `<robot>/mapping/map` bei SLAM.
Es wird keine bestehende `map -> mocap`-Kalibrierung benötigt oder geändert.

Der Export hält genau eine empfangene Karte als Snapshot fest und übergibt sie
auf einem temporären Topic an Nav2 `map_saver_cli`. Erst nach erfolgreichem
Export und Geometrieprüfung werden `<name>.pgm`, `<name>.yaml` und
`<name>.mapping.yaml` am gewählten Ziel abgelegt. Die Begleitdatei dokumentiert
Frame, Modus, Roboter, Snapshot-Zeit und Einstellungen. Die PGM-Datei ist eine
trinäre Nav2-Karte (frei ≤0,25, belegt ≥0,65, dazwischen unbekannt).

```bash
ros2 run match_mur_navigation save_mapping \
  --session /pfad/session.yaml --output /pfad/labor
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/pfad/labor.yaml -p frame_id:=mocap
# Den Map Server anschließend wie üblich konfigurieren und aktivieren.
```

Die Session-Datei enthält `robot`, `mode` und `settings` wie in der GUI erzeugt.
`--overwrite` erlaubt ausdrücklich das Ersetzen vorhandener Exportdateien.
Nav2-Karten-YAML speichert keinen ROS-Frame: Beim späteren Laden ist
`frame_id:=mocap` für diese Karten erforderlich. Eine spätere Navigation muss
Lokalisierung und Karte auf denselben Frame konfigurieren; sie darf die
Mocap-Karte nicht stillschweigend als bisherige MiR-`map` interpretieren.

## Tests und Hardware-Abnahme

```bash
colcon test --packages-select match_mur_navigation
colcon test-result --verbose
```

Die Tests prüfen Rasterberechnung, Poseninterpolation, Sensor-/TF-Weiterleitung,
Aufnahmestopp, GUI-Auswahl und Export mit Nav2. Für die reale Abnahme eine
gerade Wand von mehreren Positionen und Fahrtrichtungen sowie nach einer
Rundfahrt erfassen. Auf Doppelkonturen und richtungsabhängigen Versatz achten,
Scannerextrinsik und Zeitkorrekturen prüfen und einen kurzen Trackingausfall
provozieren. Die Karte muss erhalten bleiben und anschließend ohne Streifen
aus veralteten Posen weiterwachsen. Beide Modi separat prüfen und gespeicherte
Karten wieder laden. Diese Hardware-Abnahme ist nicht durch lokale Tests ersetzt.
