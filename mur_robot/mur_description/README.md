# Kalibrierte Markerframes

Die MuR620-Beschreibung lädt `config/calibrated_markers.yaml` und ergänzt für
den tatsächlich übergebenen `robot_namespace` feste Markerlinks unter
`base_link`. Die Hardware- und Simulationslaunches übergeben diesen Namen.
`robot_state_publisher` ergänzt anschließend genau einmal den Roboterpräfix.
Andere MuRs übernehmen keine Transformationen der MuR620a.

Die Konfiguration enthält beide Markerpaare der MuR620a:

| ID | TF-Kindframe | Translation relativ zu `mur620a/base_link` [m] | Veröffentlichung |
| --- | --- | --- | --- |
| 0 | `mur620a/aruco_rear_left` | −0.68037554, +0.24727121, +0.58000000 | aktiv |
| 1 | `mur620a/aruco_rear_right` | −0.67159335, −0.24572920, +0.58561780 | aktiv |
| 24 | `mur620a/aruco_front_left` | +0.44839220, +0.30413303, +0.43494018 | Entwurf, deaktiviert |
| 7 | `mur620a/aruco_front_right` | +0.45886989, −0.36577286, +0.44000000 | Entwurf, deaktiviert |

Dictionary: `DICT_APRILTAG_36h11`; schwarze Außenkante: 0,16 m. Die YAML hält
die vollständigen Translationen und Quaternionen (`xyzw`) sowie die daraus
berechneten URDF-Eulerwinkel in Radiant fest. Die Transformationen bilden
Markerkoordinaten auf `base_link`-Koordinaten ab. Der Ursprung liegt in der
Markermitte; die Achsen folgen der OpenCV-Quadratkonvention der Kalibrierung,
einschließlich der tatsächlichen Druckausrichtung.

Hinteres Paar: Sitzung `20261005T132811Z_a702dd6a`, 14 akzeptierte Messposen. Das
Ergebnis bleibt als **draft** dokumentiert: Die zurückgehaltenen Messposen
ergeben einen 95%-Eckpunktfehler von ca. 3,87 Pixeln. Die feste Referenzhöhe
des linken Markers beträgt 0,58 m. Vor einer Nutzung zur Lokalisierung ist die
unabhängige Prüfung dieser Genauigkeit erforderlich.

Die Frames erscheinen nach dem nächsten Start des zuständigen
`robot_state_publisher` mit der neu gebauten Beschreibung. Ein bereits
laufender Publisher lädt geänderte Xacro-Dateien nicht selbstständig nach.
Es wird kein zusätzlicher statischer Publisher parallel zur Hardware gestartet.

## Vorderes Paar: noch nicht validierter Entwurf

Quelle: Sitzung `20261005T141627Z_a2cfeb5e`, 20 akzeptierte Messposen mit
200 Originalbildern. Die verbesserte Erkennung (Detektor v2) erhöht die Zahl
verwendbarer Markerfunde von 47 auf 114. Davon liefern 16 verschiedene Posen
Beobachtungen für die gemeinsame Kamera-/Markeroptimierung. Ganze Messposen
`000006`, `000012`, `000022` werden für die unabhängige Validierung zurückgehalten.

Der Fit konvergiert mit Rang 23/23, besteht aber die Validierung **nicht**:
RMS-Eckpunktfehler **5,76 px**, 95%-Fehler **8,33 px** bei einer Grenze von
3 px. Eine zweite Rechnung mit acht Starts und den bisherigen Kamera-
transformationen als Startwert bestätigt das Ergebnis. Es wurden keine
Messposen anhand ihrer Reprojektionsfehler entfernt.

Der vollständige Export einschließlich beider Kamera-Optical-Transformationen,
Quaternionen, Original-Labelzuordnung und Qualitätsbericht liegt unter
[config/calibration_results/mur620a_front_20261005T141627Z.yaml](config/calibration_results/mur620a_front_20261005T141627Z.yaml).
Die Markertransformationen sind zusätzlich in `calibrated_markers.yaml`
hinterlegt. **`publish_tf: false`** verhindert ihre automatische Verwendung
im TF-Baum. Diese Fronttransformationen sind derzeit nicht für Lokalisierung
freigegeben. Für eine ausdrückliche geometrische Sichtprüfung lässt sich der
Schalter pro Marker aktivieren; dadurch wird der Fit nicht validiert.

Links/rechts entspricht positiver/negativer y-Koordinate im `base_link`:
**ID 24 ist links, ID 7 ist rechts.** Die ursprünglichen Aufnahmen enthalten
vertauschte Labels (`front_left` für ID 7). Deren Bilder und Messdateien
bleiben unverändert; der Export dokumentiert die Korrektur ausdrücklich.
Die fixierte Höhenreferenz ist daher **ID 7 / `front_right`: 0,44 m**.
Die Kantenlänge wurde weiterhin mit **0,16 m** angesetzt.

In den rohen Mocap-Referenzen der Kameraroboter-D variiert die Höhe zwischen
Messposen um 13,5 mm und die Drehung um bis zu 1,60° relativ zur ersten Pose.
Die Ursache ist noch nicht unabhängig geprüft. Zudem liefert der gemeinsame
Fit einen Kameraursprung rechts unterhalb von `base_link` z=0. Vor einer
Freigabe müssen Mocap-Körperframes und Tracking, die tatsächlichen
Markermaße und die unabhängigen Bildresiduen überprüft werden. Die neuen
Kameratransformationen werden nicht in die Hardware-TFs übernommen.

`calibration_groups` hält die Herkunft beider Paare getrennt fest. Die
bisherigen roboterweiten Herkunftsfelder beziehen sich aus Kompatibilitäts-
gründen weiterhin auf die hintere Kalibrierung; jeder Frontmarker nennt
seine eigene Quellsitzung.
