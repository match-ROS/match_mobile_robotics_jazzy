# Kalibrierte Markerframes

Die MuR620-Beschreibung lädt `config/calibrated_markers.yaml` und ergänzt für
den tatsächlich übergebenen `robot_namespace` feste Markerlinks unter
`base_link`. Die Hardware- und Simulationslaunches übergeben diesen Namen.
`robot_state_publisher` ergänzt anschließend genau einmal den Roboterpräfix.
Andere MuRs übernehmen keine Transformationen der MuR620a.

Der erste Datensatz enthält die hinteren Marker der MuR620a:

| ID | TF-Kindframe | Translation relativ zu `mur620a/base_link` [m] |
| --- | --- | --- |
| 0 | `mur620a/aruco_rear_left` | −0.68037554, +0.24727121, +0.58000000 |
| 1 | `mur620a/aruco_rear_right` | −0.67159335, −0.24572920, +0.58561780 |

Dictionary: `DICT_APRILTAG_36h11`; schwarze Außenkante: 0,16 m. Die YAML hält
die vollständigen Translationen und Quaternionen (`xyzw`) sowie die daraus
berechneten URDF-Eulerwinkel in Radiant fest. Die Transformationen bilden
Markerkoordinaten auf `base_link`-Koordinaten ab. Der Ursprung liegt in der
Markermitte; die Achsen folgen der OpenCV-Quadratkonvention der Kalibrierung,
einschließlich der tatsächlichen Druckausrichtung.

Quelle: Sitzung `20261005T132811Z_a702dd6a`, 14 akzeptierte Messposen. Das
Ergebnis bleibt als **draft** dokumentiert: Die zurückgehaltenen Messposen
ergeben einen 95%-Eckpunktfehler von ca. 3,87 Pixeln. Die feste Referenzhöhe
des linken Markers beträgt 0,58 m. Vor einer Nutzung zur Lokalisierung ist die
unabhängige Prüfung dieser Genauigkeit erforderlich.

Die Frames erscheinen nach dem nächsten Start des zuständigen
`robot_state_publisher` mit der neu gebauten Beschreibung. Ein bereits
laufender Publisher lädt geänderte Xacro-Dateien nicht selbstständig nach.
Es wird kein zusätzlicher statischer Publisher parallel zur Hardware gestartet.

Die neu aufgenommenen vorderen Marker (IDs 7/24, Höhe 0,44 m) sind hier noch
nicht eingetragen. Nach deren eigener Kalibrierung können `front_left` und
`front_right` ergänzt werden; das Xacro unterstützt beide Paare.
