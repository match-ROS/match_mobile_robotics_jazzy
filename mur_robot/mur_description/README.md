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
| 24 | `mur620a/aruco_front_left` | +0.44151809, +0.34290741, +0.44000000 | vorläufig aktiv, ausdrücklich angefordert |
| 7 | `mur620a/aruco_front_right` | +0.43431384, −0.32583498, +0.44593749 | vorläufig aktiv, ausdrücklich angefordert |

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

Die Frames erscheinen beim Start des zuständigen `robot_state_publisher`
mit der neu gebauten Beschreibung. Ein bereits laufender Publisher lädt
geänderte Xacro-Dateien nicht selbstständig nach; für die aktuelle Sitzung
kann sein Parameter `robot_description` um die beiden festen Markerjoints
ergänzt werden. Damit veröffentlicht derselbe Publisher auch die Marker.

## Vorderes Paar: neue Messung, vorläufige Veröffentlichung

Quelle: Sitzung `20261006T133414Z_573a8ad0`. Messpunkt **`000000` wurde auf
ausdrücklichen Wunsch aus der Auswertung ausgeschlossen**, nachdem die rohe
Mocap-Pose der stehenden MuR620d zwischen dem ersten und zweiten Messpunkt
um etwa 10 mm in der Höhe und 1,57° in der Orientierung wechselte. Die
Originalbilder bleiben erhalten. Die übrigen **22 Messposen mit 220 Bildern**
gehen vollständig in die Auswertung ein.

Detektor v2 und zusätzliche Offline-Prüfungen von Farbkanälen, Skalierung,
Kontrast und Konturkandidaten liefern **164 Markerbeobachtungen**; davon
sind zehn zusätzliche Funde aus der Bildprüfung. Diese werden anhand der
Originalpixel, des Codes und beobachteter Ecken geprüft. Vorhergesagte
Mocap-Ecken werden nicht als Bildmessungen übernommen.

Ganze Posen `000005`, `000010`, `000015`, `000020` werden für die unabhängige
Validierung zurückgehalten. Die gemeinsame Kamera-/Markeroptimierung
konvergiert mit Rang **23/23**, Konditionszahl **139,47**. Der RMS-Eckpunktfehler
beträgt **1,48 px**, der 95%-Fehler **2,36 px** bei einer Grenze von 3 px.
Das Exportlabel **`validated`** bezeichnet die bestandenen numerischen und
Pixelprüfungen. Die bildbasierte Einzelmarker-PnP-Prüfung zeigt weiterhin
ca. **0,197 m Translation-RMS und 21,88° Rotation-RMS**; präzise
Lokalisierungsgenauigkeit ist damit nicht bestätigt.

Der vollständige Export einschließlich beider Kamera-Optical-Transformationen,
Quaternionen, Ausschlussgrund und Qualitätsbericht liegt unter
[config/calibration_results/mur620a_front_20261006T133414Z.yaml](config/calibration_results/mur620a_front_20261006T133414Z.yaml).
Die Markertransformationen sind zusätzlich in `calibrated_markers.yaml`
hinterlegt. **`publish_tf: true`** aktiviert die beiden Frontframes auf
ausdrücklichen Wunsch für die vorläufige Nutzung. Der Eintrag
`publication: provisional_operator_requested` dokumentiert diese Entscheidung.
Mit `publish_tf: false` pro Marker lassen sich die Frontframes wieder abschalten.

Links/rechts entspricht positiver/negativer y-Koordinate im `base_link`:
**ID 24 ist links, ID 7 ist rechts.** Die fixierte Höhenreferenz der neuen
Messung ist **ID 24 / `front_left`: 0,44 m**; der rechte Marker wird frei
geschätzt. Beide Kantenlängen betragen **0,16 m**. In der MuR620-URDF ist
`base_footprint → base_link` identisch, daher entspricht die Bodenhöhe hier
der Höhe im `base_link`.

Nach dem ersten Messpunkt variiert die rohe D-Pose um weniger als 0,7 mm
je Raumrichtung und 0,037° relativ zur zweiten Messung. Die Ursache des
anfänglichen Sprungs bleibt ungeklärt. Die Kameratransformationen werden
als Ergebnisse gespeichert; bestehende Kamera-Hardware-TFs bleiben erhalten.

Die vorherige Sitzung vom 5. Oktober mit vertauschten ursprünglichen
Frontlabels und fehlgeschlagener Validierung bleibt als historischer
[Export](config/calibration_results/mur620a_front_20261005T141627Z.yaml) erhalten.

`calibration_groups` hält die Herkunft beider Paare getrennt fest. Die
bisherigen roboterweiten Herkunftsfelder beziehen sich aus Kompatibilitäts-
gründen weiterhin auf die hintere Kalibrierung; jeder Frontmarker nennt
seine eigene Quellsitzung.
