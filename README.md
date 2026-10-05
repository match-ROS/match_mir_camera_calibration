# MiR camera and MuR marker calibration

Eigenständige ROS-2-Jazzy-GUI auf Basis von `MurBaseGui`, parallel zur
Mocap-GUI. Standardmäßig wird MuR620a **manuell mit dem Joystick** vor
den beiden RGB-Kameras der stehenden **MuR620d** positioniert. Eine lokale
Webansicht zeigt auf dem iPhone beide Kamerabilder mit Markerpositionen und
bietet einen Knopf für die Aufnahme einer neuen Messpose. Rohe Qualisys-Posen liefern
die bekannte relative Roboterbewegung. Eine gemeinsame Offline-Optimierung
bestimmt beide Kamera- und beide Markertransformationen relativ zu `base_link`.

Im manuellen Modus wird **kein `cmd_vel`-Publisher angelegt**, auch keine
Nullbefehle bei Aufnahme, Abbruch oder Beenden gesendet. Die Joystick-Steuerung
bleibt unabhängig. Automatische Fahrt und Planung sind in diesem Modus gesperrt.
Der automatische Modus bleibt separat verfügbar; seine Hardwareprüfung steht aus.

## Installation und Start

Benötigt werden die vorhandenen Pakete `match_mur_gui`, `match_mocap_ros2`
und `mir_launch_hardware` sowie PyQt5, OpenCV mit ArUco, NumPy, SciPy,
PyYAML und `cv_bridge`. ROS-Abhängigkeiten stehen in `package.xml`.

```bash
cd /home/rosmatch/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select match_mir_camera_calibration --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=62
ros2 run match_mir_camera_calibration mir_camera_calibration_gui
```

Die Basis-GUI wählt A/D, MiR und MiR-Kameras vor. Arme, MoveIt und integrierter
Controller sind zunächst abgewählt. Andere Rollen A/B/C/D können konfiguriert
werden; die MuR-Auswahl für den Hardwarestart entsprechend ändern.
Falls noch nicht gesetzt, ist `ROS_STATIC_PEERS=mur620a;mur620d` voreingestellt.
Für andere Rollen/Rechnernamen die Discovery-Peers vor dem Start anpassen.

## Manueller Versuch mit iPhone

1. MuR620d abstellen und mit den Kameras auf die vorderen Marker der A ausrichten.
   Hardware/Kamera-Bridge für A und D sowie die vorhandene Mocap-Bridge starten.
   Benötigt werden beide RGB-Streams mit `CameraInfo`, rohe Posen unter
   `/qualisys/mur620a/pose` und `/qualisys/mur620d/pose`. MiR-`robot_state`-Topics
   werden für manuelle Aufnahmen nicht benötigt.
2. In der GUI den Modus **manual** lassen und **Backend laden** drücken.
   Das startet Recorder und Webserver zusammen. Alternativ ohne Desktop-GUI:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/rosmatch/colcon_ws/install/setup.bash
   export ROS_DOMAIN_ID=62
   export ROS_STATIC_PEERS='mur620a;mur620d'
   ros2 run match_mir_camera_calibration calibration_web
   ```

   Eigene Einstellungen: `calibration_web --config /pfad/session.yaml`.
   Anderer Port: `--port 8081`. Vor dem Wechsel zwischen GUI und eigenständigem
   Webserver das vorige Backend beenden; es darf nur eines laufen.
3. In der GUI **iPhone-QR-Code** drücken oder den QR-Code direkt im Terminal
   mit der iPhone-Kamera scannen und den Link in Safari öffnen. Zusätzlich wird
   der QR-Code als PNG gespeichert; der Startlog nennt den Pfad und den
   passenden `xdg-open`-Befehl. Das Bild bleibt für die Laufzeit des Servers verfügbar.
   Der vollständige Link inklusive Sitzungstoken ist im QR-Code enthalten.
   iPhone und Rechner müssen einander im WLAN/LAN erreichen können. Die
   LAN-Adresse wird automatisch gewählt; bei mehreren Netzwerken kann sie mit
   `--advertise-host 10.145.8.71` vorgegeben werden (passende Rechner-IP einsetzen).
   Der Standardport ist **8080**; der QR-Code gilt für diesen Serverstart.
4. Bilder und Overlays prüfen: IDs **24 vorne links / 7 vorne rechts**, Dictionary
   `DICT_APRILTAG_36h11`, schwarze Quadratseite **0,16 m** und Höhenreferenz des
   linken Markermittelpunkts **0,44 m** sind bereits eingetragen. Die Kameraseite
   bezeichnet die Kamera der D, nicht die Seite des Markers an A.
5. A mit dem Joystick positionieren, loslassen und **Neue Pose aufnehmen** drücken.
   Der Recorder wartet mindestens zwei Sekunden nach der Anforderung und auf
   stabilen Stillstand beider Roboter. Danach speichert er **fünf neue Bilder je
   Kamera**, `CameraInfo`, Detektionen und rohe 6D-Mocap-Posen. Erst nach
   **Aufnahme beendet** und erhöhtem Messposenzähler weiterfahren.
6. Für weitere Positionen und Drehwinkel wiederholen. **Aufnahme abbrechen**
   beendet die Aufnahme; dieser Webknopf stoppt nicht die Joystick-Fahrt.
   Fehlendes/veraltetes Mocap, Bewegung der D oder Bewegung während des
   Bildblocks verwerfen die Aufnahme. Verbindungsverlust zum Browser oder
   Safari im Hintergrund bricht laufende Aufnahmen ab, wenn kein weiterer
   Bedienclient Heartbeats liefert. Danach ausdrücklich neu aufnehmen.
7. Den angezeigten Messordner offline auswerten:

   ```bash
   ros2 run match_mir_camera_calibration calibrate_session /pfad/zum/messordner
   ```

Die Livepositionen sind vorläufige PnP-Schätzungen **im jeweiligen Kamera-Optical-Frame**:
x nach rechts, y nach unten, z in Blickrichtung, Werte in Metern. Das Overlay
zeigt außerdem die Markerachsen. Die Bilder werden für die Ansicht um 90°
gegen den Uhrzeigersinn gedreht; die Messbilder und Eckpunkte bleiben im
originalen Kamerabild. Bei anderem Kameramontagewinkel `preview_rotate_ccw`
in der YAML anpassen. Die endgültigen Transformationen zu `base_link` entstehen
erst in der Offline-Kalibrierung.

Die Vorschau funktioniert auch ohne Mocap; der Aufnahmeknopf bleibt dann
gesperrt. Alte Kamerabilder werden abgedunkelt und nicht als aktuelle Bilder
angezeigt. MiR-/PC-Uhren müssen synchron sein. Im manuellen Modus sind
Fahrbereich und Konturradien optional und es gibt keine automatische
Fahrweg-/Grenzüberwachung; die Fahrt wird mit dem Joystick bedient.

## Optionaler automatischer Versuch

Für automatische Fahrt `acquisition_mode: automatic` wählen und den Webserver
beenden. Fahrbereich, beide Konturradien und die folgenden Freigaben sind dafür
weiterhin Pflicht. Den Recorder über die Desktop-GUI laden.

1. Arme abstellen, freien Fahrbereich festlegen und den Kamera-MuR auf die
   konfigurierte Marker-Seite der A ausrichten. MiR-Hardware und die Kamera-Bridge starten, beispielsweise über
   **Start Hardware**. Bestehendes Mocap weiterverwenden oder **Mocap-Bridge
   starten** wählen. Eine eigene Bridge publiziert nur Rohdaten, ohne Map-/Roboter-TF.
2. Pflichtfelder im Tab **Konfiguration** eintragen. Das Muster
   [config/session.yaml](config/session.yaml) enthält bereits das erkannte
   Dictionary `DICT_APRILTAG_36h11`, für beide Marker die gemessene Kantenlänge
   von **0,16 m** und für den vorderen linken Marker die Höhenreferenz **0,44 m** im
   `base_link`. Bereich und Konturradien sind für automatische Fahrt noch einzutragen.
   Für erweiterte Einstellungen YAML bearbeiten und anschließend
   **YAML in die Formularfelder übernehmen** wählen.
   Das Raster richtet die konfigurierte Marker-Seite (`front`/`rear`) zur Kamera aus.
3. **Backend laden**. Beide Bilder mit Marker-Overlays prüfen. MiR- und PC-Uhren
   müssen ausreichend synchronisiert sein: Bilder älter als eine Sekunde,
   zukünftige und rückwärts laufende Zeitstempel werden ausgeschlossen.
4. **Einzelaufnahme** testen. Bei mindestens zwei Sekunden Stillstand werden
   je Kamera fünf neue Bilder gespeichert. `motion_enabled` darf dabei `false` sein.
5. **Plan erzeugen**, Draufsicht und Messposen prüfen. Tabellenwerte bearbeiten
   und mit **Editierte Messposen übernehmen** einreichen. Ungültige Wege werden
   abgelehnt. Alle Teilwege werden vor dem Start angezeigt.
6. Für die erste Fahrprüfung `motion_enabled`, `exclusive_control_confirmed`
   und `arms_stowed_confirmed` setzen, Backend neu laden und erneut planen.
   **Kurze Stoppprüfung** fährt höchstens 10 cm entlang eines geprüften ersten
   Teilwegs mit maximal 0,01 m/s und 0,03 rad/s. Sind Start und Ziel identisch,
   bleibt sie stehen. STOPP, Konturgrenzen und Verhalten bei ausbleibenden
   Befehlen am MiR überprüfen. Der Test bestätigt die Freigabe nicht automatisch.
7. Erst nach erfolgreicher Prüfung `boundary_verified` setzen, Backend neu
   laden, Plan ansehen und **Start / Fortsetzen** drücken. D bleibt stehen.
   Pause und STOPP sind jederzeit verfügbar; Wiederaufnahme erfolgt ausdrücklich.
8. Nach Abschluss unter **Auswertung** offline kalibrieren. Ergebnisse stehen
   im Messordner als `calibration.yaml` und `quality_report.json`.

Änderungen der Konfiguration erfordern einen gestoppten, neu geladenen
Backend-Prozess. Die allgemeine STOPP-Funktion beendet ebenfalls die
Kalibrierfahrt. Während der Fahrt sperrt die Anwendung die übrigen
Bedienknöpfe der Basis-GUI. Andere Missions-/Fahrsteuerungen müssen beendet
sein; im automatischen Modus werden zusätzliche `cmd_vel`-Publisher abgelehnt. Auch ein inaktiver
Publisher eines zuvor geöffneten Jog-Dialogs kann erkannt werden; dann
dessen Prozess beenden oder die GUI neu starten.

## Maße und Frames

- Dictionary, unterschiedliche IDs und tatsächliche schwarze Außenkantenlängen
  der Printouts eintragen; den weißen Druckrand nicht zur Länge zählen.
  Die Länge des gesamten schwarzen Quadrats einschließlich Rahmen ist gemeint,
  nicht die Rahmenbreite. Für beide vorhandenen Marker sind **16 cm = 0,16 m**
  als Standard vorbelegt.
- `height_anchor.z_m` ist die Höhe des Markermittelpunkts **im `base_link` der A**,
  nicht automatisch die Höhe über dem Boden. Im aktuellen MuR620-Modell ist
  `base_footprint → base_link` eine Identität (Translation und Drehung null).
  Damit entsprechen die gemessenen **44 cm über dem Boden** bei waagerechtem
  Stand **0,44 m im `base_link`**; dieser Wert ist für `front_left` vorbelegt.
  Die bisher aufgenommenen hinteren Marker haben eine Referenzhöhe von 0,58 m;
  deren gespeicherte Sitzungskonfiguration bleibt unverändert.
  Bei einem anderen Modell mit Höhenversatz gilt
  `z_marker_in_base_link = Höhe_über_Boden − Höhe_base_link_über_Boden`.
- Die Konturradien umfassen den kompletten Roboter inklusive abgestellter Arme,
  Aufbau und Überständen, bezogen auf `base_link`. Die vereinfachte MiR-URDF-Box
  reicht dafür nicht als Messgrundlage. Umkreise berücksichtigen alle Drehstellungen.
- `bounds` beschreibt im automatischen Modus das äußere freie Bodenrechteck im Qualisys-Frame `mocap`.
  Es wird um Kontur und Bremsreserve verkleinert. Der Kameraroboter ist ein zusätzliches Hindernis.
- Rohe QTM-Körperframes müssen gemäß vorhandener Mocap-Konfiguration mit den
  jeweiligen `base_link`-Frames zusammenfallen.
- Kamera-Optical-Frames stammen aus den echten Bild-/CameraInfo-Headern.
  Der generische URDF-Kameraframe wird nicht als Ersatz angenommen.

Exportkonvention: `parent_T_child` bildet Kindkoordinaten auf Elternkoordinaten
ab, Translation in Metern, Quaternion `xyzw`. Markerframes verwenden OpenCVs
zentrierte Quadratkonvention mit den detektierten Ecken
`[-L/2,+L/2,0], [+L/2,+L/2,0], [+L/2,-L/2,0], [-L/2,-L/2,0]`.

## Wechsel zwischen vorderen und hinteren Markern

Der Standard ist nun das vordere Paar der MuR620a: **links ID 24, rechts ID 7**,
Dictionary `DICT_APRILTAG_36h11`, schwarze Quadratseite 0,16 m,
Höhenreferenz `front_left: 0.44`. Links/rechts gilt aus Sicht des Roboters in
Fahrtrichtung; beim Blick auf seine Front erscheinen die Seiten vertauscht.
Die Kameranamen sagen nichts über die Markerseite an A aus: In den ersten
Aufnahmen sah die linke Kamera der D ID 24, die rechte ID 7. Die Zuordnung
wurde bei der Auswertung anhand der positiven/negativen y-Koordinate in
`base_link` korrigiert; alte Sitzungen behalten ihre ursprünglichen Labels.

In der GUI wählt **Markerpaar** `front` oder `rear`. Bei einer eigenen YAML
müssen `markers` und `height_anchor.marker` denselben Namenssatz verwenden:
`front_left/front_right` oder `rear_left/rear_right`. Alte Messordner mit den
hinteren IDs 0/1 und 0,58 m bleiben mit ihrer eigenen Konfiguration auswertbar.
Die gemeinsame Optimierung und die Exportframes verwenden die jeweils
aufgenommenen Namen. Ein Paarwechsel benötigt einen neu geladenen Backend;
es beginnt ein neuer Messordner, und bisherige Aufnahmen bleiben erhalten.

## Robustere Erkennung bekannter Marker

`robust_detection: true` ist der Standard. Die gewöhnliche OpenCV-Erkennung
bleibt der erste Schritt. Fehlende IDs werden zusätzlich mit einem Dictionary
nur aus den beiden konfigurierten Originalcodes gesucht. Die Original-IDs und
Druckausrichtungen bleiben erhalten. Die Fehlerkorrektur ist auf höchstens
zwei interne Code-Bits begrenzt; der Abstand zu **allen** Codes und Drehungen
des ursprünglichen Dictionarys kann diese Grenze weiter verkleinern.

Zusätzliche Suchdurchläufe verwenden eine zweifach vergrößerte Darstellung
oder lokalen Kontrastausgleich (CLAHE). Erkannte Ecken werden in die
Originalpixelkoordinaten zurückgerechnet und am Originalbild verfeinert.
Dabei wird das kleine Suchfenster an die Zellgröße angepasst. Der Code und
der schwarze Rand werden anschließend erneut auf den Originalpixeln geprüft.
Zusätzliche Treffer benötigen sichtbare Ecken mit Abstand zum Bildrand und
eine positive PnP-Lösung mit höchstens 1,5 Pixel RMS-Reprojektionsfehler.
Vollständige Verdeckung, verschwundene Marker und doppelte bekannte IDs
werden nicht durch Positionsannahmen ersetzt.

Jeder Bilddatensatz speichert `detector` (Version, OpenCV, Einstellungen) und
pro Fund `detection_method`, `correction_limit_bits`; zusätzliche Treffer
halten auch die beim Nachprüfen gefundenen `corrected_bits` fest. Originalbilder
bleiben verlustfreie PNGs. Für einen direkten Vergleich kann in der YAML
`robust_detection: false` gesetzt und das Backend neu geladen werden.

Ein Vergleich auf 235 gespeicherten Bildern der Sitzung
`20261005T141627Z_a2cfeb5e` ergab **58 → 128 Markerfunde**, ohne Verlust der
bisherigen Treffer. Das ist ein Vergleich der Erkennungsanzahl; größere
Verdeckungen und die absolute Kalibriergenauigkeit sind damit nicht validiert.
Die neuen Suchschritte brauchen auf diesem Rechner im Median etwa 7 ms je
Bild. Die Marker waren häufig nur rund 25–35 Pixel breit.

Bereits aufgenommene Sitzungen können erneut mit dem aktuellen Detektor
ausgewertet werden:

```bash
ros2 run match_mir_camera_calibration calibrate_session /pfad/zur/sitzung --redetect
```

Die Neudetektion verwendet ausschließlich akzeptierte Stillstandsaufnahmen;
Originalbilder, Zeitstempel und `measurement.json` bleiben unverändert.
`redetection_report.json` protokolliert den Vergleich. Die gewohnte gemeinsame
Optimierung und unabhängige Validierung folgen danach; `calibration.yaml`
und `quality_report.json` werden als neue Auswertungsergebnisse geschrieben.
Ohne `--redetect` werden weiterhin die ursprünglich gespeicherten Eckpunkte
verwendet.

Stark verdeckte Außenkanten oder Ecken können diese Suche weiterhin verhindern.
Für spätere Lokalisierung kann nach der Kalibrierung die feste Geometrie beider
Marker gemeinsam genutzt werden, wenn einer ausfällt. Ein daraus vorhergesagter
verdeckter Marker ist dann eine Vorhersage und keine zusätzliche Bildmessung.

## Aufnahme und Überwachung

Der unabhängige Sitzungsnode arbeitet mit 20 Hz und verwendet ausschließlich
`/qualisys/<robot>/pose` im Frame `mocap`, einschließlich z/roll/pitch.
Geglättete, Map- und eingefrorene Lokalisierungsposen werden nicht abonniert.
Im automatischen Modus müssen frische `/<robot>/robot_state`-Meldungen READY,
PAUSE oder MANUALCONTROL melden. Andere oder fehlende Zustände sperren diese
Sitzung. Im manuellen Modus wird `robot_state` weder abonniert noch verlangt:
frische rohe Mocap-Posen, Stillstand, gültige Bilder und der Bedien-Heartbeat
prüfen die Messaufnahme; der Recorder steuert keine Fahrt.

Vor jedem Bildblock müssen beide Roboter zwei Sekunden stabil stehen.
Erst danach belichtete Bilder dürfen in den aktuellen Block gelangen.
Originalheader, Empfangszeiten, CameraInfo, Rohposen und Eckpunkte werden
gespeichert. Die Zuordnung nutzt Stillstand und die bei Bildempfang aktuellen
Rohposen; eine exakte Synchronisierung während Bewegung wird nicht behauptet.
Bewegung, veraltetes Mocap, Kameraprobleme oder Abbruch verwerfen den Block.

Im automatischen Modus verwendet der Planer A* und analytische Kreiskollisionsprüfungen. Seine Reserve
beträgt `margin + v * (max(Heartbeat-Timeout, Mocap-Timeout) + Reaktionszeit)
+ v²/(2*Verzögerung)`. Standardgrenzen sind 0,05 m/s und 0,10 rad/s.
Im automatischen Modus sendet ein unabhängiger Thread Nullbefehle bei ausbleibenden Kontrollticks oder
GUI-Heartbeat. Im manuellen Modus verwirft derselbe Watchdog nur die Aufnahme.
Der GUI-Heartbeat hängt vom Qt-Ereignisloop ab und erkennt auch eine
hängende GUI. Bei Prozess-/Rechnerausfall muss der überprüfte native MiR-Timeout
das Anhalten übernehmen. Softwaregrenzen ersetzen die MiR-Sicherheit nicht.

Jede Sitzung besitzt einen eigenen Ordner:

```text
manifest.json                # Version, Frames, Startschätzungen, Zeitsemantik
config.yaml                  # Aufnahmekonfiguration
poses.jsonl                  # Kontinuierliche rohe Mocap-Daten
events.jsonl                 # Zustandswechsel und Stoppgründe
measurements/000000/
  left_000.png               # Verlustfreie Originalbilder ohne Overlay
  right_005.png
  measurement.json           # Akzeptiert/verworfen, Posen, Header, Eckpunkte
calibration.yaml             # Nach erfolgreicher Auswertung
quality_report.json
```

Nur vollständig geschriebene, akzeptierte Blöcke werden gefittet. Bei hartem
Abbruch bleiben unvollständige Blöcke ausgeschlossen. Ein fehlgeschlagener
erneuter Fit verschiebt eine vorhandene Kalibrierung nach
`calibration.previous.yaml` und schreibt den Ablehnungsgrund in den Bericht.

## Offline-Kalibrierung

```bash
ros2 run match_mir_camera_calibration calibrate_session /pfad/zur/sitzung
# Ohne ROS, bei verfügbaren Python-Abhängigkeiten:
python3 -m match_mir_camera_calibration.calibration /pfad/zur/sitzung
```

Intrinsik und Verzerrung aus `CameraInfo` bleiben fest. Unterstützt sind
unbeschnittene, ungebinnte RGB-Bilder mit `plumb_bob` oder `rational_polynomial`.
ArUco-Subpixelecken und beide IPPE-Square-PnP-Hypothesen werden aufgezeichnet.
Der gemeinsame Fit minimiert Eckpunktfehler mit Huber-Verlust und mehreren
Startwerten. Tiefensensoren und Intrinsik werden nicht mitkalibriert.

Vorhandene Kamera-TFs werden bei Aufnahmebeginn als Startschätzungen gespeichert.
Fehlende TFs können über `camera_initial_guesses` ergänzt werden. Diese Werte
werden optimiert und sind keine festen Kalibrierreferenzen. Alternativ:

```bash
ros2 run match_mir_camera_calibration calibrate_session /pfad/zur/sitzung \
  --initial-guesses /pfad/initial_guesses.yaml
```

Die Datei enthält `left` und `right`, jeweils `[x,y,z,qx,qy,qz,qw]` für
`base_link -> optical`, oder eine Sitzungskonfiguration mit diesem Abschnitt.

Mindestens zehn akzeptierte Messposen sind für unabhängige Validierung
erforderlich. Jede Kamera und jeder Marker müssen in mindestens sechs
verschiedenen Trainingsmessungen vorkommen. Der Beobachtungsgraph muss
verbunden sein: Mindestens ein Marker muss im Verlauf von beiden Kameras
gesehen werden. Reine Translation ohne unterschiedliche Drehwinkel wird als
unbestimmbar zurückgewiesen.

Bei Bodenfahrten bleibt ein gemeinsamer Höhenversatz unbestimmt. Die gemessene
Markerhöhe setzt die Höhenreferenz. B muss nicht fahren; unterschiedliche
Positionen und Drehwinkel der A liefern die relative Anregung.

Wiederholungen nahezu identischer relativer Posen (unter 3 cm und 3 Grad)
werden gemeinsam gruppiert. Alle Bilder jeder fünften Posegruppe bleiben beim
Validierungsfit zurückgehalten; Wiederholungen gelangen damit nicht zugleich
in Training und Validierung.
Der Bericht zeigt Pixel-Reprojektionsfehler und Abweichungen bildbasierter
relativer Roboterposen gegenüber Mocap. Die PnP-Hypothese für die Poseprüfung
wird anhand ihres eigenen Bildfehlers ausgewählt; planare Mehrdeutigkeit wird
nicht durch Auswahl anhand von Mocap verdeckt. Anschließend wird der Export
auf sämtlichen akzeptierten Daten gefittet.

`quality: validated` bedeutet Konvergenz, vollen Rang, Konditionszahl höchstens
10⁶ und 95%-Quantil des unabhängigen Eckpunktfehlers höchstens 3 Pixel.
Andernfalls bleibt ein bestimmbarer Fit `draft`; unbestimmbare Daten erhalten
einen Ablehnungsbericht. Absolute Genauigkeit hängt weiterhin von Markerhöhe,
QTM-Frames und Intrinsik ab. TFs werden nicht automatisch in die Hardware eingetragen.

## ROS-Schnittstellen

Der Backend-Node erhält `config_path` und einen zufälligen GUI-`owner_token`.
Es darf nur eine Kalibriersitzung gleichzeitig laufen.

| `/mir_camera_calibration/…` | Typ | Bedeutung |
| --- | --- | --- |
| `prepare` | `std_srvs/Trigger` | Raster und Wege prüfen, Vorschau erzeugen |
| `start` | `std_srvs/Trigger` | Starten oder nach Pause fortsetzen |
| `verify` | `std_srvs/Trigger` | Begrenzte 10-cm-Stoppprüfung |
| `pause` | `std_srvs/Trigger` | Nullbefehl, Burst verwerfen, pausieren |
| `stop` | `std_srvs/Trigger` | Nullbefehl und Sitzung stoppen |
| `capture` | `std_srvs/Trigger` | Stationäre Einzelaufnahme |
| `status` | `std_msgs/String` | JSON: Zustand, Plan, Rohposen, Bildalter, Messordner; transient-local |
| `waypoints` | `std_msgs/String` | JSON-Liste `[x,y,yaw_rad]`, nur bei inaktiver Fahrt |
| `heartbeat` | `std_msgs/String` | Besitzer-Token, GUI alle 0,15 s / aktiver Browser über Web-Bridge alle 0,10 s |
| `preview/{left,right}` | `sensor_msgs/Image` | BGR-Overlay |

Nur im automatischen Modus gehen Fahrbefehle auf `/<target>/cmd_vel_stamped` als `TwistStamped` mit
`base_link`-Frame. Kameraeingänge sind
`/<observer>/camera_floor_{left,right}/driver/color/{image_raw,camera_info}`.

## Tests

```bash
cd /home/rosmatch/colcon_ws/src/match_mir_camera_calibration
python3 -m pytest -q  # ROS-Tests ohne gesourctes ROS ggf. übersprungen
source /opt/ros/jazzy/setup.bash
source /home/rosmatch/colcon_ws/install/setup.bash
ROS_DOMAIN_ID=182 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ROS_STATIC_PEERS='' QT_QPA_PLATFORM=offscreen python3 -m pytest -q
```

ROS-Tests verwenden nur `calibration_test_a/b` auf localhost, ohne Hardware-Bridge.
Sie prüfen auch die Web-Aufnahme über HTTP und ROS bis zum Messordner,
Vorschau ohne Mocap, Aufnahme ohne jegliche `robot_state`-Publisher und
Koexistenz mit einem Joystick-Publisher. Im manuellen
Modus wird kein Fahrpublisher angelegt und kein Fahrbefehl gesendet.
Die HTTP-Tests prüfen Kamerabilder, Capture/Cancel, Sitzungslink und gesperrte Fahraktionen.
Weitere Tests decken verrauschte Rekonstruktion, Höhenreferenz, fehlende Anregung,
getrennte Sichtgraphen, Wege um B, Konturgrenzen, Stillstand, Pause/Stopp,
Mocap-/Heartbeat-Ausfall, Roboterfehler, Bewegung der B, PNG-Rundlauf und
GUI-Planbearbeitung ab.
