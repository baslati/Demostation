# README2 - Ausfuehrliche Dokumentation der letzten Schritte (UR3 / MoveIt / ros2_control)

## Ziel der Arbeiten
Das Ziel war, die wiederholten Ausfuehrungsabbrueche mit
`PATH_TOLERANCE_VIOLATED` zu beheben, die in der Praxis beim Senden von
action goals an den Trajektoriencontroller auftraten.

Typische Fehlermeldungen waren:
- `State tolerances failed for joint X`
- `Position Error: ... , Position Tolerance: 0.200000`
- `Controller ... failed with error PATH_TOLERANCE_VIOLATED`

## Ausgangslage und beobachtete Fehlerbilder
In den ersten Logsequenzen traten groessere Fehler am letzten Handgelenk
(wrist_3 / joint 5 je nach Zaehlung) auf, oft in Vielfachen von 2*pi
(z. B. ca. 6.283, 12.566, 25.133). Spaeter wurden die Fehler klein,
aber weiterhin knapp oberhalb der Toleranz (z. B. 0.2002 bei Toleranz 0.2000),
und wechselten teils auf andere Gelenke (z. B. joint 2).

Das deutete auf zwei Ebenen hin:
1. Fruehphase: Branch/Winkel-Umbruch-Thema (2*pi-Spruenge)
2. Spaetphase: Tracking/Randfall-Thema (knapp ueber Path-Tolerance)

## Konkrete Aenderungen, die nacheinander gemacht wurden

### 1) MoveIt-Gelenkgrenzen fuer wrist_3 angepasst
Datei:
- `ros2_ur3_project/workspace/custom_code/custom_ur_moveit_config/config/joint_limits.yaml`

Aenderungen in verschiedenen Iterationen:
- Zunaechst wurde wrist_3 als kontinuierlich behandelt (`has_position_limits: false`,
  mit Wraparound).
- Spaeter wurde auf grosses Revolute-Modell umgestellt, um Winkelzweige
  robuster zu halten (grosse Min/Max-Grenzen im Bereich von mehreren Umdrehungen).

Zweck:
- 2*pi-Branching-Effekte zwischen geplanter und gemessener Gelenkdarstellung
  reduzieren.

### 2) UR-Beschreibungsgrenzen fuer ur3e wrist_3 erweitert
Datei:
- `ros2_ur3_project/workspace/custom_code/custom_ur_description/config/ur3e/joint_limits.yaml`

Aenderung:
- wrist_3 von kontinuierlich auf gross begrenztes Revolute-Modell gesetzt
  (weite Positionsgrenzen).

Zweck:
- Konsistenz zwischen Beschreibung, Planung und Laufzeitverhalten herstellen.

### 3) Controller-Setup neu aufgebaut (eigene lokale ur_controllers.yaml)
Datei:
- `ros2_ur3_project/workspace/custom_code/custom_ur_description/config/ur_controllers.yaml`

Aenderungen:
- Vollstaendige lokale Controllerdatei erstellt.
- scaled und joint_trajectory Controller definiert.
- joints, interfaces, constraints, speed scaling Parameter sauber gesetzt.
- Temporaer wurden in mehreren Schritten einzelne Joint-Trajectory-Toleranzen
  angepasst (insbesondere shoulder_lift und wrist_3), danach teilweise wieder
  zurueckgenommen, um Symptomfixes zu vermeiden.

Wichtig:
- Die Toleranzanpassungen allein loesten das Grundproblem nicht dauerhaft,
  sondern nur situativ.

### 4) Update-Rate-Datei ergaenzt
Datei:
- `ros2_ur3_project/workspace/custom_code/custom_ur_description/config/ur3e_update_rate.yaml`

Aenderung:
- `controller_manager.ros__parameters.update_rate: 500`

Zweck:
- Vollstaendige Runtime-Config fuer das Custom-Paket bereitstellen.

### 5) MoveIt-Controller-Mapping umgestellt (scaled <-> joint)
Datei:
- `ros2_ur3_project/workspace/custom_code/custom_ur_moveit_config/config/controllers.yaml`

Aenderungen:
- Mehrfach zwischen `scaled_joint_trajectory_controller` und
  `joint_trajectory_controller` als Default gewechselt, um zu pruefen,
  ob die Tracking-Fehler durch Speed-Scaling-Kompensation beeinflusst sind.

Erkenntnis:
- Für reale UR-Laufzeitbedingungen ist der scaled Controller normalerweise
  passender, wenn Speed-Scaling im Spiel ist.

### 6) Eigene Python-Clients auf passenden Controller-Action-Endpoint gesetzt
Dateien:
- `ros2_ur3_project/workspace/custom_code/simple_ur_controller.py`
- `ros2_ur3_project/workspace/custom_code/cube_manipulator.py`
- `ros2_ur3_project/workspace/custom_code/cube_manipulator_gripper.py`
- `ros2_ur3_project/workspace/custom_code/cube_manipulator_gripper_ros.py`

Aenderungen:
- Action-Endpunkte wurden zwischen scaled/joint-Controller angepasst,
  parallel zu den Controller-Tests.

Zweck:
- Sicherstellen, dass Custom-Skripte denselben aktiven Controller wie MoveIt nutzen.

### 7) Launch-Kette korrigiert, damit wirklich die lokale Config geladen wird
Dateien:
- `ros2_ur3_project/workspace/custom_code/custom_ur_moveit_config/launch/combined_ur3e.launch.py`
- `ros2_ur3_project/workspace/custom_code/custom_ur_description/launch/ur_control.launch.py`

Aenderungen:
- `combined_ur3e.launch.py` auf `custom_ur_description/launch/ur_control.launch.py`
  umgestellt (statt driver-default).
- `runtime_config_package` explizit auf `custom_ur_description` gesetzt.
- `controllers_file` auf absolute Datei gesetzt:
  `/workspace/custom_code/custom_ur_description/config/ur_controllers.yaml`
- In `ur_control.launch.py` Unterstuetzung fuer absolute Pfade ergaenzt,
  damit kein stilles Zurueckfallen auf Paketdefaults passiert.

Zweck:
- Sicherstellen, dass wirklich die bearbeitete YAML zur Laufzeit verwendet wird.

### 8) Dynamik statt Toleranz als spaeterer Fokus
Datei:
- `ros2_ur3_project/workspace/custom_code/custom_ur_moveit_config/config/joint_limits.yaml`

Aenderung:
- Betroffene Gelenke (u. a. shoulder_lift, wrist_3) in Velocity/Acceleration
  reduziert, um Tracking knapp ueber Grenzwert zu vermeiden.

Zweck:
- Fehlerquelle an der Trajektorienhaerte reduzieren statt Toleranz nur zu vergroessern.

## Zusammenfassung der Erkenntnisse
1. Reines Erhoehen von Toleranzen war kein nachhaltiger Root-Cause-Fix.
2. Es gab Hinweise auf Winkelzweig/Umbruch (2*pi) und getrennt davon spaetere,
   knappe Tracking-Randfaelle.
3. Ein zentraler Punkt war, sicherzustellen, dass wirklich die lokale
   Controllerdatei geladen wird.
4. Danach bleiben vor allem dynamische Tracking-Themen relevant
   (Trajektorie zu "aggressiv" vs. reale Nachfuehrung).

## Wichtiger praktischer Hinweis (deine Beobachtung)
Du hast klar gesagt, dass **manuelles Bewegen wahrscheinlich wieder geholfen hat**.
Diese Beobachtung ist sehr plausibel und wichtig:
- Manuelles Entspannen/Bewegen des Arms kann Winkelzweige,
  interne Gelenkzaehler-Repraesentationen oder Ausgangsbedingungen
  wieder in einen guenstigeren Bereich bringen.
- Dadurch koennen zuvor auftretende harte Startabweichungen verschwinden,
  auch wenn die Softwarekonfiguration gleich bleibt.

Kurz: Es ist sehr gut moeglich, dass das manuelle Bewegen den entscheidenden
praktischen Effekt hatte und die Lage stabilisiert hat.

## Aktueller Stand (laut letzter Iteration)
- Launch-Pfad so angepasst, dass lokale Config erzwungen werden kann.
- Controller/MoveIt-Konfigurationen wurden konsistent aufeinander abgestimmt.
- Toleranz-only-Ansatz wurde nicht als finale Loesung betrachtet.
- Dynamikreduktion wurde als sinnvollere technische Richtung vorbereitet.
- Deine manuelle Intervention wird als relevanter Faktor mit aufgenommen.

## Empfehlung fuer naechste geordnete Schritte
1. Nur eine Konfiguration aktiv lassen (keine parallelen Varianten).
2. Mit derselben Startprozedur mehrere identische Testfahrten ausfuehren.
3. Fehlerfall immer mit komplettem Logblock sichern.
4. Wenn stabil: schrittweise Dynamik wieder anheben, nicht die Toleranz.
5. Manuelle Ausgangsstellung als reproduzierbaren Startzustand dokumentieren.

---
Diese README2 wurde erstellt, um die letzten Schritte transparent,
nachvollziehbar und ohne Verkuerzung zu dokumentieren.
