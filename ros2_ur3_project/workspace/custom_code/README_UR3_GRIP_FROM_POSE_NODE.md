# README 2 - UR3 Grip From Pose (heutiger Stand)

Diese Datei erklaert, wie `ur3_grip_from_pose_node.py` verwendet wird und fasst alle heute umgesetzten Anpassungen zusammen.

## 1) Zweck der Datei

`ur3_grip_from_pose_node.py` empfängt eine Zielpose vom Vision-Teil, rechnet sie in `base_link` um und fuehrt einen kompletten Greifablauf mit MoveIt und Tool-IO aus.

Im Unterschied zur Hover-Variante faehrt dieser Node auf den gespeicherten Griffpunkt und verwendet zusaetzliche fahr- und greif-spezifische Schritte.

## 2) Eingabe-Topic und Format

- Topic: `/tool_target_pose`
- Typ: `geometry_msgs/msg/PoseStamped`
- `header.frame_id` kann so aussehen:
  - `aruco_0`
  - `aruco_0|cropv1_clean_direction`

Wenn eine Template-ID enthalten ist (`frame|template_id`), wird der passende gespeicherte Griffpunkt-Offset verwendet.

## 3) Aktueller Ablauf im Node

1. Pose empfangen auf `/tool_target_pose`.
2. Template-Offset anwenden (`TEMPLATE_GRASP_OFFSETS`).
3. Pose in `base_link` transformieren.
4. Zur gespeicherten Griffposition planen und fahren.
5. Zusaetzliche Fahrt in der Ebene entlang Marker-`+Y` (aktuell 4 cm).
6. Greifer schliessen:
   - `Pin 16` auf `1`
   - mindestens `HOLD_SECONDS` halten
   - `Pin 16` bleibt waehrend der Greifphase auf `1` bis zum Loslassen.
7. Nach dem Greifen:
   - 2 cm nach oben
   - 2 cm wieder nach unten
8. Loslassen:
   - `Pin 16` auf `0` (Freigabe)
   - `Pin 17` Open-Puls
9. Rueckfahrt mit gleicher Logik:
   - 4-cm-Schritt in der Ebene rueckwaerts
   - danach normale Rueckfahrt zur Home-/Kameraposition.

## 4) Wichtige Parameter (oben in der Datei)

- `PRE_GRIP_MARKER_Y_OFFSET_M = 0.04`
  - Zusaetzliche Fahrstrecke in der Ebene entlang Marker-`+Y`.
- `POST_GRIP_LIFT_M = 0.02`
  - Nach dem Greifen 2 cm hoch und wieder runter.
- `HOLD_SECONDS = 2.0`
  - Mindest-Haltezeit beim Schliessen.
- `OPEN_SECONDS = 2.0`
  - Dauer des Open-Pulses.
- `MIN_TARGET_Z_IN_BASE_M = 0.003`
  - Sicherheitsuntergrenze fuer Ziel-Z.
- `PLANNING_Z_RETRY_STEPS_M = (0.0, 0.008, 0.015)`
  - Retry-Hoehen, falls Planung auf Zielhoehe scheitert.
- `TEMPLATE_GRASP_OFFSETS[...]`
  - Gespeicherter Griffpunkt-Offset pro Template.
  - Mit `translation_sign_xyz` fuer Achs-/Vorzeichenkorrekturen.

## 5) Starten

### Option A: Direkt als Python-Skript

```bash
python3 /workspace/custom_code/ur3_grip_from_pose_node.py
```

### Option B: Ueber deine bestehenden Startskripte/Container

Starte wie bisher dein UR3-Setup (Docker/Compose + ROS2 Umgebung), dann diesen Node im passenden Container.

## 6) Minimaler Test

- Pruefen, ob Topic vorhanden ist:

```bash
ros2 topic list | grep tool_target_pose
```

- Testnachrichten beobachten:

```bash
ros2 topic echo /tool_target_pose
```

Achte im Log auf diese Marker:
- `[TEMPLATE]` (Offset aktiv)
- `[PLANNING]` / `[EXECUTION]`
- `[PRE-GRIP]` (4-cm-Schritt)
- `[GRIPPER]` (Pin 16/17)
- `[POST-GRIP]` (2 cm hoch/runter)
- `[RETURN]` (4-cm-Rueckfahrt)

## 7) Was heute angepasst/neu hinzugefuegt wurde

- Trennung von Hover- und Direct-Grip-Variante (eigene Datei fuer Direct Grip).
- Template-ID in `frame_id` (`frame|template_id`) ausgewertet.
- Template-basierte Griffpunkt-Offsets eingebaut.
- Vorzeichensteuerung ueber `translation_sign_xyz` eingefuehrt.
- Direktfahrt zur gespeicherten Griffposition umgesetzt.
- Zusaetzliche Marker-`+Y` Fahrt in der Ebene (auf 4 cm angepasst).
- Rueckfahrt der 4-cm-Strecke mit gleicher Logik (reverse) eingebaut.
- Nach-Greif-Bewegung: 2 cm hoch, 2 cm runter, dann loslassen.
- Greifer-IO auf robustes Worker-Thread-Warten angepasst (kein `spin_until_future_complete` im Worker-Pfad).
- `Pin 16` waehrend der Greifphase dauerhaft auf `1` gehalten; Freigabe erst vor dem Oeffnen.
- Planungsrobustheit erhoeht (Z-Untergrenze + Retry auf hoeheren Z-Werten).

## 8) Typische Probleme und schnelle Loesungen

- Problem: `Keine Trajektorie gefunden`
  - Ursache oft: Ziel zu nah am Tisch.
  - Loesung: `MIN_TARGET_Z_IN_BASE_M` erhoehen (z. B. 0.005 bis 0.010).

- Problem: Greifer laesst trotzdem los
  - Pruefe reale IO-Logik der Hardware (active-high/active-low, Pin-Belegung).
  - Pruefe in Logs, ob waehrend Greifphase ungewollt `Pin 16 -> 0` gesendet wird.

- Problem: Zusatzzug in falsche Richtung
  - `PRE_GRIP_MARKER_Y_OFFSET_M` auf negatives Vorzeichen setzen.

## 9) Verwandte Dateien

- `ur3_grip_from_pose_node.py` (dieser Ablauf)
- `ur3_hover_grip_from_pose_node.py` (Hover-Variante)
- `gripper_ros.py` (einfacher IO-Test fuer Open/Close)
- `cube_manipulator_gripper_ros.py` (Referenz fuer frueher funktionierenden Ablauf)
