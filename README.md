# Demostation: ROS2 Universal Robots Steuerung

## Projektübersicht

Dieses Projekt ermöglicht die Planung und Steuerung eines Universal Robots (UR) mit ROS2 und MoveIt. Ziel ist es, über RVIZ oder Python-Skripte die aktuelle Pose des Roboters abzufragen, eine neue Zielpose zu setzen, Kollisionen zu prüfen und die Bewegung auszuführen. Die Steuerung erfolgt direkt über einen Docker-Container.

## Funktionen

- **Abfrage der aktuellen Pose**: Python-Skripte fragen die Endeffektor-Position und -Orientierung ab.
- **Planung einer Zielpose**: Es kann eine neue Zielpose (Position + Orientierung) gesetzt werden, z.B. 5cm höher als die aktuelle Pose.
- **Kollisionsprüfung**: MoveIt prüft automatisch, ob die geplante Bewegung kollisionsfrei ist.
- **Ausführung der Bewegung**: Die geplante Trajektorie wird an den Controller gesendet und ausgeführt.
- **Docker-Unterstützung**: Das Projekt kann komplett im Docker-Container mit ROS2 und MoveIt betrieben werden.

## Verzeichnisstruktur

```
ros2_ur3_project/
├── code.deb
├── docker-compose.yml
├── Dockerfile
├── entrypoint.sh
├── workspace/
│   ├── custom_code/
│   │   ├── simple_ur_controller.py
│   │   ├── custom_ur_description/...
│   	└── custom_ur_moveit_config/...
│   └── src/
└── ...
```

## Setup & Nutzung für einen Seeed J4012 & UR3e

### 1. Docker-Container bauen und starten

```bash
docker compose up --build

xhost +local:docker

docker run -it --rm --net=host   --env="DISPLAY=$DISPLAY"   --env="LIBGL_ALWAYS_SOFTWARE=1"   --env="MESA_GL_VERSION_OVERRIDE=3.3"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --volume="$HOME/.Xauthority:/root/.Xauthority:rw"   --volume="$PWD/workspace/custom_code:/workspace/custom_code"   --privileged   --runtime=nvidia   ur3-ros2

```


### 2. im Container:

```bash
colcon build --symlink-install

ros2 launch custom_ur_moveit_config combined_ur3e.launch.py use_fake_hardware:=false

```

### 3. Funktionsweise

- Das Skript fragt die aktuelle Pose ab.
- Plant eine Bewegung (z.B. 5cm nach oben).
- Prüft Kollisionen und führt die Bewegung aus.

### 4. Python-Skript ausführen; in neuem Terminal

```bash
docker ps 
docker exec -it DOCKERNAME bash
```

```bash
cd /workspace/custom_code
python3 simple_ur_controller.py
```
### 5. Fehlersuche

#### IP ADRESSEN
IP Roboter: 192.168.122.20
IP Computer: 192.168.122.100

Anpassung der IP Adressen nötig, so hat es am Seeed J4012 geklappt
```bash
sudo ip addr flush dev enP8p1s0
sudo ip addr add 192.168.122.100/24 dev enP8p1s0
sudo ip link set enP8p1s0 up
```

#### Netzwerk-Fehler: can't initialize iptables table 'raw': Table does not exist

**Fehler:**
- Seeed Jetson Kernel fehlen iptables raw Tabellen-Module
- Docker versucht Netzwerk-Isolation mit iptables zu aktivieren
- Build schlägt fehl: can't initialize iptables table 'raw': Table does not exist

**Lösung:**
- Docker-Daemon-Konfiguration erstellen: `/etc/docker/daemon.json`
- iptables, userland-proxy und ip-forward deaktivieren
- Docker-Service neustarten
- Build funktioniert wieder

**Kommandos:**
```bash
sudo tee /etc/docker/daemon.json <<EOF
{
  "iptables": false,
  "userland-proxy": false,
  "ip-forward": false
}
EOF
sudo systemctl restart docker
```

**Ursache:**
- Seeed Jetson hat minimalistische Kernel-Konfiguration ohne vollständige Netfilter-Unterstützung

#### GPU wird nicht genutzt, nur CPU – Docker mit NVIDIA-Runtime konfigurieren

i. Fehler:
- Docker verwendet standardmäßig die CPU, GPU bleibt ungenutzt
- ROS2/MoveIt läuft langsam, keine Hardware-Beschleunigung

ii. Lösung:
- Docker-Daemon-Konfiguration anpassen: `/etc/docker/daemon.json`
- NVIDIA-Runtime eintragen und Pfad setzen
- Docker-Service neu starten

iii. Kommandos:
```bash
sudo systemctl stop docker
sudo nano /etc/docker/daemon.json
# Inhalt:
{
  "iptables": false,
  "userland-proxy": false,
  "ip-forward": false,
  "runtimes": {
    "nvidia": {
      "path": "/usr/bin/nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
sudo systemctl restart docker
```

iv. Ursache:
- Standardmäßig ist die NVIDIA-Runtime nicht aktiviert
- Erst nach Anpassung wird die GPU für Docker-Container genutzt
- Danach läuft alles mit GPU-Unterstützung
