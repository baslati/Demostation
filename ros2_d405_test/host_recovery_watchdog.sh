#!/bin/bash
# Laeuft auf dem HOST (nicht im Container). Erledigt zwei Dinge:
#
# 1) Faengt den Fall ab, den der In-Container-Watchdog (camera_watchdog.sh)
#    nicht loesen kann: die D405 ist laut `lsusb` wieder eingesteckt, aber der
#    Container sieht den neuen USB-Geraeteknoten nicht (Docker mappt
#    /dev/bus/usb nur beim Container-Start, Hotplug-Aenderungen werden nicht
#    automatisch uebernommen). Loesung: d405-Container killen und ueber
#    start_test.sh neu starten, genau wie bei einem manuellen Klick auf
#    D405_Test_Start.desktop.
#
# 2) Wartet auf eine Reboot-Anfrage aus der GUI (cobot_gui.py, Reboot-Button).
#    Die GUI laeuft im d405-Container und kann Docker/Host nicht direkt
#    ansprechen, deshalb legt sie stattdessen eine Trigger-Datei in einem
#    bind-gemounteten Ordner an. Sobald dieser Watchdog die Datei sieht,
#    stoppt er BEIDE Container (d405 + ur3) sauber und startet beide ueber
#    ihre .desktop-Kommandos neu (D405_Test_Start.desktop / Demostation_Start.desktop) -
#    also ein sauberer Docker-Neustart statt des alten harten Kernel-Reboots
#    (echo b > /proc/sysrq-trigger), der den Roboter mitten in der Bewegung
#    hart ausschalten konnte.
#
# Das eigentliche Stoppen/Neustarten uebernimmt restart_stacks.sh, per
# "setsid nohup ... &" komplett von diesem Watchdog-Prozess losgeloest.
# Grund: Sobald der d405-Container entfernt wird, kehrt das aeussere
# start_test.sh aus seinem blockierenden "docker exec -it" zurueck und
# durchlaeuft seinen eigenen Aufraeum-Code, der u.a. den Watchdog beendet
# (siehe start_test.sh). Wuerde restart_stacks.sh als Teil DIESES Prozesses
# laufen, koennte genau das mitten in der Ausfuehrung passieren. Als
# losgeloester Prozess mit eigenem Namen/eigener Session ist restart_stacks.sh
# davon nicht betroffen.
#
# Wird automatisch am Ende von start_test.sh im Hintergrund gestartet.

set -u

CONTAINER=d405_test_container
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="/tmp/d405_host_watchdog.log"
PID_FILE="/tmp/host_recovery_watchdog.pid"
REBOOT_TRIGGER_FILE="$PROJECT_DIR/workspace/src/custom_code/.reboot_request"

CHECK_INTERVAL_SEC=5
STALE_THRESHOLD_SEC=20   # so lange muss "Geraet da, aber nicht gefunden" anhalten
COOLDOWN_SEC=60          # Mindestabstand zwischen zwei automatischen Neustarts

stale_since=0
last_restart=0

# PID in einer Datei hinterlegen, damit start_test.sh beim Aufraeumen gezielt
# GENAU DIESE Instanz beenden kann (statt per "pkill -f host_recovery_watchdog.sh"
# blind irgendeine/die falsche Instanz zu treffen, z.B. eine bereits neu
# gestartete).
echo $$ > "$PID_FILE"

echo "$(date '+%F %T') [HOST-WATCHDOG] gestartet (PID $$), ueberwache Container '$CONTAINER'" >> "$LOG_FILE"

while true; do
    now=$(date +%s)

    if [ -f "$REBOOT_TRIGGER_FILE" ]; then
        rm -f "$REBOOT_TRIGGER_FILE"
        echo "$(date '+%F %T') [HOST-WATCHDOG] Reboot-Anfrage aus GUI erhalten -> delegiere an restart_stacks.sh all" >> "$LOG_FILE"
        setsid nohup "$PROJECT_DIR/restart_stacks.sh" all >> "$LOG_FILE" 2>&1 < /dev/null &
        disown
        exit 0
    fi

    device_present=false
    if lsusb | grep -q "8086:0b5b"; then
        device_present=true
    fi

    last_line=$(docker exec "$CONTAINER" tail -n 1 /tmp/camera.log 2>/dev/null)

    if $device_present && echo "$last_line" | grep -q "No RealSense devices were found"; then
        if [ "$stale_since" -eq 0 ]; then
            stale_since=$now
        fi
    else
        stale_since=0
    fi

    if [ "$stale_since" -ne 0 ] \
        && [ $((now - stale_since)) -ge "$STALE_THRESHOLD_SEC" ] \
        && [ $((now - last_restart)) -ge "$COOLDOWN_SEC" ]; then

        echo "$(date '+%F %T') [HOST-WATCHDOG] Kamera seit ${STALE_THRESHOLD_SEC}s trotz USB-Reconnect nicht erkannt -> delegiere an restart_stacks.sh d405" >> "$LOG_FILE"
        setsid nohup "$PROJECT_DIR/restart_stacks.sh" d405 >> "$LOG_FILE" 2>&1 < /dev/null &
        disown

        last_restart=$now
        stale_since=0
    fi

    sleep "$CHECK_INTERVAL_SEC"
done
