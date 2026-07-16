#!/bin/bash
# Wird vom host_recovery_watchdog.sh per "setsid nohup ... &" komplett
# losgeloest gestartet (eigene Prozessgruppe/Session), damit dieses Skript
# NIE ein Opfer des Cleanup-Codes der alten start_test.sh/start_demostation.sh
# werden kann (die killen u.a. den Watchdog, aber eben nicht diesen Prozess
# hier - der hat einen anderen Namen und ist per setsid entkoppelt).
#
# Uebernimmt fuer BEIDE Faelle (Reboot-Button und automatische Kamera-
# Wiederherstellung) das komplette "Container weg, alte Prozesse weg, neue
# Terminals hoch" - vorher war das an zwei Stellen in host_recovery_watchdog.sh
# dupliziert und race-anfaellig (siehe Git-Historie dieser Datei).
#
# Aufruf: restart_stacks.sh all   -> ur3 + d405 (Reboot-Button)
#         restart_stacks.sh d405 -> nur d405 (Kamera-Watchdog)

set -u

MODE="${1:?Usage: restart_stacks.sh all|d405}"

CONTAINER=d405_test_container
UR3_CONTAINER=demostation-ur3
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UR3_DIR="$(cd "$PROJECT_DIR/../ros2_ur3_project" && pwd)"
LOG_FILE="/tmp/d405_host_watchdog.log"

log() {
    echo "$(date '+%F %T') [RESTART-STACKS:$MODE] $*" >> "$LOG_FILE"
}

# pkill -f, aber die eigene PID nie mittoeten (falls das Suchmuster jemals
# zufaellig auf die eigene Kommandozeile passen sollte).
safe_pkill() {
    local pattern="$1" pid
    for pid in $(pgrep -f "$pattern" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        kill "$pid" 2>/dev/null || true
    done
}

# Wartet bis zu 30s aktiv darauf, dass ein Container wirklich aus
# "docker ps -a" verschwunden ist, statt blind auf eine feste Zeit zu
# vertrauen.
wait_for_gone() {
    local name="$1" waited=0
    while docker ps -a --format '{{.Names}}' | grep -qx "$name"; do
        if [ "$waited" -ge 30 ]; then
            log "WARNUNG: $name nach 30s immer noch in 'docker ps -a' gelistet"
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 0
}

# gnome-terminal (D-Bus-Factory) startet gelegentlich ein Fenster nicht
# ("Error creating terminal: Failed to get screen from object path ..."),
# vor allem wenn kurz zuvor alle Fenster geschlossen wurden und dann kurz
# hintereinander mehrere neue Fenster angefordert werden. Deshalb hier
# grosszuegig Zeit lassen, bevor das naechste Fenster angefordert wird.
launch_terminal() {
    local label="$1" dir="$2" script="$3"
    gnome-terminal -- bash -c "cd '$dir' && ./$script; exec bash" >> "$LOG_FILE" 2>&1
    sleep 30
    if pgrep -f "$script" >/dev/null 2>&1; then
        log "$label gestartet"
    else
        log "FEHLER: $label nicht gestartet"
    fi
}

log "Start (PID $$)"

# --- a) Container stoppen/entfernen ------------------------------------
if [ "$MODE" = "all" ]; then
    docker stop "$UR3_CONTAINER" >> "$LOG_FILE" 2>&1
fi
docker rm -f "$CONTAINER" >> "$LOG_FILE" 2>&1

# --- b) aktiv auf tatsaechliches Verschwinden warten -------------------
if [ "$MODE" = "all" ]; then
    wait_for_gone "$UR3_CONTAINER"
fi
wait_for_gone "$CONTAINER"

# --- c) Karenz, damit die Cleanup-Bloecke der alten Skripte (deren
#        "docker exec -it" durch a) gerade zurueckgekehrt ist) durchlaufen
#        koennen, bevor wir gleich ihre Prozesse hart beenden -----------
sleep 4

# --- d) alte Terminal-Wrapper UND die eigentlichen Skriptprozesse
#        beenden (ein reines pkill auf ".../exec bash" trifft nur den
#        Wrapper, nicht den per fork gestarteten "./start_test.sh"-
#        Kindprozess - der wuerde sonst als Waise weiterlaufen) ---------
if [ "$MODE" = "all" ]; then
    safe_pkill "start_demostation.sh; exec bash"
    safe_pkill "\./start_demostation.sh"
fi
safe_pkill "start_test.sh; exec bash"
safe_pkill "\./start_test.sh"
safe_pkill "org.chromium.Chromium"

# --- e) neue Terminals starten -----------------------------------------
if [ "$MODE" = "all" ]; then
    launch_terminal "ur3 (start_demostation.sh)" "$UR3_DIR" "start_demostation.sh"
fi
launch_terminal "d405 (start_test.sh)" "$PROJECT_DIR" "start_test.sh"

log "fertig"
