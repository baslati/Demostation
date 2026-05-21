"""
Cobot Bediener-GUI — Streamlit-Implementierung
ZirkulEA · UR3e + Jetson Orin NX · ROS 2 Humble
"""
import streamlit as st
import time
from datetime import datetime

st.set_page_config(
    page_title="Cobot Bediener-GUI — ZirkulEA",
    layout="wide",
    initial_sidebar_state="collapsed",
)

# ─── Konstanten ──────────────────────────────────────────────────────────────

PLIERS_DEF = {
    "Z1": {
        "id": "Z1", "name": "Sprengringzange, gerade", "sub": "Innenringe — gerade",
        "desc": "Innensicherungsringe in Bohrungen", "color": "#1F4E79",
        "shape": "straight-inner", "slot_x": 245, "slot_y": 470,
    },
    "Z2": {
        "id": "Z2", "name": "Sprengringzange, gerade", "sub": "Außenringe — gerade",
        "desc": "Außensicherungsringe auf Wellen", "color": "#D88436",
        "shape": "straight-outer", "slot_x": 345, "slot_y": 472,
    },
    "Z3": {
        "id": "Z3", "name": "Sprengringzange, abgewinkelt 90°", "sub": "Innenringe — abgewinkelt",
        "desc": "Schwer zugängliche Innenringe", "color": "#9A6FCB",
        "shape": "angled-inner", "slot_x": 445, "slot_y": 474,
    },
}

FETCH_PHASES = [
    {"key": "plan",  "label": "Bewegung wird geplant …",            "s": 1.6, "bar": 25, "robot_status": "planning",  "gripper": "open"},
    {"key": "exec",  "label": "Bewegung wird ausgeführt …",          "s": 2.0, "bar": 65, "robot_status": "executing", "gripper": "open"},
    {"key": "hand",  "label": "Zange übergeben — bitte entnehmen.",  "s": 1.8, "bar": 95, "robot_status": "handover",  "gripper": "closed"},
]
STOW_PHASES = [
    {"key": "plan",  "label": "Aufräumbahn wird geplant …",           "s": 1.6, "bar": 25, "robot_status": "planning",  "gripper": "open"},
    {"key": "pick",  "label": "Zange wird von Tisch aufgenommen …",   "s": 1.8, "bar": 55, "robot_status": "executing", "gripper": "closed"},
    {"key": "place", "label": "Zange wird in Ablage zurückgelegt …",  "s": 1.8, "bar": 90, "robot_status": "executing", "gripper": "open"},
]

ARUCO = '''<pattern id="aruco" width="16" height="16" patternUnits="userSpaceOnUse">
  <rect width="16" height="16" fill="#fff"/>
  <path d="M0 0h4v4H0zM12 0h4v4h-4zM4 4h4v4H4zM8 4h4v4H8zM0 8h4v4H0zM12 8h4v4h-4zM4 12h4v4H4zM8 12h4v4H8z" fill="#000"/>
</pattern>'''

# ─── State-Initialisierung ───────────────────────────────────────────────────

def _d(key, val):
    if key not in st.session_state:
        st.session_state[key] = val

_d("pliers", {k: {**v, "location": "in_ablage"} for k, v in PLIERS_DEF.items()})
_d("tab", "request")
_d("robot_status", "idle")  # idle | planning | executing | handover | stopped
_d("gripper", "open")
_d("busy", False)
_d("estop", False)
_d("log", [
    {"ts": datetime.now().strftime("%H:%M:%S"), "sev": "ok",
     "text": "System bereit. UR3e · RealSense · Greifer · ArUco verbunden."},
    {"ts": datetime.now().strftime("%H:%M:%S"), "sev": "info",
     "text": "Drei Zangen detektiert: Z1, Z2, Z3 — alle in Ablage."},
])
_d("confirm_id", None)
_d("confirm_mode", None)
_d("step_text", "Warte auf Bedienereingabe.")
_d("step_bar", 0)
_d("target_text", "—")
_d("seq_start", None)
_d("seq_schedule", [])
_d("seq_id", None)
_d("seq_mode", None)
_d("seq_prev_phase", -1)
_d("auto_tisch_id", None)
_d("auto_tisch_time", None)

# ─── Hilfsfunktionen ────────────────────────────────────────────────────────

def log_event(text, sev="info"):
    ts = datetime.now().strftime("%H:%M:%S")
    st.session_state.log.insert(0, {"ts": ts, "sev": sev, "text": text})
    st.session_state.log = st.session_state.log[:30]


def trigger_estop():
    st.session_state.estop = True
    st.session_state.robot_status = "stopped"
    st.session_state.busy = False
    st.session_state.seq_start = None
    st.session_state.step_bar = 0
    st.session_state.step_text = "Not-Halt ausgelöst — System zurücksetzen erforderlich."
    st.session_state.target_text = "—"
    log_event("NOT-HALT ausgelöst. Alle Aktoren freigeschaltet.", "err")


def reset_system():
    st.session_state.estop = False
    st.session_state.robot_status = "idle"
    st.session_state.gripper = "open"
    st.session_state.step_text = "Warte auf Bedienereingabe."
    st.session_state.step_bar = 0
    log_event("System wurde zurückgesetzt. Bereit.", "ok")


def start_sequence(pid, mode):
    phases = FETCH_PHASES if mode == "fetch" else STOW_PHASES
    cum = 0.0
    sched = []
    for ph in phases:
        cum += ph["s"]
        sched.append((cum, ph))
    st.session_state.seq_start = time.time()
    st.session_state.seq_schedule = sched
    st.session_state.seq_id = pid
    st.session_state.seq_mode = mode
    st.session_state.seq_prev_phase = -1
    st.session_state.busy = True
    p = st.session_state.pliers[pid]
    st.session_state.target_text = (
        f"slot-{pid} → werker" if mode == "fetch" else f"tisch {pid} → slot-{pid}"
    )
    log_event(f"Auftrag: {'Hole' if mode == 'fetch' else 'Räume ein'} {pid} ({p['sub']}).", "info")
    log_event(f"MoveIt: planning trajectory to {pid}.", "info")


def advance_sequence():
    """Aktuellen Sequenz-Schritt anhand der verstrichenen Zeit ermitteln."""
    if not st.session_state.busy or st.session_state.seq_start is None:
        return False
    elapsed = time.time() - st.session_state.seq_start
    sched = st.session_state.seq_schedule
    pid = st.session_state.seq_id
    mode = st.session_state.seq_mode

    cur_idx, cur_ph = -1, None
    for i, (end_t, ph) in enumerate(sched):
        if elapsed < end_t:
            cur_idx, cur_ph = i, ph
            break

    if cur_ph is None:
        _finalize_sequence(pid, mode)
        return False

    if cur_idx != st.session_state.seq_prev_phase:
        st.session_state.seq_prev_phase = cur_idx
        k = cur_ph["key"]
        if k == "exec":   log_event(f"UR3e: executing trajectory ({pid}).", "info")
        elif k == "pick":  log_event(f"Greifer: schließt um {pid} (ArUco match).", "info")
        elif k == "place": log_event(f"Greifer: löst {pid} in Slot, verifiziert.", "info")
        elif k == "hand":  log_event(f"Übergabe-Pose erreicht. Bitte {pid} entnehmen.", "ok")

    st.session_state.robot_status = cur_ph["robot_status"]
    st.session_state.gripper = cur_ph["gripper"]
    st.session_state.step_text = cur_ph["label"]
    st.session_state.step_bar = cur_ph["bar"]
    return True


def _finalize_sequence(pid, mode):
    p = st.session_state.pliers[pid]
    if mode == "fetch":
        p["location"] = "beim_werker"
        st.session_state.gripper = "open"
        log_event(f"{pid} an Werker übergeben. Greifer offen.", "ok")
        st.session_state.auto_tisch_id = pid
        st.session_state.auto_tisch_time = time.time() + 4.5
    else:
        p["location"] = "in_ablage"
        st.session_state.gripper = "open"
        log_event(f"{pid} in Ablage abgelegt. Slot verifiziert.", "ok")
    st.session_state.robot_status = "idle"
    st.session_state.step_text = "Warte auf Bedienereingabe."
    st.session_state.step_bar = 0
    st.session_state.target_text = "—"
    st.session_state.busy = False
    st.session_state.seq_start = None


def check_auto_tisch():
    if (st.session_state.auto_tisch_id and st.session_state.auto_tisch_time
            and time.time() > st.session_state.auto_tisch_time):
        pid = st.session_state.auto_tisch_id
        if st.session_state.pliers[pid]["location"] == "beim_werker":
            st.session_state.pliers[pid]["location"] = "auf_tisch"
            log_event(f"{pid} via Kamera erkannt: auf Tisch-Ablagefläche.", "info")
        st.session_state.auto_tisch_id = None
        st.session_state.auto_tisch_time = None
        return True
    return False


# ─── SVG-Generatoren ────────────────────────────────────────────────────────

def _pliers_paths(shape):
    if shape == "straight-outer":
        return (
            '<path d="M14 86 L40 56"/><path d="M30 90 L50 64"/>'
            '<path d="M50 58 L78 36"/><path d="M54 62 L84 44"/>'
            '<path d="M78 36 l4 -6"/><path d="M84 44 l6 -2"/>'
        )
    if shape == "straight-inner":
        return (
            '<path d="M14 86 L40 56"/><path d="M30 90 L50 64"/>'
            '<path d="M50 58 L78 36"/><path d="M54 62 L84 44"/>'
            '<path d="M78 36 l-2 -6"/><path d="M84 44 l-6 -2"/>'
        )
    return (  # angled-inner
        '<path d="M14 86 L40 58"/><path d="M30 90 L50 64"/>'
        '<path d="M50 58 L72 42 L72 26"/><path d="M54 62 L78 48 L80 30"/>'
        '<path d="M72 26 l-4 -2"/><path d="M80 30 l4 -3"/>'
    )


def pliers_icon_svg(shape, color, w=80, h=80):
    paths = _pliers_paths(shape)
    pivot = f'<circle cx="50" cy="58" r="4" fill="{color}"/>'
    return (
        f'<svg viewBox="0 0 100 100" width="{w}" height="{h}" fill="none" '
        f'stroke="{color}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round">'
        f'{paths}{pivot}</svg>'
    )


def pliers_mini_g(shape, color):
    paths = _pliers_paths(shape)
    pivot = f'<circle cx="50" cy="58" r="4" fill="{color}"/>'
    return (
        f'<g stroke="{color}" stroke-width="1.5" fill="none" stroke-linecap="round" '
        f'transform="translate(-22 0) scale(0.45)">{paths}{pivot}</g>'
    )


def build_rviz_svg(pliers, estop):
    busy = st.session_state.busy
    seq_id = st.session_state.seq_id
    seq_mode = st.session_state.seq_mode

    # Trajektorie
    traj_html = ""
    if busy and seq_id:
        p_def = PLIERS_DEF[seq_id]
        ax, ay = 150, 220
        bx, by = (p_def["slot_x"], p_def["slot_y"] - 12) if seq_mode == "fetch" else (590, 500)
        cx, cy = (ax + bx) / 2, min(ay, by) - 90
        d = f"M {ax} {ay} Q {cx} {cy} {bx} {by}"
        traj_html = (
            f'<path d="{d}" stroke="#7AE3D2" stroke-width="2" fill="none" stroke-linecap="round"'
            f' stroke-dasharray="6 6" class="traj-anim" marker-end="url(#arrowEnd)"/>'
            f'<circle cx="{ax}" cy="{ay}" r="4" fill="#7AE3D2"/>'
            f'<circle cx="{bx}" cy="{by}" r="5" fill="none" stroke="#7AE3D2" stroke-width="2"/>'
            f'<circle cx="{bx}" cy="{by}" r="9" fill="none" stroke="#7AE3D2" stroke-width="1"'
            f' opacity=".4" class="pulse-anim"/>'
        )

    # Slot-Visualisierungen
    slot_data = [("Z1", 245, 470, "#3D6CB0"), ("Z2", 345, 472, "#D88436"), ("Z3", 445, 474, "#9A6FCB")]
    slot_vis = ""
    for sid, sx, sy, sc in slot_data:
        p = pliers[sid]
        inner = pliers_mini_g(p["shape"], p["color"]) if p["location"] == "in_ablage" else ""
        slot_vis += (
            f'<g transform="translate({sx} {sy})">'
            f'<rect x="-34" y="-22" width="68" height="44" fill="#1B2230" stroke="{sc}" stroke-width="1.5" rx="3"/>'
            f'<rect x="-30" y="-18" width="14" height="14" fill="url(#aruco)"/>'
            f'<text x="22" y="-8" font-size="9" fill="{sc}" font-family="ui-monospace,monospace" opacity=".8">{sid}</text>'
            f'{inner}</g>'
        )

    # Zangen auf dem Tisch
    on_table = ""
    i = 0
    for sid, p in pliers.items():
        if p["location"] == "auf_tisch":
            tx, ty = 560 + i * 70, 500
            rot = 15 if i % 2 else -12
            on_table += (
                f'<g transform="translate({tx} {ty}) rotate({rot})">'
                f'<rect x="-30" y="-18" width="14" height="14" fill="url(#aruco)"/>'
                f'<text x="22" y="-8" font-size="9" fill="{p["color"]}" font-family="ui-monospace,monospace">{sid}</text>'
                f'{pliers_mini_g(p["shape"], p["color"])}</g>'
            )
            i += 1

    # E-Stop Overlay
    estop_html = ""
    if estop:
        estop_html = (
            '<rect x="0" y="0" width="960" height="620" fill="rgba(200,16,46,0.12)"/>'
            '<rect x="6" y="6" width="948" height="608" fill="none" stroke="#C8102E"'
            ' stroke-width="3" stroke-dasharray="10 6"/>'
            '<text x="480" y="50" text-anchor="middle" font-family="Inter,sans-serif"'
            ' font-size="20" font-weight="800" fill="#FF6F84" letter-spacing="3">SYSTEM GESTOPPT</text>'
        )

    return f"""
<style>
  @keyframes dash  {{ to {{ stroke-dashoffset: -40; }} }}
  @keyframes pulse {{ 0%,100%{{ opacity:.6 }} 50%{{ opacity: 1 }} }}
  .traj-anim  {{ animation: dash 1.2s linear infinite; }}
  .pulse-anim {{ animation: pulse 1.6s ease-in-out infinite; }}
</style>
<svg viewBox="0 0 960 620" style="width:100%;border-radius:10px;display:block">
  <defs>
    <linearGradient id="floorFade" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#161A21"/><stop offset="1" stop-color="#0c0f14"/>
    </linearGradient>
    <linearGradient id="tableTop" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#3A414E"/><stop offset="1" stop-color="#2A313D"/>
    </linearGradient>
    <linearGradient id="workzone" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#FFD24A" stop-opacity="0.20"/>
      <stop offset="1" stop-color="#FFD24A" stop-opacity="0.05"/>
    </linearGradient>
    {ARUCO}
    <marker id="arrowEnd" viewBox="0 0 10 10" refX="6" refY="5" markerWidth="6" markerHeight="6" orient="auto">
      <path d="M0 0L10 5L0 10Z" fill="#7AE3D2"/>
    </marker>
  </defs>
  <rect width="960" height="620" fill="url(#floorFade)"/>
  <!-- Boden-Gitter -->
  <g stroke="#2A313D" stroke-width="1">
    <line x1="0" y1="370" x2="960" y2="370" stroke="#384050"/>
    <line x1="40" y1="410" x2="920" y2="410"/>
    <line x1="80" y1="455" x2="880" y2="455"/>
    <line x1="120" y1="505" x2="840" y2="505"/>
    <line x1="160" y1="560" x2="800" y2="560"/>
    <line x1="480" y1="320" x2="0"   y2="620"/>
    <line x1="480" y1="320" x2="120" y2="620"/>
    <line x1="480" y1="320" x2="240" y2="620"/>
    <line x1="480" y1="320" x2="360" y2="620"/>
    <line x1="480" y1="320" x2="480" y2="620"/>
    <line x1="480" y1="320" x2="600" y2="620"/>
    <line x1="480" y1="320" x2="720" y2="620"/>
    <line x1="480" y1="320" x2="840" y2="620"/>
    <line x1="480" y1="320" x2="960" y2="620"/>
  </g>
  <!-- Achsen-Triade -->
  <g transform="translate(120 440)">
    <line x1="0" y1="0" x2="40" y2="6" stroke="#FF5A4E" stroke-width="2"/>
    <line x1="0" y1="0" x2="20" y2="-22" stroke="#7CE07A" stroke-width="2"/>
    <line x1="0" y1="0" x2="0" y2="-34" stroke="#6AA7FF" stroke-width="2"/>
    <circle r="3" fill="#fff"/>
    <text x="44" y="8" font-size="10" fill="#FF5A4E" font-family="ui-monospace,monospace">x</text>
    <text x="22" y="-22" font-size="10" fill="#7CE07A" font-family="ui-monospace,monospace">y</text>
    <text x="3" y="-36" font-size="10" fill="#6AA7FF" font-family="ui-monospace,monospace">z</text>
  </g>
  <!-- Tisch -->
  <polygon points="180,420 780,420 880,560 80,560" fill="url(#tableTop)" stroke="#4A5160" stroke-width="1.5"/>
  <polygon points="500,432 740,432 820,540 480,540" fill="url(#workzone)"
           stroke="#FFD24A" stroke-opacity="0.35" stroke-dasharray="4 4"/>
  <text x="640" y="490" text-anchor="middle" font-family="ui-monospace,monospace"
        font-size="11" fill="#E8C76B" opacity=".85">Arbeitsbereich Werker</text>
  <!-- Slots -->
  {slot_vis}
  <!-- Tisch-Ablage -->
  {on_table}
  <!-- Roboter UR3e (schematisch) -->
  <g transform="translate(150 320)">
    <ellipse cx="0" cy="100" rx="40" ry="10" fill="#0e1218" stroke="#2A313D"/>
    <rect x="-26" y="60" width="52" height="42" rx="5" fill="#262C38" stroke="#3A414E"/>
    <rect x="-22" y="64" width="44" height="6" fill="#1A1F2B"/>
    <rect x="-14" y="20" width="28" height="46" rx="6" fill="#2F3645" stroke="#454C5A"/>
    <circle cx="0" cy="22" r="11" fill="#1A1F2B" stroke="#5A6273"/>
    <circle cx="0" cy="22" r="3" fill="#7AE3D2"/>
    <g transform="rotate(-30 0 22)">
      <rect x="-9" y="-78" width="18" height="100" rx="6" fill="#373E4D" stroke="#4A5160"/>
      <circle cx="0" cy="-78" r="9" fill="#1A1F2B" stroke="#5A6273"/>
    </g>
    <g transform="translate(0 -67) rotate(55)">
      <rect x="-7" y="-90" width="14" height="92" rx="5" fill="#3D4554" stroke="#4A5160"/>
      <circle cx="0" cy="-90" r="7" fill="#1A1F2B" stroke="#5A6273"/>
      <g transform="translate(0 -100)">
        <rect x="-10" y="0" width="20" height="10" rx="2" fill="#444C5C" stroke="#5A6273"/>
        <rect x="-9" y="10" width="3" height="16" fill="#7A8294"/>
        <rect x="6"  y="10" width="3" height="16" fill="#7A8294"/>
      </g>
    </g>
    <text x="46" y="-110" font-family="ui-monospace,monospace" font-size="10" fill="#7AE3D2">tcp · /tool0</text>
    <line x1="0" y1="-105" x2="44" y2="-110" stroke="#7AE3D2" stroke-width="1" stroke-dasharray="2 2"/>
  </g>
  <!-- Trajektorie -->
  {traj_html}
  <!-- E-Stop Overlay -->
  {estop_html}
  <!-- Info-Overlay -->
  <g style="pointer-events:none">
    <text x="16" y="26" font-size="11" fill="#A9B3C2" font-family="ui-monospace,monospace">RViz · /world</text>
    <text x="16" y="42" font-size="10" fill="#A9B3C2" font-family="ui-monospace,monospace">frame_id: <tspan fill="white">base_link</tspan></text>
    <text x="16" y="58" font-size="10" fill="#A9B3C2" font-family="ui-monospace,monospace">planner: <tspan fill="white">RRTConnect</tspan></text>
    <text x="820" y="26" font-size="10" fill="#A9B3C2" font-family="ui-monospace,monospace">topic: <tspan fill="white">/joint_states</tspan></text>
  </g>
  <!-- Legende -->
  <g transform="translate(16 608)" style="pointer-events:none">
    <line x1="0" y1="0" x2="12" y2="0" stroke="#FF5A4E" stroke-width="2"/>
    <text x="16" y="4" font-size="10" fill="#A9B3C2" font-family="ui-monospace,monospace">X</text>
    <line x1="40" y1="0" x2="52" y2="0" stroke="#7CE07A" stroke-width="2"/>
    <text x="56" y="4" font-size="10" fill="#A9B3C2" font-family="ui-monospace,monospace">Y</text>
    <line x1="80" y1="0" x2="92" y2="0" stroke="#6AA7FF" stroke-width="2"/>
    <text x="96" y="4" font-size="10" fill="#A9B3C2" font-family="ui-monospace,monospace">Z</text>
    <rect x="128" y="-6" width="12" height="12" fill="#FFD24A" opacity=".4"/>
    <text x="144" y="4" font-size="10" fill="#A9B3C2" font-family="ui-monospace,monospace">workspace</text>
  </g>
</svg>"""


# ─── CSS ────────────────────────────────────────────────────────────────────

CSS = """
<style>
#MainMenu, footer, header, .stDeployButton { visibility: hidden; }
.block-container { padding: 0.5rem 1.5rem 1rem !important; max-width: 100% !important; }
.stApp { background: #F5F6F8; }

/* Header */
.cb-header {
    background: white; border-bottom: 1px solid #E2E5EA;
    padding: 10px 0 10px 0; display: flex; align-items: center; gap: 14px;
    margin-bottom: 12px;
}
.cb-logo {
    width: 36px; height: 36px; background: #1A1F2B; color: white;
    border-radius: 6px; display: inline-flex; align-items: center; justify-content: center;
    font-family: monospace; font-size: 11px; font-weight: 700; letter-spacing: 1px;
}
.cb-title { font-size: 15px; font-weight: 600; color: #1A1F2B; line-height: 1.2; }
.cb-sub   { font-size: 12px; color: #6C7280; }
.cb-leds  { display: flex; align-items: center; gap: 18px; margin-left: 12px; flex-wrap: wrap; }
.cb-led   { display: flex; align-items: center; gap: 6px; font-size: 12px; color: #6C7280; }
.dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; flex-shrink: 0; }
.dot-green { background: #009682; box-shadow: 0 0 0 3px rgba(0,150,130,.15); }
.dot-red   { background: #C8102E; box-shadow: 0 0 0 3px rgba(200,16,46,.18); }
.dot-amber { background: #E8A500; box-shadow: 0 0 0 3px rgba(232,165,0,.18); }
.dot-blue  { background: #1F4E79; box-shadow: 0 0 0 3px rgba(31,78,121,.15); }
.dot-gray  { background: #9AA3B0; }

/* Panel */
.cb-panel {
    background: white; border: 1px solid #E2E5EA; border-radius: 12px; overflow: hidden;
}
.cb-panel-body { padding: 14px; }

/* Pliers Card */
.pc {
    background: white; border: 1px solid #E2E5EA; border-radius: 10px;
    padding: 14px 14px 12px 14px; display: flex; gap: 14px;
    position: relative; overflow: hidden; margin-bottom: 10px;
}
.pc.disabled { opacity: .5; }
.pc-stripe { position: absolute; top: 0; left: 0; right: 0; height: 4px; }
.pc-iconbox {
    width: 88px; min-width: 88px; height: 88px;
    background: #F7F8FA; border: 1px solid #E2E5EA; border-radius: 8px;
    display: flex; align-items: center; justify-content: center; flex-shrink: 0;
}
.pc-body   { flex: 1; display: flex; flex-direction: column; justify-content: space-between; padding: 2px 0; min-width: 0; }
.pc-badge  { font-family: monospace; font-size: 11px; padding: 2px 7px; border-radius: 4px; font-weight: 700; }
.pc-name   { font-size: 15px; font-weight: 600; color: #1A1F2B; margin: 3px 0 1px; }
.pc-sub    { font-size: 13px; color: #6C7280; }
.pc-desc   { font-size: 12px; color: #6C7280; margin-top: 2px; }
.pc-foot   { display: flex; align-items: center; justify-content: space-between; margin-top: 10px; flex-wrap: wrap; gap: 6px; }
.loc-pill  {
    display: inline-flex; align-items: center; gap: 6px;
    padding: 3px 10px; border-radius: 999px;
    background: #F7F8FA; font-size: 12px; font-weight: 600; color: #1A1F2B;
}

/* Status */
.st-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; margin-bottom: 10px; }
.st-cell { border: 1px solid #E2E5EA; border-radius: 8px; padding: 12px; background: white; }
.st-label { font-size: 11px; text-transform: uppercase; letter-spacing: .05em; color: #6C7280; font-weight: 600; }
.st-val   { font-size: 15px; font-weight: 600; color: #1A1F2B; margin-top: 7px; display: flex; align-items: center; gap: 6px; }
.st-mono  { font-family: monospace; }
.st-step  { border: 1px solid #E2E5EA; border-radius: 8px; padding: 12px; background: #FAFBFC; }
.st-step-label { font-size: 11px; text-transform: uppercase; letter-spacing: .05em; color: #6C7280; font-weight: 600; }
.st-step-text { font-size: 14px; color: #1A1F2B; margin: 6px 0 10px; }
.prog-track { height: 6px; background: #EEF0F4; border-radius: 3px; overflow: hidden; }
.prog-bar   { height: 100%; background: #009682; border-radius: 3px; transition: width .3s ease; }

/* Log */
.log-entry {
    display: grid; grid-template-columns: 80px 60px 1fr;
    gap: 8px; padding: 5px 0; border-bottom: 1px dashed #EEF0F4;
    font-family: monospace; font-size: 12px; align-items: baseline;
}
.log-entry:last-child { border-bottom: none; }
.log-ts { color: #6C7280; }
.sev-ok   { color: #009682; font-weight: 700; }
.sev-info { color: #1F4E79; font-weight: 700; }
.sev-warn { color: #E8A500; font-weight: 700; }
.sev-err  { color: #C8102E; font-weight: 700; }

/* Streamlit widget overrides */
div[data-testid="stButton"] button[kind="secondary"] {
    border-radius: 8px; font-weight: 600;
}
</style>
"""


# ─── Dialog ─────────────────────────────────────────────────────────────────

@st.dialog("Aktion bestätigen")
def confirm_dialog():
    pid = st.session_state.confirm_id
    mode = st.session_state.confirm_mode
    if not pid:
        st.rerun()
        return
    p = st.session_state.pliers[pid]
    is_req = (mode == "fetch")
    icon = "→" if is_req else "←"
    title = f"{pid} — {p['sub']} {'holen lassen' if is_req else 'zurück in Ablage räumen'}?"
    body = (
        f"Der Cobot fährt die Ablage an, schließt den Greifer um **{pid}** und übergibt sie am Werker-Standpunkt."
        if is_req else
        f"Der Cobot lokalisiert **{pid}** via ArUco auf dem Tisch und legt sie zurück in den definierten Slot."
    )
    cmd = f"/ur3e/fetch_pliers {pid}" if is_req else f"/ur3e/stow_pliers {pid}"

    st.markdown(
        f'<div style="display:flex;align-items:center;gap:12px;margin-bottom:12px">'
        f'<div style="width:48px;height:48px;border-radius:50%;background:rgba(0,150,130,.1);'
        f'color:#009682;display:flex;align-items:center;justify-content:center;font-size:22px;flex-shrink:0">'
        f'{icon}</div>'
        f'<div><div style="font-size:17px;font-weight:700">{title}</div>'
        f'<div style="font-size:13px;color:#6C7280;margin-top:3px">{body}</div></div></div>',
        unsafe_allow_html=True,
    )
    st.code(
        f"cmd: {cmd}\nframe: base_link\nplanner: RRTConnect · timeout 5.0 s",
        language=None,
    )

    col_a, col_b = st.columns(2)
    with col_a:
        if st.button("Abbrechen", use_container_width=True):
            st.session_state.confirm_id = None
            st.session_state.confirm_mode = None
            st.rerun()
    with col_b:
        if st.button("Bestätigen", type="primary", use_container_width=True):
            start_sequence(pid, mode)
            st.session_state.confirm_id = None
            st.session_state.confirm_mode = None
            st.rerun()


# ─── Render-Hilfsfunktionen ──────────────────────────────────────────────────

def loc_pill(location):
    dot_cls = {"in_ablage": "dot-green", "beim_werker": "dot-blue", "auf_tisch": "dot-amber"}.get(location, "dot-gray")
    label    = {"in_ablage": "in Ablage", "beim_werker": "beim Werker", "auf_tisch": "auf Tisch"}.get(location, location)
    return f'<span class="loc-pill"><span class="dot {dot_cls}"></span>{label}</span>'


def render_pliers_card(p, tab, busy, estop):
    is_req = (tab == "request")
    if is_req:
        disabled = p["location"] != "in_ablage"
        reason = ("Zange ist beim Werker" if p["location"] == "beim_werker" else
                  "Nicht in Ablage" if disabled else "")
        action_label = "Holen →"
        action_mode = "fetch"
    else:
        disabled = p["location"] != "auf_tisch"
        reason = ("Bereits in Ablage" if p["location"] == "in_ablage" else
                  "Zange ist beim Werker" if p["location"] == "beim_werker" else "")
        action_label = "Aufräumen →"
        action_mode = "stow"

    if busy or estop:
        disabled = True

    dis_cls = "disabled" if disabled else ""
    icon_svg = pliers_icon_svg(p["shape"], p["color"], w=76, h=76)
    color = p["color"]

    action_html = (
        f'<span style="color:#C8102E;font-size:12px">{reason}</span>'
        if disabled else
        f'<span style="color:#009682;font-weight:600;font-size:13px">{action_label}</span>'
    )

    st.markdown(
        f'''<div class="pc {dis_cls}">
          <div class="pc-stripe" style="background:{color}"></div>
          <div class="pc-iconbox">{icon_svg}</div>
          <div class="pc-body">
            <div>
              <span class="pc-badge" style="background:{color}20;color:{color}">{p["id"]}</span>
              <div class="pc-name">{p["name"]}</div>
              <div class="pc-sub">{p["sub"]}</div>
              <div class="pc-desc">{p["desc"]}</div>
            </div>
            <div class="pc-foot">
              {loc_pill(p["location"])}
              {action_html}
            </div>
          </div>
        </div>''',
        unsafe_allow_html=True,
    )

    if not disabled:
        btn_key = f"select_{p['id']}_{tab}"
        if st.button(f"{p['id']} auswählen", key=btn_key, use_container_width=True):
            st.session_state.confirm_id = p["id"]
            st.session_state.confirm_mode = action_mode
            st.rerun()


def render_status_panel():
    rs = st.session_state.robot_status
    gripper = st.session_state.gripper
    target = st.session_state.target_text
    step = st.session_state.step_text
    bar = st.session_state.step_bar

    dot_map = {
        "idle": "dot-green", "planning": "dot-amber", "executing": "dot-amber",
        "handover": "dot-blue", "stopped": "dot-red", "error": "dot-red",
    }
    label_map = {
        "idle": "Idle", "planning": "Planung läuft", "executing": "Bewegung läuft",
        "handover": "Übergabe", "stopped": "GESTOPPT", "error": "Fehler",
    }
    dot_cls = dot_map.get(rs, "dot-gray")
    rs_label = label_map.get(rs, rs)
    grip_label = "Geschlossen" if gripper == "closed" else "Offen"

    st.markdown(
        f'''<div class="st-grid">
          <div class="st-cell">
            <div class="st-label">Roboter</div>
            <div class="st-val"><span class="dot {dot_cls}"></span>{rs_label}</div>
          </div>
          <div class="st-cell">
            <div class="st-label">Greifer</div>
            <div class="st-val">{grip_label}</div>
          </div>
          <div class="st-cell">
            <div class="st-label">Ziel</div>
            <div class="st-val st-mono" style="font-size:13px">{target}</div>
          </div>
        </div>
        <div class="st-step">
          <div class="st-step-label">Aktueller Schritt</div>
          <div class="st-step-text">{step}</div>
          <div class="prog-track"><div class="prog-bar" style="width:{bar}%"></div></div>
        </div>''',
        unsafe_allow_html=True,
    )


def render_log():
    entries = st.session_state.log[:8]
    rows = "".join(
        f'<div class="log-entry">'
        f'<span class="log-ts">{e["ts"]}</span>'
        f'<span class="sev-{e["sev"]}">{e["sev"].upper()}</span>'
        f'<span>{e["text"]}</span>'
        f'</div>'
        for e in entries
    )
    st.markdown(
        f'<div class="cb-panel"><div class="cb-panel-body">'
        f'<div style="display:flex;justify-content:space-between;margin-bottom:8px">'
        f'<span style="font-size:11px;text-transform:uppercase;letter-spacing:.05em;font-weight:700;color:#6C7280">Ereignisprotokoll</span>'
        f'<span style="font-size:11px;color:#6C7280">Letzte 8 Einträge</span></div>'
        f'<div style="max-height:130px;overflow-y:auto">{rows}</div>'
        f'</div></div>',
        unsafe_allow_html=True,
    )


# ─── Haupt-Render ────────────────────────────────────────────────────────────

# Sequenz vorantreiben + auto-Tisch prüfen
still_running = advance_sequence()
check_auto_tisch()

# CSS einfügen
st.markdown(CSS, unsafe_allow_html=True)

# Dialog anzeigen wenn nötig
if st.session_state.confirm_id:
    confirm_dialog()

# ── Header ──
now = datetime.now()
clock_str = now.strftime("%H:%M:%S · %d.%m.%Y")
led_cls = "dot-red" if st.session_state.estop else "dot-green"

st.markdown(
    f'''<div class="cb-header">
      <span class="cb-logo">wbk</span>
      <div>
        <div class="cb-title">Cobot-Assistenzsystem · Sprengringzangen-Bereitstellung</div>
        <div class="cb-sub">ZirkulEA · UR3e + Jetson Orin NX · ROS 2 Humble</div>
      </div>
      <div class="cb-leds">
        <div class="cb-led"><span class="dot {led_cls}"></span>UR3e</div>
        <div class="cb-led"><span class="dot {led_cls}"></span>RealSense</div>
        <div class="cb-led"><span class="dot {led_cls}"></span>Greifer</div>
        <div class="cb-led"><span class="dot {led_cls}"></span>ArUco</div>
      </div>
      <div style="margin-left:auto;text-align:right">
        <div style="font-family:monospace;font-size:12px;color:#6C7280">{clock_str}</div>
        <div style="font-size:11px;color:#6C7280">Werkstattzelle 04 · Schicht F</div>
      </div>
    </div>''',
    unsafe_allow_html=True,
)

# ── E-Stop / Reset-Leiste ──
h_col1, h_col2, h_spacer = st.columns([1.4, 1.4, 7])
with h_col1:
    if st.button("⛔  NOT-HALT", key="estop_btn", use_container_width=True,
                 type="primary" if not st.session_state.estop else "secondary",
                 disabled=st.session_state.estop):
        trigger_estop()
        st.rerun()
with h_col2:
    if st.session_state.estop:
        if st.button("↺  System zurücksetzen", key="reset_btn", use_container_width=True):
            reset_system()
            st.rerun()

st.markdown("<div style='height:4px'></div>", unsafe_allow_html=True)

# ── Haupt-Layout ──
col_rviz, col_ctrl = st.columns([3, 2], gap="medium")

with col_rviz:
    rviz_svg = build_rviz_svg(st.session_state.pliers, st.session_state.estop)
    st.markdown(
        f'<div style="border-radius:12px;overflow:hidden;border:1px solid #0d1015;'
        f'background:#161A21">{rviz_svg}</div>',
        unsafe_allow_html=True,
    )

with col_ctrl:
    pliers = st.session_state.pliers
    tab = st.session_state.tab
    busy = st.session_state.busy
    estop = st.session_state.estop

    # ── Tabs ──
    st.markdown('<div class="cb-panel">', unsafe_allow_html=True)
    tab_r_active = "active" if tab == "request" else ""
    tab_s_active = "active" if tab == "return" else ""
    st.markdown(
        f'''<div style="display:flex;background:#FAFBFC;border-bottom:1px solid #E2E5EA;padding:0 6px">
          <div style="padding:12px 14px;font-size:14px;font-weight:600;
            color:{'#1A1F2B' if tab == 'request' else '#6C7280'};
            border-bottom:{'3px solid #009682' if tab == 'request' else '3px solid transparent'};
            display:flex;align-items:center;gap:8px">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M5 12h14M13 6l6 6-6 6"/>
            </svg>Zange anfordern</div>
          <div style="padding:12px 14px;font-size:14px;font-weight:600;
            color:{'#1A1F2B' if tab == 'return' else '#6C7280'};
            border-bottom:{'3px solid #009682' if tab == 'return' else '3px solid transparent'};
            display:flex;align-items:center;gap:8px">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M19 12H5M11 18l-6-6 6-6"/>
            </svg>Zange zurücklegen</div>
        </div>''',
        unsafe_allow_html=True,
    )

    t_col1, t_col2 = st.columns(2)
    with t_col1:
        if st.button("→ Zange anfordern", key="tab_req_btn", use_container_width=True,
                     disabled=busy, type="secondary"):
            st.session_state.tab = "request"
            st.rerun()
    with t_col2:
        if st.button("← Zange zurücklegen", key="tab_ret_btn", use_container_width=True,
                     disabled=busy, type="secondary"):
            st.session_state.tab = "return"
            st.rerun()

    # Titel + Zangen-Kacheln
    is_req = (tab == "request")
    mode_badge = "REQUEST" if is_req else "STOW"
    title = "Welche Zange soll der Cobot bringen?" if is_req else "Welche Zange soll der Cobot aufräumen?"
    subtitle = ("Drei Sprengringzangen sind auf der Ablage definiert." if is_req else
                "Nur Zangen, die aktuell auf dem Tisch liegen, können zurückgelegt werden.")

    st.markdown(
        f'''<div style="padding:14px 14px 6px">
          <div style="display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:10px">
            <div>
              <div style="font-size:17px;font-weight:700;color:#1A1F2B">{title}</div>
              <div style="font-size:12px;color:#6C7280;margin-top:2px">{subtitle}</div>
            </div>
            <span style="background:#F1F2F5;color:#6C7280;font-family:monospace;font-size:11px;
              padding:4px 10px;border-radius:999px;font-weight:700;flex-shrink:0;margin-left:10px">{mode_badge}</span>
          </div>''',
        unsafe_allow_html=True,
    )

    for pid in ["Z1", "Z2", "Z3"]:
        render_pliers_card(pliers[pid], tab, busy, estop)

    st.markdown('</div></div>', unsafe_allow_html=True)  # close cb-panel-body + cb-panel

    # ── Status-Panel ──
    st.markdown("<div style='height:8px'></div>", unsafe_allow_html=True)
    st.markdown(
        '<div class="cb-panel"><div class="cb-panel-body">'
        '<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px">'
        '<span style="font-size:13px;font-weight:700;text-transform:uppercase;letter-spacing:.05em;color:#6C7280">Systemstatus</span>'
        f'<span style="font-family:monospace;font-size:12px;color:#6C7280">t = {(time.time() % 10000):.1f} s</span>'
        '</div>',
        unsafe_allow_html=True,
    )
    render_status_panel()
    st.markdown('</div></div>', unsafe_allow_html=True)

# ── Log ──
st.markdown("<div style='height:8px'></div>", unsafe_allow_html=True)
render_log()

# ── Auto-Rerun wenn Sequenz läuft oder auto-Tisch ausstehend ──
needs_rerun = (
    st.session_state.busy
    or (st.session_state.auto_tisch_time and time.time() < st.session_state.auto_tisch_time)
)
if needs_rerun:
    time.sleep(0.35)
    st.rerun()
