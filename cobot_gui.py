"""
Cobot Bediener-GUI — Streamlit
ZirkulEA · UR3e + Jetson Orin NX · ROS 2 Humble
"""
import subprocess
import threading

import streamlit as st

st.set_page_config(
    page_title="Cobot — ZirkulEA",
    layout="wide",
    initial_sidebar_state="collapsed",
)

# ─── ROS2 Import (tolerant falls nicht verfügbar) ────────────────────────────

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# ─── State-Konstanten ────────────────────────────────────────────────────────

# GUI-Zustände
IDLE              = "IDLE"
EXECUTING_STORAGE = "EXECUTING_STORAGE"
SCANNING          = "SCANNING"
EXECUTING_STOW    = "EXECUTING_STOW"
WARN_HORIZONTAL   = "WARN_HORIZONTAL"
WARN_INACCURATE   = "WARN_INACCURATE"
WARN_FAILED       = "WARN_FAILED"

PLIER_LABELS = {
    "breitv1_clean_direction": "Breite Zange",
    "langv1_clean_direction":  "Lange Zange",
    "kurzv3_clean_direction":  "Kurze Zange",
}

# ─── ROS2 Node ───────────────────────────────────────────────────────────────

class CobotGuiNode(Node):
    def __init__(self):
        super().__init__("cobot_gui_node")

        # Publisher
        self.scan_trigger_pub    = self.create_publisher(Bool,   "/gui/scan_trigger",    10)
        self.plier_selection_pub = self.create_publisher(String, "/gui/plier_selection", 10)

        # Eingehende Status-Nachrichten (thread-safe über Lock)
        self._lock = threading.Lock()
        self._robot_status     = None
        self._detection_status = None

        self.create_subscription(String, "/gui/robot_status",      self._on_robot_status,     10)
        self.create_subscription(String, "/tool_detection_status", self._on_detection_status, 10)

    def _on_robot_status(self, msg: String):
        with self._lock:
            self._robot_status = msg.data

    def _on_detection_status(self, msg: String):
        with self._lock:
            self._detection_status = msg.data

    def pop_robot_status(self):
        with self._lock:
            val = self._robot_status
            self._robot_status = None
            return val

    def pop_detection_status(self):
        with self._lock:
            val = self._detection_status
            self._detection_status = None
            return val

    def publish_plier_selection(self, template_id: str):
        msg = String()
        msg.data = template_id
        self.plier_selection_pub.publish(msg)

    def publish_scan_trigger(self):
        msg = Bool()
        msg.data = True
        self.scan_trigger_pub.publish(msg)


def get_node() -> "CobotGuiNode | None":
    """Singleton-Pattern: Node wird einmal pro Session erstellt."""
    if not ROS2_AVAILABLE:
        return None
    try:
        if "ros_node" not in st.session_state:
            if not rclpy.ok():
                rclpy.init()
            st.session_state.ros_node = CobotGuiNode()
        node: CobotGuiNode = st.session_state.ros_node
        rclpy.spin_once(node, timeout_sec=0.05)
        return node
    except Exception as e:
        st.sidebar.warning(f"ROS2 nicht verfügbar: {e}")
        return None


# ─── Session State initialisieren ────────────────────────────────────────────

def _d(key, val):
    if key not in st.session_state:
        st.session_state[key] = val

_d("state",       IDLE)
_d("active_plier", None)   # Template-ID der laufenden Aktion
_d("warn_shown",  False)   # Verhindert doppeltes Dialog-Öffnen


# ─── State-Machine Übergänge ─────────────────────────────────────────────────

def to_idle():
    st.session_state.state        = IDLE
    st.session_state.active_plier = None
    st.session_state.warn_shown   = False


def start_storage(template_id: str):
    st.session_state.state        = EXECUTING_STORAGE
    st.session_state.active_plier = template_id
    node = get_node()
    if node:
        node.publish_plier_selection(template_id)


def start_scan():
    st.session_state.state        = SCANNING
    st.session_state.active_plier = None
    node = get_node()
    if node:
        node.publish_scan_trigger()


def to_executing_stow():
    st.session_state.state = EXECUTING_STOW


def to_warn(warn_state: str):
    st.session_state.state      = warn_state
    st.session_state.warn_shown = False


# ─── ROS2-Statusverarbeitung (einmal pro Rerun) ───────────────────────────────

def process_ros_messages():
    node = get_node()
    if node is None:
        return

    robot_status = node.pop_robot_status()
    detection_status = node.pop_detection_status()
    current = st.session_state.state

    # Roboter-Feedback auswerten
    if robot_status:
        if current == EXECUTING_STORAGE:
            if robot_status == "success":
                to_idle()
                st.rerun()
            elif robot_status in ("failed", "gripper_opened"):
                to_warn(WARN_FAILED)
                st.rerun()
        elif current == EXECUTING_STOW:
            if robot_status == "success":
                to_idle()
                st.rerun()
            elif robot_status in ("failed", "gripper_opened"):
                to_warn(WARN_FAILED)
                st.rerun()

    # Kamera-Scan-Feedback auswerten
    if detection_status:
        if current == SCANNING:
            if detection_status == "TOOL_SCAN_OK":
                # Roboter übernimmt jetzt (ur3_grip_and_place), wartet auf robot_status
                to_executing_stow()
                st.rerun()
            elif detection_status == "TOOL_SCAN_NOT_FOUND":
                to_warn(WARN_INACCURATE)
                st.rerun()
            elif detection_status == "horizontal_warning":
                to_warn(WARN_HORIZONTAL)
                st.rerun()
            elif detection_status == "inaccurate":
                to_warn(WARN_INACCURATE)
                st.rerun()


# ─── Warn-Dialoge ────────────────────────────────────────────────────────────

@st.dialog("Hinweis")
def warn_horizontal_dialog():
    st.warning("**Bitte Zange schräg ablegen.**\n\nDie Zange liegt waagerecht — der Roboter kann sie so nicht greifen.")
    if st.button("OK", use_container_width=True, type="primary"):
        to_idle()
        st.rerun()


@st.dialog("Hinweis")
def warn_inaccurate_dialog():
    st.warning("**Erkennung ungenau — bitte Zange neu positionieren.**\n\nDer Roboter fährt zur Kameraposition und wartet.")
    if st.button("OK", use_container_width=True, type="primary"):
        to_idle()
        st.rerun()


@st.dialog("Hinweis")
def warn_failed_dialog():
    st.error("**Greifer geöffnet — bitte Zange neu platzieren.**\n\nDer Greifer wurde geöffnet. Bitte Zange korrekt ablegen und erneut versuchen.")
    if st.button("OK", use_container_width=True, type="primary"):
        to_idle()
        st.rerun()


# ─── CSS ─────────────────────────────────────────────────────────────────────

CSS = """
<style>
#MainMenu, footer, header, .stDeployButton { visibility: hidden; }
.block-container { padding: 1rem 2rem 1rem !important; max-width: 100% !important; }
.stApp { background: #F5F6F8; }

.status-box {
    border-radius: 10px; padding: 18px 22px;
    font-size: 18px; font-weight: 600; margin-bottom: 18px;
    display: flex; align-items: center; gap: 12px;
}
.status-idle    { background: #E8F4F1; border: 1.5px solid #009682; color: #00695C; }
.status-busy    { background: #FFF8E1; border: 1.5px solid #F9A825; color: #795548; }
.status-warn    { background: #FFF3E0; border: 1.5px solid #EF6C00; color: #BF360C; }
.status-error   { background: #FFEBEE; border: 1.5px solid #C62828; color: #B71C1C; }

div[data-testid="stButton"] button {
    border-radius: 8px !important; font-weight: 700 !important;
    font-size: 16px !important; height: 54px !important;
}
</style>
"""


# ─── Ausgabezeile ─────────────────────────────────────────────────────────────

def render_status_line():
    state = st.session_state.state
    plier = st.session_state.active_plier
    plier_label = PLIER_LABELS.get(plier, plier) if plier else ""

    if state == IDLE:
        cls, icon, text = "status-idle", "✅", "Bereit — Wählen Sie eine Zange zum Holen."
    elif state == EXECUTING_STORAGE:
        cls, icon, text = "status-busy", "⚙️", f"{plier_label} wird aus dem Lager geholt …"
    elif state == SCANNING:
        cls, icon, text = "status-busy", "🔍", "Scan läuft — Zange auf grauer Fläche wird erkannt …"
    elif state == EXECUTING_STOW:
        cls, icon, text = "status-busy", "⚙️", "Zange wird eingelagert …"
    elif state in (WARN_HORIZONTAL, WARN_INACCURATE, WARN_FAILED):
        cls, icon, text = "status-warn", "⚠️", "Bitte Hinweis im Dialogfenster bestätigen."
    else:
        cls, icon, text = "status-idle", "ℹ️", state

    st.markdown(
        f'<div class="status-box {cls}">{icon}&nbsp; {text}</div>',
        unsafe_allow_html=True,
    )


# ─── Haupt-Render ─────────────────────────────────────────────────────────────

# ROS2 Nachrichten verarbeiten
process_ros_messages()

# CSS
st.markdown(CSS, unsafe_allow_html=True)

# Warn-Dialoge öffnen (nur einmal auslösen)
state = st.session_state.state
if state == WARN_HORIZONTAL and not st.session_state.warn_shown:
    st.session_state.warn_shown = True
    warn_horizontal_dialog()
elif state == WARN_INACCURATE and not st.session_state.warn_shown:
    st.session_state.warn_shown = True
    warn_inaccurate_dialog()
elif state == WARN_FAILED and not st.session_state.warn_shown:
    st.session_state.warn_shown = True
    warn_failed_dialog()

# Ausgabezeile
render_status_line()

# Buttons
busy = st.session_state.state not in (IDLE,)

col_reboot, col_breit, col_lang, col_kurz, col_aufraeum = st.columns([1, 2, 2, 2, 2])

with col_reboot:
    if st.button("Reboot", key="btn_reboot", use_container_width=True, type="secondary"):
        subprocess.Popen([
            "bash", "-c",
            "docker restart demostation-ur3 ; "
            "docker restart demostation-d405 "
        ])
        to_idle()
        st.rerun()

with col_breit:
    if st.button("Breit", key="btn_breit", use_container_width=True,
                 type="primary", disabled=busy):
        start_storage("breitv1_clean_direction")
        st.rerun()

with col_lang:
    if st.button("Lang", key="btn_lang", use_container_width=True,
                 type="primary", disabled=busy):
        start_storage("langv1_clean_direction")
        st.rerun()

with col_kurz:
    if st.button("Kurz", key="btn_kurz", use_container_width=True,
                 type="primary", disabled=busy):
        start_storage("kurzv3_clean_direction")
        st.rerun()

with col_aufraeum:
    if st.button("Aufräumen", key="btn_aufraeum", use_container_width=True,
                 type="secondary", disabled=busy):
        start_scan()
        st.rerun()

# Auto-Rerun während Ausführung (wartet auf ROS2-Feedback)
if st.session_state.state in (EXECUTING_STORAGE, SCANNING, EXECUTING_STOW):
    import time
    time.sleep(0.3)
    st.rerun()
