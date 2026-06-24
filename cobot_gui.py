"""
Cobot Bediener-GUI - Streamlit
ZirkulEA · UR3e + Jetson Orin NX · ROS 2 Humble
"""
import subprocess
import threading

import streamlit as st

st.set_page_config(
    page_title="Cobot - ZirkulEA",
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
HOMING_FOR_SCAN         = "HOMING_FOR_SCAN"
WARN_HORIZONTAL         = "WARN_HORIZONTAL"
WARN_DETECTION_FAILED   = "WARN_DETECTION_FAILED"
WARN_NO_PATH            = "WARN_NO_PATH"
WARN_TOLERANCE_VIOLATION = "WARN_TOLERANCE_VIOLATION"

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
        self.home_drive_pub      = self.create_publisher(Bool,   "/gui/home_drive",      10)

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

    def publish_home_drive(self):
        msg = Bool()
        msg.data = True
        self.home_drive_pub.publish(msg)


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
        import traceback
        print(f"[DEBUG] get_node() Exception: {e}\n{traceback.format_exc()}", flush=True)
        st.sidebar.warning(f"ROS2 nicht verfügbar: {e}")
        return None


# ─── Session State initialisieren ────────────────────────────────────────────

def _d(key, val):
    if key not in st.session_state:
        st.session_state[key] = val

_d("state",       IDLE)
_d("active_plier", None)   # Template-ID der laufenden Aktion


# ─── State-Machine Übergänge ─────────────────────────────────────────────────

def to_idle():
    st.session_state.state        = IDLE
    st.session_state.active_plier = None


def start_storage(template_id: str):
    st.session_state.state        = EXECUTING_STORAGE
    st.session_state.active_plier = template_id
    node = get_node()
    print(f"[DEBUG] start_storage: template={template_id}, node={'OK' if node else 'None'}", flush=True)
    if node:
        node.publish_plier_selection(template_id)
        print(f"[DEBUG] publish_plier_selection gesendet: {template_id}", flush=True)


def start_scan():
    st.session_state.state        = SCANNING
    st.session_state.active_plier = None
    node = get_node()
    if node:
        node.publish_scan_trigger()

def start_homing_for_scan():
    st.session_state.state        = HOMING_FOR_SCAN
    st.session_state.active_plier = None
    node = get_node()
    if node:
        node.publish_home_drive()


def to_executing_stow():
    st.session_state.state = EXECUTING_STOW


def to_warn(warn_state: str):
    st.session_state.state = warn_state


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
        if current == HOMING_FOR_SCAN and robot_status == "home_reached":
            start_scan()
            st.rerun()
        if current in (EXECUTING_STORAGE, EXECUTING_STOW, HOMING_FOR_SCAN):
            if robot_status == "success":
                to_idle()
                st.rerun()
            elif robot_status == "no_path":
                to_warn(WARN_NO_PATH)
                st.rerun()
            elif robot_status == "tolerance_violation":
                to_warn(WARN_TOLERANCE_VIOLATION)
                st.rerun()
            elif robot_status in ("failed",):
                to_warn(WARN_NO_PATH)
                st.rerun()

    # Kamera-Scan-Feedback auswerten
    if detection_status:
        if current == SCANNING:
            if detection_status == "TOOL_SCAN_OK":
                # Roboter übernimmt jetzt (ur3_grip_and_place), wartet auf robot_status
                to_executing_stow()
                st.rerun()
            elif detection_status == "TOOL_SCAN_NOT_FOUND":
                to_warn(WARN_DETECTION_FAILED)
                st.rerun()
            elif detection_status == "horizontal_warning":
                to_warn(WARN_HORIZONTAL)
                st.rerun()


# ─── CSS ─────────────────────────────────────────────────────────────────────

CSS = """
<style>
/* ── Grundlayout ── */
#MainMenu, footer, header, .stDeployButton { visibility: hidden; }
.block-container { padding: 1rem 2rem 1rem !important; max-width: 100% !important; }

/* App-Hintergrund: dezentes #F5F6F8 statt reinem Weiß,
   damit Statusboxen und Buttons visuell hervorstechen */
.stApp { background: #F5F6F8 !important; }

/* ── Schriftart ── */
.stApp, .stApp * {
    font-family: Arial, sans-serif !important;
}
h1, h2, h3 { color: #144466 !important; font-family: Arial, sans-serif !important; }

/* ── Statusboxen ── */
.status-box {
    border-radius: 10px; padding: 18px 22px;
    font-size: 18px; font-weight: 600; margin-bottom: 18px;
    display: flex; align-items: center; gap: 12px;
    font-family: Arial, sans-serif;
}

/* idle — Türkis/Petrol #009682, Hintergrund helles Teal */
.status-idle  { background: #EBF5F4; border: 1.5px solid #009682; color: #144466; }

/* busy — Warngelb #EEB70D, helles Gelb */
.status-busy  { background: #FEF8E0; border: 1.5px solid #EEB70D; color: #2C333E; }

/* warn — ebenfalls #EEB70D, etwas wärmerer Gelbton für Unterschied zu busy */
.status-warn  { background: #FFF3CC; border: 1.5px solid #EEB70D; color: #2C333E; }

/* error — Fehlerrot #B2372C, helles Rosa */
.status-error { background: #FDECEA; border: 1.5px solid #B2372C; color: #B2372C; }

/* ── Buttons ── */
div[data-testid="stButton"] button {
    border-radius: 8px !important;
    font-weight: 900 !important;
    font-size: 24px !important;
    height: 54px !important;
    font-family: Arial, sans-serif !important;
    background-color: #009682 !important;
    color: #FFFFFF !important;
    border: none !important;
}
div[data-testid="stButton"] button:hover {
    background-color: #00838F !important;
    color: #FFFFFF !important;
}
</style>
"""


# ─── Ausgabezeile ─────────────────────────────────────────────────────────────

WARN_MESSAGES = {
    WARN_HORIZONTAL:
        "Bitte legen Sie die Zange leicht schräg ab.",
    WARN_DETECTION_FAILED:
        "Die Zange konnte nicht korrekt erkannt werden, bitte verändern Sie die Position.",
    WARN_NO_PATH:
        "Die Zange liegt nicht im möglichen Arbeitsbereich, bitte legen Sie sie in der Mitte der grauen Fläche ab.",
    WARN_TOLERANCE_VIOLATION:
        "Positionserror des UR3e: Bitte bewegen Sie den Roboter manuell über das Teach-Pendant. "
        "Wenn die Remote-Kontrolle am UR3 wieder aktiviert ist, klicken Sie auf Zurücksetzen.",
}

def render_status_line():
    state = st.session_state.state
    plier = st.session_state.active_plier
    plier_label = PLIER_LABELS.get(plier, plier) if plier else ""

    if state == IDLE:
        cls, icon, text = "status-idle", "✅", "Bereit — Wenn die graue Fläche leer ist, wählen Sie eine Zange aus dem Lager. Oder lassen Sie eine Zange von der grauen Fläche aufräumen."
    elif state == EXECUTING_STORAGE:
        cls, icon, text = "status-busy", "⚙️", f"{plier_label} wird aus dem Lager geholt …"
    elif state == HOMING_FOR_SCAN:
        cls, icon, text = "status-busy", "⚙️", "Roboter fährt zur Kameraposition …"
    elif state == SCANNING:
        cls, icon, text = "status-busy", "🔍", "Scan läuft — Zange auf grauer Fläche wird erkannt …"
    elif state == EXECUTING_STOW:
        cls, icon, text = "status-busy", "⚙️", "Zange wird eingelagert …"
    elif state in WARN_MESSAGES:
        cls, icon, text = "status-error", "⚠️", WARN_MESSAGES[state]
    else:
        cls, icon, text = "status-idle", "ℹ️", state

    st.markdown(
        f'<div class="status-box {cls}">{icon}&nbsp; {text}</div>',
        unsafe_allow_html=True,
    )
    if state in WARN_MESSAGES:
        if st.button("✓ Zurücksetzen", key="btn_reset_warn", type="secondary"):
            if state == WARN_TOLERANCE_VIOLATION:
                node = get_node()
                if node:
                    node.publish_home_drive()
            to_idle()
            st.rerun()


# ─── Haupt-Render ─────────────────────────────────────────────────────────────

# ROS2 Nachrichten verarbeiten
process_ros_messages()

# CSS
st.markdown(CSS, unsafe_allow_html=True)

# Ausgabezeile
render_status_line()

# Buttons
busy = st.session_state.state not in (IDLE,)

col_reboot, col_breit, col_lang, col_kurz, col_aufraeum = st.columns([1, 2, 2, 2, 2])

with col_reboot:
    if st.button("Reboot", key="btn_reboot", use_container_width=True, type="secondary"):
        subprocess.Popen(["bash", "-c", "echo b > /proc/sysrq-trigger"])
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
        start_homing_for_scan()
        st.rerun()

# Auto-Rerun während Ausführung (wartet auf ROS2-Feedback)
if st.session_state.state in (EXECUTING_STORAGE, HOMING_FOR_SCAN, SCANNING, EXECUTING_STOW):
    import time
    time.sleep(0.3)
    st.rerun()
