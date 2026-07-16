import time
import rtde_control
import rtde_receive
import rtde_io

#Definitionen
RTDE_FREQUENCY = 50.0
RTDE_IP = "192.168.122.20"
#RTDE_PORT = 63352
RTDE_PORT = 50002

# Initialisierung
rtde_In_Out = rtde_io.RTDEIOInterface(RTDE_IP)
rtde_c = rtde_control.RTDEControlInterface(
    RTDE_IP,
    RTDE_FREQUENCY,
    rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP,
)

print("Starte Greifer-Testsequenz...")
rtde_In_Out.setToolDigitalOut(0, 0)

##Test
# Close
print("Schließe Greifer (setToolDigitalOut 1,0)...")
rtde_In_Out.setToolDigitalOut(1, 0)
time.sleep(0.2)
print("Setze ToolDigitalOut 0,0 (Greifer bleibt geschlossen)...")
rtde_In_Out.setToolDigitalOut(0, 0)
time.sleep(2)

# Open
print("Öffne Greifer (setToolDigitalOut 0,1)...")
rtde_In_Out.setToolDigitalOut(0, 1)
time.sleep(2)
print("Setze ToolDigitalOut 0,0 (Greifer bleibt offen)...")
rtde_In_Out.setToolDigitalOut(0, 0)
time.sleep(2)

print("Beende und trenne Verbindungen.")
rtde_c.disconnect()
rtde_In_Out.disconnect()