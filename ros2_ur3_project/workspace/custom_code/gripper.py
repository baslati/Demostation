import time
import rtde_control
import rtde_receive
import rtde_io

#Definitionen
RTDE_FREQUENCY = 50.0
RTDE_IP = "192.168.122.20"
RTDE_PORT = 63352

#Initialisierung
rtde_In_Out = rtde_io.RTDEIOInterface(RTDE_IP)

rtde_c = rtde_control.RTDEControlInterface(
    RTDE_IP,
    RTDE_FREQUENCY,
    rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP,
)

rtde_r = rtde_receive.RTDEReceiveInterface(RTDE_IP)

rtde_In_Out.setToolDigitalOut(0, 0)

##Test
#Close
rtde_In_Out.setToolDigitalOut(1, 0)
time.sleep(2)

#Open
rtde_In_Out.setToolDigitalOut(0, 1)
time.sleep(2)


