#!/usr/bin/env python3

import subprocess
import time
import os

def main():
    print("Starting Plier Detection Test...")

    # Starte die RealSense-Kamera im Hintergrund
    print("Launching RealSense Camera...")
    camera_process = subprocess.Popen([
        'ros2', 'launch', 'pliers_detection', 'realsense_camera.launch.py'
    ])

    # Warte, damit die Kamera startet
    time.sleep(5)

    # Starte den PlierDetector-Node im Hintergrund
    print("Launching PlierDetector...")
    detector_process = subprocess.Popen([
        'ros2', 'run', 'pliers_detection', 'plier_detector.py'
    ])

    # Warte kurz
    time.sleep(2)

    # Starte RViz für Visualisierung
    print("Launching RViz...")
    rviz_process = subprocess.Popen([
        'ros2', 'run', 'rviz2', 'rviz2', '-d', '/workspace/src/pliers_detection/launch/pliers_detection.rviz'
    ])

    print("RealSense Camera, PlierDetector and RViz are running.")
    print("In RViz: Displays are pre-configured.")
    print("Press Ctrl+C to stop.")

    try:
        # Warte auf Beendigung
        camera_process.wait()
        detector_process.wait()
        rviz_process.wait()
    except KeyboardInterrupt:
        print("Stopping...")
        camera_process.terminate()
        detector_process.terminate()
        rviz_process.terminate()
        camera_process.terminate()
        detector_process.terminate()
        rviz_process.terminate()
        camera_process.wait()
        detector_process.wait()
        rviz_process.wait()
        print("Stopped.")

if __name__ == '__main__':
    main()
