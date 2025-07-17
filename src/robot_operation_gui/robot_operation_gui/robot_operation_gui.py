# This code generate the GUI for launching necessary nodes, servers, and drivers for
#   the AR4 robot operation. 
# Made with aid of AI(GPT) by Kevin Arroyo Abreu

import sys
import signal
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QMessageBox
)
from PyQt5.QtGui import QFont, QIcon
from PyQt5.QtCore import Qt

class RobotLauncher(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AR4 Robot Operation GUI")
        self.setGeometry(300, 300, 400, 250)

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        title = QLabel("AR4 Robot Control Panel")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        launch_btn1 = QPushButton("Launch Robot Driver")
        launch_btn1.clicked.connect(self.launch_robot_driver)
        layout.addWidget(launch_btn1)

        launch_btn2 = QPushButton("Launch Robot Driver (No Calibration)")
        launch_btn2.clicked.connect(self.launch_robot_driver_no_calib)
        layout.addWidget(launch_btn2)

        launch_cam = QPushButton("Check Camera Ports")
        launch_cam.clicked.connect(self.check_camera)
        layout.addWidget(launch_cam)

        launch_btn3 = QPushButton("🚀 Launch Robot Proceses")
        launch_btn3.clicked.connect(self.launch_robot_proceses)
        layout.addWidget(launch_btn3)

        prog1_btn = QPushButton("🧊 Run Cube Routine")
        prog1_btn.clicked.connect(self.run_cube_routine)
        layout.addWidget(prog1_btn)

        prog2_btn = QPushButton("🆕 Run New Routine")
        prog2_btn.clicked.connect(self.run_new_routine)
        layout.addWidget(prog2_btn)

        #ADD SHUDOWN AND POSE GUI and BUILD ALL

        self.setLayout(layout)

    def run_in_terminal(self, command):
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'
            ])
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to launch command:\n{e}")

    def launch_robot_driver(self):
        self.run_in_terminal("ros2 launch annin_ar4_driver driver.launch.py calibrate:=True")

    def launch_robot_driver_no_calib(self):
        self.run_in_terminal("ros2 launch annin_ar4_driver driver.launch.py calibrate:=False")

    def launch_robot_proceses(self):
        self.run_in_terminal("ros2 launch programs_cpp robot_launch.py")

    def run_cube_routine(self):
        self.run_in_terminal("ros2 run programs_cpp cubeRoutine")

    def run_new_routine(self):
        self.run_in_terminal("ros2 run programs_cpp newRoutine")

    def check_camera(self):
        self.run_in_terminal("v4l2-ctl --list-devices")


def main():
    # Make Ctrl+C work
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    window = RobotLauncher()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()