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
        self.setGeometry(300, 300, 500, 350)

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        title = QLabel("AR4 Robot Control Panel")
        title.setFont(QFont("Courier", 16, QFont.Bold))
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

        launch_btn4 = QPushButton("🚀 Launch Robot Proceses (with Target Detect)")
        launch_btn4.clicked.connect(self.launch_robot_procesesV2)
        layout.addWidget(launch_btn4)

        prog1_btn = QPushButton("Run Cube Routine")
        prog1_btn.clicked.connect(self.run_cube_routine)
        layout.addWidget(prog1_btn)

        prog2_btn = QPushButton("Run Cube-Target Routine")
        prog2_btn.clicked.connect(self.run_cubeTarget_routine)
        layout.addWidget(prog2_btn)

        build_btn = QPushButton("Build the workspace")
        build_btn.clicked.connect(self.build)
        layout.addWidget(build_btn)

        estop_btn = QPushButton("Reset E-Stop")
        estop_btn.clicked.connect(self.estop_reset)
        layout.addWidget(estop_btn)

        posegui_btn = QPushButton("Pose GUI")
        posegui_btn.clicked.connect(self.pose_gui)
        layout.addWidget(posegui_btn)

        kill_term_btn = QPushButton("❌ Close All Terminals")
        kill_term_btn.clicked.connect(self.close_all_terminals)
        layout.addWidget(kill_term_btn)

        #ADD SHUTDOWN

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

    def launch_robot_procesesV2(self):
        self.run_in_terminal("ros2 launch programs_cpp robot_launchV2.py")

    def run_cube_routine(self):
        self.run_in_terminal("ros2 run programs_cpp cubeRoutine")

    def run_cubeTarget_routine(self):
        self.run_in_terminal("ros2 run programs_cpp cubeRoutineV2")

    def check_camera(self):
        self.run_in_terminal("v4l2-ctl --list-devices")

    def build(self):
        self.run_in_terminal("cd ~/ar4 && colcon build && source install/setup.bash")
    
    def pose_gui(self):
        self.run_in_terminal("ros2 run pose_gui pose_gui")

    def estop_reset(self):
        self.run_in_terminal("ros2 run annin_ar4_driver reset_estop.sh mk3")

    def close_all_terminals(self):
        try:
            subprocess.run(["pkill", "gnome-terminal"], check=True)
        except subprocess.CalledProcessError as e:
            QMessageBox.warning(self, "Warning", "No terminal processes to kill.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to kill terminal windows:\n{e}")




def main():
    # Make Ctrl+C work
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    window = RobotLauncher()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()