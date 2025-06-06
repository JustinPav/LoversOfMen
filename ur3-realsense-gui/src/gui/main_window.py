from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
import sys

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UR3 and RealSense Control")
        self.setGeometry(100, 100, 300, 300)  # Adjusted height to fit the new buttons

        layout = QVBoxLayout()

        self.reset_ur3_button = QPushButton("Reset UR3")
        self.reset_ur3_button.clicked.connect(self.reset_ur3)

        self.move_to_target_button = QPushButton("Move to Target")
        self.move_to_target_button.clicked.connect(self.move_to_target)
        
        self.move_ur3_button = QPushButton("Move to Goal")
        self.move_ur3_button.clicked.connect(self.move_ur3)

        self.start_realsense_button = QPushButton("Start RealSense")
        self.start_realsense_button.clicked.connect(self.start_realsense)

        # Changed "Stop RealSense Camera" to "Capture Image"
        self.capture_image_button = QPushButton("Capture Image")
        self.capture_image_button.clicked.connect(self.capture_image)

        # Adding the gripper buttons
        self.open_gripper_button = QPushButton("Open Gripper")
        self.open_gripper_button.clicked.connect(self.open_gripper)

        self.close_gripper_button = QPushButton("Close Gripper")
        self.close_gripper_button.clicked.connect(self.close_gripper)

        # Adding the E-Stop button
        self.estop_button = QPushButton("E-Stop")
        self.estop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.estop_button.clicked.connect(self.estop)

        layout.addWidget(self.start_realsense_button)
        layout.addWidget(self.capture_image_button)  # Add the Capture Image button
        layout.addWidget(self.reset_ur3_button)
        layout.addWidget(self.move_to_target_button)
        layout.addWidget(self.move_ur3_button)
        layout.addWidget(self.open_gripper_button)  # Add the Open Gripper button
        layout.addWidget(self.close_gripper_button)  # Add the Close Gripper button
        layout.addWidget(self.estop_button)  # Add the E-Stop button to the layout

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def reset_ur3(self):
        print("Resetting UR3...")

    def move_to_target(self):
        print("Moving to Target...")
    def move_ur3(self):
        print("Moving UR3 to Goal...")

    def start_realsense(self):
        print("Starting RealSense Camera...")

    def capture_image(self):
        print("Capturing Image...")

    def estop(self):
        print("Emergency Stop Activated!")
        self.stop_ur3()
        self.stop_realsense()

    def close_gripper(self):
        print("Closing Gripper...")

    def open_gripper(self):
        print("Opening Gripper...")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())