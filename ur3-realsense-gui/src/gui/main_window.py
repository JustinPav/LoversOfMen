from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
import sys

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UR3 and RealSense Control")
        self.setGeometry(100, 100, 300, 200)

        layout = QVBoxLayout()

        self.start_ur3_button = QPushButton("Start UR3 Robot")
        self.start_ur3_button.clicked.connect(self.start_ur3)

        self.stop_ur3_button = QPushButton("Stop UR3 Robot")
        self.stop_ur3_button.clicked.connect(self.stop_ur3)

        self.start_realsense_button = QPushButton("Start RealSense Camera")
        self.start_realsense_button.clicked.connect(self.start_realsense)

        self.stop_realsense_button = QPushButton("Stop RealSense Camera")
        self.stop_realsense_button.clicked.connect(self.stop_realsense)

        layout.addWidget(self.start_ur3_button)
        layout.addWidget(self.stop_ur3_button)
        layout.addWidget(self.start_realsense_button)
        layout.addWidget(self.stop_realsense_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def start_ur3(self):
        print("Starting UR3 Robot...")

    def stop_ur3(self):
        print("Stopping UR3 Robot...")

    def start_realsense(self):
        print("Starting RealSense Camera...")

    def stop_realsense(self):
        print("Stopping RealSense Camera...")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())