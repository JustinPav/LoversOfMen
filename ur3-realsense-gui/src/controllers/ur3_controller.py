class UR3Controller:
    def __init__(self):
        # Initialize the UR3 robot connection and state
        self.robot_connected = False
        self.robot_status = "Idle"

    def start_robot(self):
        # Code to start the UR3 robot
        if not self.robot_connected:
            print("Connecting to UR3 robot...")
            self.robot_connected = True
            self.robot_status = "Running"
            print("UR3 robot started.")
        else:
            print("UR3 robot is already running.")

    def stop_robot(self):
        # Code to stop the UR3 robot
        if self.robot_connected:
            print("Stopping UR3 robot...")
            self.robot_connected = False
            self.robot_status = "Stopped"
            print("UR3 robot stopped.")
        else:
            print("UR3 robot is not running.")

    def monitor_robot(self):
        # Code to monitor the UR3 robot's status
        if self.robot_connected:
            print(f"UR3 robot status: {self.robot_status}")
        else:
            print("UR3 robot is not connected.")

    def execute_process(self, process_name):
        # Code to execute a specific process on the UR3 robot
        if self.robot_connected:
            print(f"Executing process '{process_name}' on UR3 robot...")
            # Add logic to execute the process
            print(f"Process '{process_name}' completed.")
        else:
            print("Cannot execute process. UR3 robot is not connected.")