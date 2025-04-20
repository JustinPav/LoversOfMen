class RealSenseController:
    def __init__(self, realsense_service):
        self.realsense_service = realsense_service

    def start_camera(self):
        self.realsense_service.start_capture()

    def stop_camera(self):
        self.realsense_service.stop_capture()

    def get_camera_status(self):
        return self.realsense_service.get_status()