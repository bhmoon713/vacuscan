#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class CaptureController(Node):
    def __init__(self):
        super().__init__('capture_controller')

        # Create the service
        self.capture_srv = self.create_service(
            Trigger,
            '/wafer/capture',
            self.handle_capture
        )

        self.get_logger().info("📸 CaptureController started: /wafer/capture service ready")

    def handle_capture(self, request, response):
        self.get_logger().info("📸 Capture requested — taking image or measurement...")

        # --------------------------------------
        # TODO: Insert your real capture logic!
        # e.g., trigger Python OpenCV camera
        # or send a message to an image node
        # --------------------------------------

        response.success = True
        response.message = "Capture OK"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CaptureController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
