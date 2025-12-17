#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray

# Geometry (meters) — must match your URDF and web UI
corner = 0.34
a = 0.152
C = np.array([[+corner, +corner],
              [-corner, +corner],
              [-corner, -corner],
              [+corner, -corner]], dtype=float)
A = np.array([[+a, +a],
              [-a, +a],
              [-a, -a],
              [+a, -a]], dtype=float)

class WireLengthPublisher(Node):
    def __init__(self):
        super().__init__('wire_length_publisher')
        self.pose = Pose2D()
        self.sub = self.create_subscription(Pose2D, '/wafer/pose', self.on_pose, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/wafer/wire_lengths', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def on_pose(self, msg: Pose2D):
        self.pose = msg

    def tick(self):
        wa = np.array([self.pose.x, self.pose.y], dtype=float)
        L = []
        for i in range(4):
            start = C[i]
            end   = wa + A[i]
            L.append(np.linalg.norm(end - start))  # meters
        msg = Float32MultiArray(data=L)
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(WireLengthPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
