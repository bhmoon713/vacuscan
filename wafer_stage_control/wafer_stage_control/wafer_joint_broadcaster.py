#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState

class WaferJointBroadcaster(Node):
    def __init__(self):
        super().__init__('wafer_joint_broadcaster')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Pose2D, '/wafer/pose', self.on_pose, 10)
        self.last_pose = Pose2D()
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def on_pose(self, msg: Pose2D):
        self.last_pose = msg

    def tick(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['x_slide', 'y_slide']
        js.position = [self.last_pose.x, self.last_pose.y]
        self.pub.publish(js)

def main():
    rclpy.init()
    rclpy.spin(WaferJointBroadcaster())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
