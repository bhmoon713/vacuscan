#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
from wafer_stage_interfaces.srv import WaferGoto

P = np.array([[-0.25, -0.25],
              [ 0.25, -0.25],
              [-0.25,  0.25],
              [ 0.25,  0.25]])

class WaferCtrl(Node):
    def __init__(self):
        super().__init__('wafer_ctrl')
        self.pose_pub = self.create_publisher(Pose2D, '/wafer/pose', 10)
        self.len_pub  = self.create_publisher(Float32MultiArray, '/wafer/lengths', 10)
        self.cmd_pub  = self.create_publisher(Float32MultiArray, '/motor/cmd_velocity', 10)
        self.srv = self.create_service(WaferGoto, '/wafer/goto', self.on_goto)
        self.timer = self.create_timer(0.01, self.loop)  # 100 Hz
        self.x = np.array([0.0, 0.0])    # replace with fused sensors
        self.goal = None
        self.vmax = 0.05                 # m/s limit
        self.kp = 4.0
        self.tol = 5e-5

    def on_goto(self, req, resp):
        self.goal = np.array([req.x, req.y], dtype=float)
        self.vmax = float(req.speed) if req.speed > 0 else self.vmax
        self.tol = max(1e-5, float(req.tol))
        resp.success = True
        resp.message = "Goal accepted"
        return resp

    def loop(self):
        if self.goal is not None:
            err = self.goal - self.x
            if np.linalg.norm(err) < self.tol:
                self.goal = None
                self.publish_lengths_pose()
                return
            v_xy = np.clip(self.kp * err, -self.vmax, self.vmax)
            L = np.linalg.norm(self.x - P, axis=1)
            J = (self.x - P) / L[:, None]           # 4x2
            dL = J @ v_xy                            # 4x1
            self.cmd_pub.publish(Float32MultiArray(data=dL.tolist()))
            # TEMP simulate plant motion:
            self.x += v_xy * 0.01
        self.publish_lengths_pose()

    def publish_lengths_pose(self):
        pose = Pose2D(x=float(self.x[0]), y=float(self.x[1]), theta=0.0)
        self.pose_pub.publish(pose)
        L = np.linalg.norm(self.x - P, axis=1)
        self.len_pub.publish(Float32MultiArray(data=L.tolist()))

def main():
    rclpy.init()
    rclpy.spin(WaferCtrl())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
