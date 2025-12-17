#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from vacuscan_interfaces.srv import WaferGoto


# Keep the old anchor points P so /wafer/lengths topic stays compatible
P = np.array([
    [-0.25, -0.25],
    [ 0.25, -0.25],
    [-0.25,  0.25],
    [ 0.25,  0.25],
])


class XYWaferCtrl(Node):
    """
    Controller for vacuscan stage using x_control_joint and y_control_joint.

    - Service: /wafer/goto (WaferGoto)
    - Subscribes: /joint_states (for x_control_joint, y_control_joint)
    - Publishes:
        /wafer/pose     (Pose2D with x,y from joints)
        /wafer/lengths  (Float32MultiArray, synthetic "wire" lengths)
        /x_joint_trajectory_controller/joint_trajectory
        /y_joint_trajectory_controller/joint_trajectory
    """

    def __init__(self):
        super().__init__("xy_wafer_ctrl")

        # --- Publishers for pose + synthetic lengths ---
        self.pose_pub = self.create_publisher(Pose2D, "/wafer/pose", 10)
        self.len_pub  = self.create_publisher(Float32MultiArray,
                                              "/wafer/lengths", 10)

        # --- Publishers for joint trajectory controllers ---
        self.x_traj_pub = self.create_publisher(
            JointTrajectory,
            "/x_joint_trajectory_controller/joint_trajectory",
            10
        )
        self.y_traj_pub = self.create_publisher(
            JointTrajectory,
            "/y_joint_trajectory_controller/joint_trajectory",
            10
        )

        # --- Service: /wafer/goto ---
        self.srv = self.create_service(WaferGoto, "/wafer/goto",
                                       self.on_goto)

        # --- Joint state subscription ---
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            10
        )

        # Current stage pose (x, y) from joints
        self.current_x = 0.0
        self.current_y = 0.0

        # Last goal (used mainly for logging / debugging)
        self.goal = None
        self.tol = 1e-4

        # Default speed [m/s] if req.speed == 0
        self.default_speed = 0.05

        self.get_logger().info("XYWaferCtrl node ready (using x/y joints).")

    # ---------------------------------------------------------
    # Joint state callback: update current_x/current_y and re-publish pose
    # ---------------------------------------------------------
    def on_joint_state(self, msg: JointState):
        try:
            # Find indices of the two prismatic joints
            ix = msg.name.index("x_control_joint")
            iy = msg.name.index("y_control_joint")
        except ValueError:
            # Joints not present in this message
            return

        # Update current stage pose from joint positions
        self.current_x = float(msg.position[ix])
        self.current_y = float(msg.position[iy])

        # Publish /wafer/pose
        pose = Pose2D()
        pose.x = self.current_x
        pose.y = self.current_y
        pose.theta = 0.0
        self.pose_pub.publish(pose)

        # Publish synthetic /wafer/lengths (so old UI doesn’t break)
        stage_xy = np.array([self.current_x, self.current_y])
        L = np.linalg.norm(stage_xy - P, axis=1)
        self.len_pub.publish(Float32MultiArray(data=L.tolist()))

    # ---------------------------------------------------------
    # Service handler for /wafer/goto
    # ---------------------------------------------------------
    def on_goto(self, req: WaferGoto.Request,
                resp: WaferGoto.Response) -> WaferGoto.Response:
        """
        Move to target (req.x, req.y) by sending joint trajectories.

        - req.x, req.y : target positions [meters]
        - req.speed     : desired speed [m/s] (0 → use default_speed)
        - req.tol       : tolerance (currently only stored, not enforced here)
        """
        target_x = float(req.x)
        target_y = float(req.y)

        # speed handling
        speed = float(req.speed) if req.speed > 0.0 else self.default_speed
        speed = max(1e-3, speed)  # avoid division by zero

        # Compute distance in each axis
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        max_delta = max(abs(dx), abs(dy))

        if max_delta < max(req.tol, 1e-4):
            resp.success = True
            resp.message = "Already within tolerance of goal."
            self.get_logger().info(
                f"[goto] Already at goal ({target_x:.4f}, {target_y:.4f})."
            )
            return resp

        # Rough duration = distance / speed (use max axis distance)
        duration_sec = max_delta / speed
        duration_sec = max(duration_sec, 0.1)  # minimum 0.1s

        # Save goal / tol for higher-level nodes (if needed)
        self.goal = (target_x, target_y)
        self.tol = max(1e-5, float(req.tol))

        self.get_logger().info(
            f"[goto] New goal: x={target_x:.4f}, y={target_y:.4f}, "
            f"speed={speed:.3f} m/s, est T={duration_sec:.2f}s"
        )

        # Build ROS Duration
        sec_i = int(math.floor(duration_sec))
        nsec_i = int((duration_sec - sec_i) * 1e9)
        traj_duration = Duration(sec=sec_i, nanosec=nsec_i)

        # --- X trajectory ---
        traj_x = JointTrajectory()
        traj_x.joint_names = ["x_control_joint"]
        pt_x = JointTrajectoryPoint()
        pt_x.positions = [target_x]
        pt_x.time_from_start = traj_duration
        traj_x.points.append(pt_x)

        # --- Y trajectory ---
        traj_y = JointTrajectory()
        traj_y.joint_names = ["y_control_joint"]
        pt_y = JointTrajectoryPoint()
        pt_y.positions = [target_y]
        pt_y.time_from_start = traj_duration
        traj_y.points.append(pt_y)

        # Publish both trajectories
        self.x_traj_pub.publish(traj_x)
        self.y_traj_pub.publish(traj_y)

        resp.success = True
        resp.message = "Goal accepted"
        self.get_logger().info(
            "[goto] Trajectories published to "
            "/x_joint_trajectory_controller/joint_trajectory and "
            "/y_joint_trajectory_controller/joint_trajectory"
        )
        return resp


def main():
    rclpy.init()
    node = XYWaferCtrl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
