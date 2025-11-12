#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# Corner coordinates (m) — CCW from TOP-RIGHT, same as URDF
C = np.array([
    [ 0.34,  0.34],  # corner_1
    [-0.34,  0.34],  # corner_2
    [-0.34, -0.34],  # corner_3
    [ 0.34, -0.34],  # corner_4
], dtype=float)

# Wafer anchor offsets (relative to wafer center), CCW from TOP-RIGHT
a = 0.152
A = np.array([
    [ a,  a],  # wafer_anchor_1
    [-a,  a],  # wafer_anchor_2
    [-a, -a],  # wafer_anchor_3
    [ a, -a],  # wafer_anchor_4
], dtype=float)

class WireViz(Node):
    def __init__(self):
        super().__init__('wafer_wire_visualizer')
        self.sub = self.create_subscription(Pose2D, '/wafer/pose', self.on_pose, 10)
        self.pub = self.create_publisher(MarkerArray, '/wafer/wires_markers', 10)
        self.current = Pose2D()
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        # allow overriding corners via params if you want later
        # (e.g., ros2 param set /wafer_wire_visualizer corner_half 0.3)

    def on_pose(self, p: Pose2D):
        self.current = p

    def tick(self):
        wa = np.array([self.current.x, self.current.y])
        ma = MarkerArray()

        # Common header
        frame = 'base'
        now = self.get_clock().now().to_msg()

        # Draw corner spheres and wafer sphere
        # corners
        # Wires: arrows + length text
        for i in range(4):
            start = C[i]              # corner_i (base frame)
            end   = wa + A[i]         # wafer_anchor_i (base frame)
            length = float(np.linalg.norm(end - start))

            # Arrow marker (start→end)
            arr = Marker()
            arr.header.frame_id = frame
            arr.header.stamp = now
            arr.ns = 'wires'
            arr.id = 200 + i
            arr.type = Marker.ARROW
            arr.action = Marker.ADD
            arr.scale.x = 0.004   # shaft diameter
            arr.scale.y = 0.010   # head diameter
            arr.scale.z = 0.020   # head length
            arr.color.r = 1.0; arr.color.g = 0.3; arr.color.b = 0.0; arr.color.a = 0.9
            arr.points = [Point(x=float(start[0]), y=float(start[1]), z=0.0),
                          Point(x=float(end[0]),   y=float(end[1]),   z=0.0)]
            arr.lifetime = Duration(sec=0)
            ma.markers.append(arr)

            # --- improved midpoint positioning for text ---
            vec = end - start
            mid = start + 0.5 * vec
            # normalized perpendicular direction for small offset (to avoid overlap)
            if np.linalg.norm(vec) > 1e-6:
                perp = np.array([-vec[1], vec[0]])
                perp /= np.linalg.norm(perp)
            else:
                perp = np.array([0.0, 0.0])
            # small perpendicular offset (10 mm)
            mid += 0.01 * perp

            txt = Marker()
            txt.header.frame_id = frame
            txt.header.stamp = now
            txt.ns = 'wire_lengths'
            txt.id = 300 + i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.scale.z = 0.025      # 30 mm text height
            txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 0.2; txt.color.a = 1.0
            txt.pose.orientation.w = 1.0
            txt.pose.position.x = float(mid[0])
            txt.pose.position.y = float(mid[1])
            txt.pose.position.z = 0.02              # just above plane
            # txt.text = f"{length*1000.0:.1f} mm"
            txt.text = f"{length*1000.0:.1f}"
            txt.lifetime = Duration(sec=0)
            ma.markers.append(txt)


        self.pub.publish(ma)

def main():
    rclpy.init()
    rclpy.spin(WireViz())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
