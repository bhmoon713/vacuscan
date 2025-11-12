#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from threading import Thread, Lock
from std_srvs.srv import Trigger
from wafer_stage_interfaces.srv import RunWaypoints, WaferGoto
from geometry_msgs.msg import Pose2D
from ament_index_python.packages import get_package_share_directory
import yaml, time, os, math
import threading 


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('waypoint_runner')

        # Parameters
        bringup_share = get_package_share_directory('wafer_stage_bringup')
        default_file = os.path.join(bringup_share, 'config', 'waypoints.yaml')
        self.declare_parameter('waypoints_file', default_file)
        self.declare_parameter('pose_topic', '/wafer/pose')
        self.declare_parameter('goto_service', '/wafer/goto')

        # Internal state
        self.routes = {}
        self.current_route = None
        self.thread = None
        self.abort_flag = False
        self.lock = Lock()
        self.pose = Pose2D()

        # Load routes
        self._load_routes()

        # Subscriptions & service clients
        self.create_subscription(Pose2D, self.get_parameter('pose_topic').value, self._on_pose, 10)
        self.goto_cli = self.create_client(WaferGoto, self.get_parameter('goto_service').value)

        # Services
        self.run_srv = self.create_service(RunWaypoints, '/wafer/run_waypoints', self._on_run)
        self.abort_srv = self.create_service(Trigger, '/wafer/abort_waypoints', self._on_abort)

        self.get_logger().info(f'Loaded waypoint routes: {list(self.routes.keys())}')

    # ------------------------------------------------------------------
    # Load waypoints from YAML
    # ------------------------------------------------------------------
    def _load_routes(self):
        path = self.get_parameter('waypoints_file').value
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.routes = data.get('routes', {})
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints YAML: {e}')
            self.routes = {}

    # ------------------------------------------------------------------
    # Subscribers / Callbacks
    # ------------------------------------------------------------------
    def _on_pose(self, msg: Pose2D):
        with self.lock:
            self.pose = msg

    def _on_abort(self, req, res):
        self.get_logger().warn('Abort requested')
        self.abort_flag = True
        res.success = True
        res.message = 'Abort flag set'
        return res

    def _on_run(self, req, res):
        try:
            route_name = req.route  # <-- field name from .srv
            tol_mm     = req.tol_mm if req.tol_mm > 0 else self.declare_or_get('tol_mm', 30.0)
            timeout_s  = req.timeout_s if req.timeout_s > 0 else self.declare_or_get('timeout_s', 8.0)
            pause_s    = req.pause_s if req.pause_s >= 0 else self.declare_or_get('pause_s', 0.2)
            do_loop    = bool(req.loop)

            if route_name not in self.routes:
                res.accepted = False
                res.message  = f'Unknown route "{route_name}". Options: {list(self.routes.keys())}'
                self.get_logger().warn(res.message)
                return res

            pts = self.routes[route_name]
            self.get_logger().info(
                f'Running route "{route_name}" (N={len(pts)}) tol={tol_mm:.1f}mm '
                f'timeout={timeout_s:.1f}s pause={pause_s:.1f}s loop={do_loop}'
            )
            # launch in thread so service returns immediately
            self._stop_flag = False
            self._thread = threading.Thread(
                target=self._run_route,
                args=(pts, tol_mm, timeout_s, pause_s, do_loop),
                daemon=True
            )
            self._thread.start()

            res.accepted = True
            res.message  = 'Route started'
            return res
        except Exception as e:
            res.accepted = False
            res.message  = f'Exception: {e}'
            self.get_logger().error(res.message)
            return res

    # ------------------------------------------------------------------
    # Utility: flexible point normalization
    # ------------------------------------------------------------------
    def _norm_point(self, p):
        """Parse a YAML waypoint entry -> ('pause', s) or (x_mm, y_mm)."""
        # List or tuple
        if isinstance(p, (list, tuple)) and len(p) == 2:
            try:
                return float(p[0]), float(p[1])
            except Exception:
                return None

        if not isinstance(p, dict):
            return None

        # Direct forms
        if 'x_m' in p and 'y_m' in p:
            return float(p['x_m']) * 1000.0, float(p['y_m']) * 1000.0
        if 'x_mm' in p and 'y_mm' in p:
            return float(p['x_mm']), float(p['y_mm'])
        if 'x' in p and 'y' in p:
            return float(p['x']), float(p['y'])

        # Wrapped
        if 'goto' in p and isinstance(p['goto'], dict):
            g = p['goto']
            if 'x_m' in g and 'y_m' in g:
                return float(g['x_m']) * 1000.0, float(g['y_m']) * 1000.0
            if 'x_mm' in g and 'y_mm' in g:
                return float(g['x_mm']), float(g['y_mm'])
            if 'x' in g and 'y' in g:
                return float(g['x']), float(g['y'])

        # Pause
        if 'pause_s' in p:
            return ('pause', float(p['pause_s']))

        return None

    # ------------------------------------------------------------------
    # Core route runner
    # ------------------------------------------------------------------
    def _run_route(self, route_name: str, loop: bool):
        r = self.routes[route_name]
        pts = r.get('points', [])
        tol_mm = float(r.get('tol_mm', 50.0))
        timeout_s = float(r.get('timeout_s', 10.0))
        dwell_s = float(r.get('pause_s', 0.0))

        self.get_logger().info(
            f'Running route "{route_name}" (N={len(pts)}) tol={tol_mm}mm timeout={timeout_s}s pause={dwell_s}s loop={loop}'
        )

        keep_going = True
        while rclpy.ok() and keep_going and not self.abort_flag:
            for i, p in enumerate(pts):
                if self.abort_flag:
                    break

                parsed = self._norm_point(p)
                if parsed is None:
                    self.get_logger().warn(f'[{route_name}] step {i}: unrecognized {p}, skipping.')
                    continue

                # Pause
                if isinstance(parsed, tuple) and parsed[0] == 'pause':
                    _, ps = parsed
                    self.get_logger().info(f'[{route_name}] step {i}: pause {ps}s')
                    time.sleep(ps)
                    continue

                # Motion
                if isinstance(parsed, tuple) and len(parsed) == 2:
                    x_mm, y_mm = parsed
                    ok = self._goto_and_wait(x_mm, y_mm, tol_mm, timeout_s)
                    if not ok:
                        self.get_logger().warn(f'[{route_name}] step {i}: timeout/tolerance not met')
                    if dwell_s > 0:
                        time.sleep(dwell_s)

            keep_going = bool(loop)

        self.get_logger().info('Route finished')

    # ------------------------------------------------------------------
    # Goto + wait until position reached or timeout
    # ------------------------------------------------------------------
    def _goto_and_wait(self, x_mm: float, y_mm: float, tol_mm: float, timeout_s: float) -> bool:
        req = WaferGoto.Request()
        req.x = x_mm / 1000.0
        req.y = y_mm / 1000.0

        # Call service async, don’t break executor’s Future API
        if not self.goto_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('WaferGoto service unavailable!')
            return False

        fut = self.goto_cli.call_async(req)

        t0 = time.time()
        while rclpy.ok():
            if self.abort_flag:
                return False
            with self.lock:
                dx_mm = (self.pose.x * 1000.0) - x_mm
                dy_mm = (self.pose.y * 1000.0) - y_mm
            if math.hypot(dx_mm, dy_mm) <= tol_mm:
                return True
            if time.time() - t0 > timeout_s:
                return False
            time.sleep(0.02)
        return False


def main():
    rclpy.init()
    node = WaypointRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
