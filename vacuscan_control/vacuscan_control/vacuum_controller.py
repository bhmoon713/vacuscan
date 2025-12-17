#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class VacuumController(Node):
    """
    Simple vacuum controller.

    - /vacuum/on  (std_srvs/Trigger)
    - /vacuum/off (std_srvs/Trigger)

    Right now it only tracks state + logs.
    Later you can add GPIO / serial / Modbus, etc.
    """

    def __init__(self):
        super().__init__('vacuum_controller')

        self.is_on = False

        # Services
        self.srv_on = self.create_service(
            Trigger, '/vacuum/on', self.on_vacuum_on
        )
        self.srv_off = self.create_service(
            Trigger, '/vacuum/off', self.on_vacuum_off
        )

        self.get_logger().info('VacuumController ready: /vacuum/on, /vacuum/off')

    # --- service callbacks ------------------------------------------------
    def on_vacuum_on(self, req, res):
        if not self.is_on:
            self.is_on = True
            # TODO: add your real hardware command here
            self.get_logger().info('Vacuum ON')
            res.success = True
            res.message = 'Vacuum turned ON'
        else:
            res.success = True
            res.message = 'Vacuum already ON'
        return res

    def on_vacuum_off(self, req, res):
        if self.is_on:
            self.is_on = False
            # TODO: add your real hardware command here
            self.get_logger().info('Vacuum OFF')
            res.success = True
            res.message = 'Vacuum turned OFF'
        else:
            res.success = True
            res.message = 'Vacuum already OFF'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = VacuumController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
