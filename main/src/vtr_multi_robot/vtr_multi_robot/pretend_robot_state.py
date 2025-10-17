#!/usr/bin/env python3
# File: pretend_robot_state.py
# Minimal ROS2 node that creates two services of type vtr_navigation_msgs/srv/RobotState
# and returns a default response when called.

import rclpy
from rclpy.node import Node
from vtr_navigation_msgs.srv import RobotState


class PretendRobotState(Node):
    def __init__(self):
        super().__init__('pretend_robot_state')

        # Create two services (names can be changed as needed)
        self.srv1 = self.create_service(
            RobotState,
            '/robot1/vtr/robot_state_srv',
            lambda req, res: self._handle_state(req, res, 14)
        )
        self.srv2 = self.create_service(
            RobotState,
            '/robot2/vtr/robot_state_srv',
            lambda req, res: self._handle_state(req, res, 3)
        )

    def _handle_state(self, request, response, service_idx: int):
        # Log that the service was called
        self.get_logger().info(f"Service '{service_idx}' was called")

        # Return a default-constructed response. If you want to populate specific fields,
        # inspect the RobotState.Response type and set fields here, e.g.:
        response.robot_state.index = service_idx
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PretendRobotState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down PretendRobotState node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()