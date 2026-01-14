#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from vtr_navigation_msgs.msg import MissionCommand
from vtr_navigation_msgs.srv import ServerState


class CancelActiveGoalNode(Node):
    def __init__(self):
        super().__init__('cancel_active_goal_node')
        self.cli = self.create_client(ServerState, 'server_state_srv')
        self.pub = self.create_publisher(MissionCommand, "mission_command", 10)
        self.current_id = None
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for action cancel service...')
        self.send_cancel_request()
        self.timer = None

    def send_cancel_request(self):
        req = ServerState.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_cancel_response)

    def handle_cancel_response(self, future):
        response = future.result().server_state
        self.m = MissionCommand()
        self.m.type = MissionCommand.CANCEL_GOAL
        self.m.goal_handle.id = response.current_goal_id
        # Start timer to publish after 0.5 seconds
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.publish_cancel_goal)

    def publish_cancel_goal(self):
        self.pub.publish(self.m)
        self.get_logger().info('Published cancel goal command.')
        # Destroy timer after publishing once
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None


def main(args=None):
    rclpy.init(args=args)
    node = CancelActiveGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()