#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from vtr_messages.msg import DesiredActionIn

class PlaybackSafetyMonitor(Node):

  pub_status = None
  status_timer = None

  def __init__(self):
    super().__init__('safety_monitor_node')
    self.pub_status = self.create_publisher(DesiredActionIn, '/safety_monitor_node/out/desired_action', qos_profile=1)
    self.status_timer = self.create_timer(0.2, callback=self.send_status)

  def send_status(self):
    msg = DesiredActionIn()

    msg.desired_action = "CONTINUE"
    msg.speed_limit = 4.5

    self.pub_status.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  controller_node = PlaybackSafetyMonitor()

  rclpy.spin(controller_node)


if __name__ == "__main__":
    main()
