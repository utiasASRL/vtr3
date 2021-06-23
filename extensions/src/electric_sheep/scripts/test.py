#!/usr/bin/env python

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import Bool


class MinimalSubscriber(Node):

  def __init__(self):
    super().__init__('minimal_subscriber')
    self.subscription = self.create_subscription(PointCloud2, 'raw_points',
                                                 self.listener_callback, 10)
    self.subscription = self.create_subscription(Bool, 'term', self.save, 10)
    self.subscription  # prevent unused variable warning

    self.nsec = []

  def listener_callback(self, msg):
    print(msg.header.stamp)
    self.nsec.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
    print(self.nsec[-1])

  def save(self, *args):
    to_save = np.array(self.nsec)
    print(to_save.shape)
    np.save("/ext0/datasets/electric_sheep/data0525/lidarstamp.npy", to_save)


# class MinimalSubscriber(Node):

#   def __init__(self):
#     super().__init__('minimal_subscriber')
#     self.subscription = self.create_subscription(NavSatFix, 'gps',
#                                                  self.listener_callback, 10)
#     self.subscription = self.create_subscription(Bool, 'term', self.save, 10)
#     self.subscription  # prevent unused variable warning

#     self.nsec = []
#     self.latitude = []
#     self.longitude = []

#     position_covariance = []
#     position_covariance_type = 0

#   def listener_callback(self, msg):
#     print(msg.header.stamp)
#     self.nsec.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
#     self.latitude.append(msg.latitude)
#     self.longitude.append(msg.longitude)
#     print(self.nsec[-1], ": ", self.latitude[-1], ",", self.longitude[-1])

#   def save(self, *args):
#     to_save = np.array([self.nsec, self.longitude, self.latitude])
#     print(to_save.shape)
#     np.save("/ext0/datasets/electric_sheep/data0525/gps.npy", to_save)


def main(args=None):
  rclpy.init(args=args)

  minimal_subscriber = MinimalSubscriber()

  rclpy.spin(minimal_subscriber)

  minimal_subscriber.save()

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  minimal_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
