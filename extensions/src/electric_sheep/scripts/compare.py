#!/usr/bin/env python

import numpy as np

import rclpy
from rclpy.node import Node

from vtr_messages.msg import RobotStatus
from std_msgs.msg import Bool

from collections import OrderedDict
import numpy as np

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("TkAgg")
plt.ion()


def find_nearest(array, value):
  array = np.asarray(array)
  idx = (np.abs(array - value)).argmin()
  if idx + 1 < len(array) and array[idx] <= value and array[idx + 1] >= value:
    a = array[idx]
    b = array[idx + 1]
  elif idx > 0 and array[idx] >= value and array[idx - 1] <= value:
    a = array[idx - 1]
    b = array[idx]
  else:
    return None, None, None, None
  c = value
  return a, b, (b - c) / (b - a), (c - a) / (b - a)


class LidarSubscriber(Node):

  def __init__(self, gps_file="gps.npy"):
    super().__init__('minimal_subscriber')
    self.subscription1 = self.create_subscription(RobotStatus, '/vtr/robot',
                                                  self.listener_callback, 10)
    self.subscription2 = self.create_subscription(Bool, '/term', self.save, 10)

    # get gps data
    gps = np.load(gps_file)
    self.gps_dict = OrderedDict()
    for i in range(gps.shape[1]):
      self.gps_dict[gps[0, i]] = np.array([gps[1, i], gps[2, i]])
    self.gps_time = np.array(list(self.gps_dict.keys()))

    self.gtlng = []
    self.gtlat = []

    self.lng = []
    self.lat = []

    fig = plt.figure(1)
    self.ax = fig.add_subplot()
    self.ax.clear()

  def listener_callback(self, msg):
    time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    a, b, aw, bw = find_nearest(self.gps_time, time)
    if not a is None:
      interp = aw * self.gps_dict[a] + bw * self.gps_dict[b]
      print(interp)
      self.ax.clear()

      ## GPS
      self.gtlng.append(interp[0])
      self.gtlat.append(interp[1])
      self.ax.plot(self.gtlng, self.gtlat, c='g', label='GPS')

      ## LIDAR
      self.lng.append(msg.lng_lat_theta[0])
      self.lat.append(msg.lng_lat_theta[1])

      #   # OPTION 1 without alignment plot directly
      #   self.ax.plot(self.lng, self.lat, c='r', label='LiDAR')

      # OPTION 2 with alignment
      # point cloud alignment
      lidar = np.array([self.lng, self.lat])
      gps = np.array([self.gtlng, self.gtlat])
      lidar_com = np.sum(lidar, axis=1, keepdims=True) / lidar.shape[1]
      gps_com = np.sum(gps, axis=1, keepdims=True) / lidar.shape[1]
      W = (lidar - lidar_com) @ (gps - gps_com).T / lidar.shape[1]
      u, s, vh = np.linalg.svd(W)
      I = np.eye(2)
      I[1, 1] = np.linalg.det(u) * np.linalg.det(vh)
      C = u @ I @ vh
      # transform
      lidar = C.T @ lidar + gps_com - C.T @ lidar_com
      self.ax.plot(lidar[0], lidar[1], c='r', label='LiDAR')

      self.ax.set_xlim([-75.5436, -75.5431])
      self.ax.set_ylim([40.53701, 40.5376])
      self.ax.set_xlabel('Longitude')
      self.ax.set_ylabel('Latitude')
      self.ax.legend()

      plt.pause(0.0001)

  def save(self, *args):
    """ros2 topic pub --once /term std_msgs/msg/Bool "{data: true}"
    """
    lidar = np.array([self.lng, self.lat])
    gps = np.array([self.gtlng, self.gtlat])
    np.save("/ext0/datasets/electric_sheep/data0525/lidar.npy", lidar)
    np.save("/ext0/datasets/electric_sheep/data0525/gps_interpolated.npy", gps)
    print("Data saved!")


def main(args=None):
  rclpy.init(args=args)

  node = LidarSubscriber("/ext0/datasets/electric_sheep/data0525/gps.npy")
  rclpy.spin(node)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
