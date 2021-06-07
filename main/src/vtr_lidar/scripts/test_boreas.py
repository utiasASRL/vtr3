#!/usr/bin/env python

import os
import os.path as osp

import numpy as np
import numpy.linalg as npla
from scipy.spatial import transform as sptf

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import pyboreas


def pose2tfstamped(pose, stamp, to_frame, from_frame):
  tran = pose[:3, 3]
  rot = sptf.Rotation.from_matrix(pose[:3, :3]).as_quat()

  tfs = geometry_msgs.TransformStamped()
  # The default (fixed) frame in RViz is called 'world'
  tfs.header.frame_id = to_frame
  tfs.header.stamp = stamp
  tfs.child_frame_id = from_frame
  tfs.transform.translation.x = tran[0]
  tfs.transform.translation.y = tran[1]
  tfs.transform.translation.z = tran[2]
  tfs.transform.rotation.x = rot[0]
  tfs.transform.rotation.y = rot[1]
  tfs.transform.rotation.z = rot[2]
  tfs.transform.rotation.w = rot[3]
  return tfs


class PCDPublisher(Node):

  def __init__(self):
    super().__init__('boreas_publisher_node')

    # Velodyne Lidar HDL-64E specification
    channels = 128
    meas_range = 300
    fov_deg = np.array([-25., 15.])
    fov_rad = np.radians(fov_deg)
    avg_vert_ang_res = 0.005497100006281353  # vertical fov / 127 in radian
    min_vert_ang_res = 0.0019198621771937625  # minimum reported in specification in radian
    avg_hori_ang_res = 0.005  # heuristic value
    min_hori_ang_res = 0.001745329251994329  # minimum reported in specification in radian
    max_hori_ang_res = 0.006981317007977318  # maximum in radian

    # Change this to the directory where you store KITTI data
    basedir = osp.join(os.getenv('VTRDATA'), 'boreas')

    # Specify the dataset to load
    sequence = 'boreas-2020-11-26-13-58'
    # sequence = 'boreas-2020-12-01-13-26'

    # Load the data. Optionally, specify the frame range to load.
    dataset = pyboreas.ProcessedData(basedir, sequence)

    # Display some of the data
    np.set_printoptions(precision=4, suppress=True)

    # Get lidar data iterator
    self.lidar_iter = dataset.get_lidar_iter(True, True)

    # Ground truth is provided w.r.t sensor, so we set sensor to vehicle
    # transform to identity
    yfwd2xfwd = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    self.T_robot_lidar = yfwd2xfwd @ dataset.T_applanix_lidar
    self.T_lidar_robot = npla.inv(self.T_robot_lidar)

    # Set our world origin to be the first robot frame
    # T_world_robot[0] = T_world_enu @ T_enu_robot[0] = I so that
    # T_world_enu = inverse(T_enu_robot[0])
    T_enu_lidar_0 = dataset.poses.lidar[0]
    self.T_world_enu = npla.inv(T_enu_lidar_0 @ self.T_lidar_robot)

    # Create a publisher that publishes sensor_msgs.PointCloud2
    self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2,
                                               'raw_points', 10)
    self.tf_publisher = TransformBroadcaster(self)
    self.static_tf_publisher = StaticTransformBroadcaster(self)

    # broadcast static T sensor vehicle transform
    stamp = self.get_clock().now().to_msg()
    tfs = pose2tfstamped(self.T_robot_lidar, stamp, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_robot_lidar, stamp, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    input("Enter to start.")

    # # Option 2 user input
    # while True:
    #   input("Enter to get next frame")
    #   self.publish()

    # Option 3 result trigger
    self.result_sub = self.create_subscription(std_msgs.Bool, 'vtr/result',
                                               self.publish, 1)
    self.publish()

  def publish(self, *args, **kwargs):

    _, T_enu_lidar, pcd = next(self.lidar_iter)
    T_world_robot = self.T_world_enu @ T_enu_lidar @ self.T_lidar_robot
    stamp = self.get_clock().now().to_msg()

    print("T_world_robot:\n", T_world_robot)

    tfs = pose2tfstamped(T_world_robot, stamp, 'world', 'robot_ground_truth')
    self.tf_publisher.sendTransform(tfs)

    # Supposed to be output from the algorithm
    # tfs = pose2tfstamped(T_world_robot, stamp, 'world', 'robot')
    # self.tf_publisher.sendTransform(tfs)

    # broadcast static T senro vehicle transform (should be sent infrequently)
    stamp = self.get_clock().now().to_msg()
    tfs = pose2tfstamped(self.T_robot_lidar, stamp, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_robot_lidar, stamp, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    # publish points
    points = np.expand_dims(pcd[..., :3], -1)
    self.pcd_publisher.publish(point_cloud(points, 'velodyne', stamp))


def point_cloud(points, parent_frame, stamp):
  """Creates a point cloud message.
  Args:
      points: Nx3 array of xyz positions.
      parent_frame: frame in which the point cloud is defined
  Returns:
      sensor_msgs/PointCloud2 message

  Code source:
      https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

  References:
      http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
      http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
      http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

  """
  # In a PointCloud2 message, the point cloud is stored as an byte
  # array. In order to unpack it, we also include some parameters
  # which desribes the size of each individual point.

  ros_dtype = sensor_msgs.PointField.FLOAT32
  dtype = np.float32
  itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

  data = points.astype(dtype).tobytes()

  # The fields specify what the bytes represents. The first 4 bytes
  # represents the x-coordinate, the next 4 the y-coordinate, etc.
  fields = [
      sensor_msgs.PointField(name=n,
                             offset=i * itemsize,
                             datatype=ros_dtype,
                             count=1) for i, n in enumerate('xyz')
  ]

  # The PointCloud2 message also has a header which specifies which
  # coordinate frame it is represented in.
  header = std_msgs.Header(frame_id=parent_frame, stamp=stamp)

  return sensor_msgs.PointCloud2(
      header=header,
      height=1,
      width=points.shape[0],
      is_dense=False,
      is_bigendian=False,
      fields=fields,
      point_step=(itemsize * 3),  # Every point consists of three float32s.
      row_step=(itemsize * 3 * points.shape[0]),
      data=data)


def main(args=None):
  # Boilerplate code.
  rclpy.init(args=args)
  pcd_publisher = PCDPublisher()
  rclpy.spin(pcd_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  pcd_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
