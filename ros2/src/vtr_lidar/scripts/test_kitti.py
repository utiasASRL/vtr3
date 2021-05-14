#!/usr/bin/env python

import sys
import os

import numpy as np
from scipy.spatial import transform as sptf

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import pykitti
import pylgmath


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
    super().__init__('kitti_publisher_node')

    # Change this to the directory where you store KITTI data
    basedir = '/home/yuchen/ASRL/dataset/kitti/dataset'

    # Specify the dataset to load
    sequence = '00'
    frame_end = 4000
    frame_skip = 1

    # Load the data. Optionally, specify the frame range to load.
    # dataset = pykitti.odometry(basedir, sequence)
    dataset = pykitti.odometry(basedir,
                               sequence,
                               frames=range(0, frame_end, frame_skip))

    # dataset.calib:      Calibration data are accessible as a named tuple
    # dataset.timestamps: Timestamps are parsed into a list of timedelta objects
    # dataset.poses:      List of ground truth poses T_w_cam0
    # dataset.camN:       Generator to load individual images from camera N
    # dataset.gray:       Generator to load monochrome stereo pairs (cam0, cam1)
    # dataset.rgb:        Generator to load RGB stereo pairs (cam2, cam3)
    # dataset.velo:       Generator to load velodyne scans as [x,y,z,reflectance]

    # Display some of the data
    np.set_printoptions(precision=4, suppress=True)
    print('\nSequence: ' + str(dataset.sequence))
    print('\nFrame range: ' + str(dataset.frames))

    poses = np.array(dataset.poses)
    vel_poses = np.array(dataset.poses) @ dataset.calib.T_cam0_velo

    # Velodyne Lidar HDL-64E
    channels = 64
    meas_range = 120
    fov_deg = np.array([-24.9, 2.0])
    fov_rad = np.radians(fov_deg)
    ang_res_deg = 0.08
    N = int(channels * (360 / ang_res_deg))

    self.T_cam0_velo = dataset.calib.T_cam0_velo
    self.veloit = iter(dataset.velo)
    self.poseit = iter(dataset.poses)

    # spoof the T_sensor vehicle
    T_cam0_velo = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0],
                            [0, 0, 0, 1]])
    self.T_cam0_velo = np.linalg.inv(T_cam0_velo) @ self.T_cam0_velo

    # I create a publisher that publishes sensor_msgs.PointCloud2 to the
    # topic 'pcd'. The value '10' refers to the history_depth, which I
    # believe is related to the ROS1 concept of queue size.
    # Read more here:
    # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2,
                                               'raw_points', 100)
    self.tf_publisher = TransformBroadcaster(self)
    self.static_tf_publisher = StaticTransformBroadcaster(self)

    # broadcast static T senro vehicle transform
    stamp = self.get_clock().now().to_msg()
    tfs = pose2tfstamped(self.T_cam0_velo, stamp, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_cam0_velo, stamp, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    input("Enter to start.")

    # Option 1 timed
    # timer_period = 1 / 1
    # self.timer = self.create_timer(timer_period, self.publish)

    # Option 2 user input
    # while True:
    #   input("Enter to get next frame")
    #   self.publish()

    # Option 3 result trigger
    self.result_sub = self.create_subscription(std_msgs.Bool, 'vtr/result',
                                               self.publish, 1)
    self.publish()

  def publish(self, *args, **kwargs):

    pose = next(self.poseit)
    stamp = self.get_clock().now().to_msg()

    tfs = pose2tfstamped(pose, stamp, 'world', 'robot_ground_truth')
    self.tf_publisher.sendTransform(tfs)

    # broadcast static T senro vehicle transform (should be sent infrequently)
    stamp = self.get_clock().now().to_msg()
    tfs = pose2tfstamped(self.T_cam0_velo, stamp, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_cam0_velo, stamp, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    points = np.expand_dims(next(self.veloit)[..., :3], -1)
    points = pylgmath.point_conv.cart2homo(points)
    points = pylgmath.point_conv.homo2cart(points)

    # subsample points
    # points = points[::20, ...]  # this is dangerous
    # cam0 is considered robot frame in kitti
    pcd = point_cloud(points, 'velodyne', stamp)
    # Then I publish the PointCloud2 object
    self.pcd_publisher.publish(pcd)


def point_cloud(points, parent_frame, stamp):
  """ Creates a point cloud message.
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
