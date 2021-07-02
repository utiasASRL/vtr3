#!/usr/bin/env python

import os
import os.path as osp

import numpy as np
import numpy.linalg as npla
from scipy.spatial import transform as sptf

import rclpy
from rclpy.qos import QoSPresetProfiles
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time_source import CLOCK_TOPIC
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import rosgraph_msgs.msg as rosgraph_msgs

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import pyboreas

from pylgmath import cmnop


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
    sequence = 'boreas-2020-11-26-13-58'  # 0:9700 almost one run.
    # sequence = 'boreas-2020-12-01-13-26'

    # Load the data. Optionally, specify the frame range to load.
    dataset = pyboreas.ProcessedData(basedir, sequence)

    # Display some of the data
    np.set_printoptions(precision=4, suppress=True)

    # Get lidar data iterator
    start_frame = 1390
    self.lidar_iter = dataset.get_lidar_iter(True, True, start_frame)

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
    self.clock_publisher = self.create_publisher(rosgraph_msgs.Clock,
                                                 CLOCK_TOPIC, 1)
    self.pcd_publisher = self.create_publisher(
        sensor_msgs.PointCloud2, '/raw_points',
        10)  # QoSPresetProfiles.get_from_short_key('SENSOR_DATA')
    self.gt_pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2,
                                                  'ground_truth_points', 10)
    self.tf_publisher = TransformBroadcaster(self)
    self.static_tf_publisher = StaticTransformBroadcaster(self)

    # shift for replay
    shift_sec = 0.

    _, start_ros_time, _ = dataset.get_lidar_frame(start_frame, True)
    start_hour_sec = (start_ros_time - int(start_ros_time % (3600 * 1e9))) / 1e9
    self.start_sec = start_hour_sec + shift_sec

    self.last_timestamp = 0

    # publish current time
    curr_time = Time(seconds=self.start_sec).to_msg()
    clock_msg = rosgraph_msgs.Clock()
    clock_msg.clock = curr_time
    self.clock_publisher.publish(clock_msg)
    # broadcast static T sensor vehicle transform
    tfs = pose2tfstamped(self.T_robot_lidar, curr_time, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_robot_lidar, curr_time, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    input("Enter to start.")

    # Option 1 timed
    # timer_period = 1 / 10
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

    frame, _, T_enu_lidar, points = next(self.lidar_iter)
    # pyboreas.visualization.visualize_point_cloud(points)
    points = points.astype(np.float64)
    if (points[-1, 5] + self.start_sec < self.last_timestamp):
      self.start_sec += 3600.  # increase by one hour
    points[..., 5] += self.start_sec
    self.last_timestamp = points[-1, 5]

    T_world_robot = self.T_world_enu @ T_enu_lidar @ self.T_lidar_robot

    # publish current time
    curr_time_secs = points[-1, 5]
    seconds = int(np.floor(curr_time_secs))
    nanoseconds = int((curr_time_secs - np.floor(curr_time_secs)) * 1e9)

    # print("T_world_robot:\n", T_world_robot)
    curr_time = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
    clock_msg = rosgraph_msgs.Clock()
    clock_msg.clock = curr_time
    self.clock_publisher.publish(clock_msg)

    # broadcast static T senro vehicle transform (should be sent infrequently)
    tfs = pose2tfstamped(self.T_robot_lidar, curr_time, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_robot_lidar, curr_time, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    # Supposed to be output from the algorithm
    # tfs = pose2tfstamped(T_world_robot, curr_time, 'world', 'robot')
    # self.tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(T_world_robot, curr_time, 'world',
                         'robot_ground_truth')
    self.tf_publisher.sendTransform(tfs)

    # publish points
    points = points[..., [0, 1, 2, 5]]
    # here we replace the last element to current time
    print("Publishing frame number: ", frame, ", time: ", curr_time)
    self.pcd_publisher.publish(point_cloud(points, 'velodyne', curr_time))

    self.gt_pcd_publisher.publish(
        point_cloud_rviz(points[..., :3, 0], 'velodyne', curr_time))

    # points[..., 3, 0] -= np.floor(points[..., 3, 0])
    # np.set_printoptions(9)
    # polpts = cmnop.cart2pol(points[..., :3])
    # for i in range(1, polpts.shape[0]):
    #   if polpts[i, 2, 0] - polpts[i - 1, 2, 0] > np.pi:
    #     polpts[i, 2, 0] -= 2 * np.pi
    #   elif polpts[i, 2, 0] - polpts[i - 1, 2, 0] < -np.pi:
    #     polpts[i, 2, 0] += 2 * np.pi
    # print(max(polpts[:, 2]))
    # print(min(polpts[:, 2]))
    # print("time", points[0:10, 3, 0])
    # print(polpts[0:-1:100, 2])
    # print("time", points[-10:-1, 3, 0])
    # print(polpts[-10:-1, 2].T)
    # for i in range(0, 1000000, 100):
    #   print(polpts[i, 2, 0])


def point_cloud(points, parent_frame, stamp):
  """Creates a point cloud message.
  Args:
      points: Nx4 array of xyz positions plus time stamp
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

  # for point cloud xyz
  points_dtype = sensor_msgs.PointField.FLOAT32
  points_itemsize = np.dtype(np.float32).itemsize
  time_dtype = sensor_msgs.PointField.FLOAT64
  time_itemsize = np.dtype(np.float64).itemsize

  data = np.recarray((points.shape[0],),
                     dtype=[('x', np.float32), ('y', np.float32),
                            ('z', np.float32), ('t', np.float64)])
  data.x = points[:, 0]
  data.y = points[:, 1]
  data.z = points[:, 2]
  data.t = points[:, 3]
  data = data.tobytes()  # convert to bytes

  # The fields specify what the bytes represents.
  fields = [
      sensor_msgs.PointField(name=n,
                             offset=i * points_itemsize,
                             datatype=points_dtype,
                             count=1) for i, n in enumerate('xyz')
  ]
  fields.append(
      sensor_msgs.PointField(name='t',
                             offset=3 * points_itemsize,
                             datatype=time_dtype,
                             count=1))

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
      point_step=(3 * points_itemsize + time_itemsize),
      row_step=((3 * points_itemsize + time_itemsize) * points.shape[0]),
      data=data)


def point_cloud_rviz(points, parent_frame, stamp):
  """Creates a point cloud message.
  Args:
      points: Nx4 array of xyz positions plus time stamp
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

  # for point cloud xyz
  ros_dtype = sensor_msgs.PointField.FLOAT32
  dtype = np.float32
  itemsize = np.dtype(dtype).itemsize  # A 64-bit float takes 8 bytes.

  data = points.astype(dtype).tobytes()

  # The fields specify what the bytes represents. The first 8 bytes
  # represents the x-coordinate, the next 8 the y-coordinate, etc.
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
      point_step=(itemsize * 3),  # Every point consists of four float32s.
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
