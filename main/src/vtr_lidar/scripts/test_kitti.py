#!/usr/bin/env python3

import os
import os.path as osp

import numpy as np
from scipy.spatial import transform as sptf

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time_source import CLOCK_TOPIC
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import rosgraph_msgs.msg as rosgraph_msgs

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import pykitti


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
    basedir = osp.join(os.getenv('VTRDATA'), 'kitti/dataset')

    # Specify the dataset to load
    sequence = '00'
    frame_end = 9999
    frame_skip = 1

    # Load the data. Optionally, specify the frame range to load.
    dataset = pykitti.odometry(basedir, sequence)
    # dataset = pykitti.odometry(basedir,
    #                            sequence,
    #                            frames=range(0, frame_end, frame_skip))

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
    self.timeit = iter(dataset.timestamps)

    # spoof the T_sensor vehicle
    self.T_cam0_robot = np.array([
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1],
    ])
    # Ground truth is provided w.r.t.
    self.T_robot_cam0 = np.linalg.inv(self.T_cam0_robot)

    self.T_robot_velo = self.T_robot_cam0 @ self.T_cam0_velo

    # publishers
    self.clock_publisher = self.create_publisher(rosgraph_msgs.Clock,
                                                 CLOCK_TOPIC, 1)
    self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2,
                                               '/points', 10)
    self.path_publisher = self.create_publisher(nav_msgs.Path, '/gt_path', 10)
    self.tf_publisher = TransformBroadcaster(self)
    self.static_tf_publisher = StaticTransformBroadcaster(self)

    self.shift_secs = 0.
    # self.shift_secs = 80000.

    # publish current time
    curr_time = Time(seconds=self.shift_secs).to_msg()
    clock_msg = rosgraph_msgs.Clock()
    clock_msg.clock = curr_time
    self.clock_publisher.publish(clock_msg)
    # broadcast static T senro vehicle transform
    tfs = pose2tfstamped(self.T_robot_velo, curr_time, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_robot_velo, curr_time, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    # publish ground truth path
    path = self.poses2path(dataset.poses, curr_time, 'world')
    self.path_publisher.publish(path)

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

    # publish current time
    curr_time_secs = next(self.timeit).total_seconds()
    seconds = int(np.floor(curr_time_secs))
    nanoseconds = int((curr_time_secs - np.floor(curr_time_secs)) * 1e9)

    curr_time = Time(seconds=self.shift_secs + seconds,
                     nanoseconds=nanoseconds).to_msg()
    clock_msg = rosgraph_msgs.Clock()
    clock_msg.clock = curr_time
    self.clock_publisher.publish(clock_msg)

    # broadcast static T senro vehicle transform (should be sent infrequently)
    tfs = pose2tfstamped(self.T_robot_velo, curr_time, 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(self.T_robot_velo, curr_time, 'base_link', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    # broadcast ground truth robot transform
    pose = next(self.poseit)
    pose = self.T_robot_cam0 @ pose @ self.T_cam0_robot
    tfs = pose2tfstamped(pose, curr_time, 'world', 'robot_ground_truth')
    self.tf_publisher.sendTransform(tfs)

    # publish point cloud
    points = next(self.veloit)[..., :4]
    points = points.astype(np.float64)
    # here we replace the last element to current time
    points[..., 3] = curr_time.sec + curr_time.nanosec / 1e9
    # cam0 is considered robot frame in kitti
    self.pcd_publisher.publish(point_cloud(points, 'velodyne', curr_time))

  def poses2path(self, poses, stamp, to_frame):
    paths = nav_msgs.Path()
    paths.header.frame_id = to_frame
    paths.header.stamp = stamp
    for pose in poses:

      pose = self.T_robot_cam0 @ pose @ self.T_cam0_robot

      pose_msg = geometry_msgs.PoseStamped()
      tran = pose[:3, 3]
      rot = sptf.Rotation.from_matrix(pose[:3, :3]).as_quat()

      # The default (fixed) frame in RViz is called 'world'
      pose_msg.pose.position.x = tran[0]
      pose_msg.pose.position.y = tran[1]
      pose_msg.pose.position.z = tran[2]
      pose_msg.pose.orientation.x = rot[0]
      pose_msg.pose.orientation.y = rot[1]
      pose_msg.pose.orientation.z = rot[2]
      pose_msg.pose.orientation.w = rot[3]
      paths.poses.append(pose_msg)
    return paths


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
