import os
import os.path as osp
import sys
import numpy as np
import numpy.linalg as npla
import scipy.spatial.transform as sptf

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

from pyboreas import BoreasDataset

np.set_printoptions(precision=6, suppress=True)

## GLOBAL SETTINGS
ROOT = osp.join(os.getenv('VTRDATA'), 'boreas/sequences')
SEQUENCES = [
    ["boreas-2020-12-01-13-26"],
    # ["boreas-2020-12-18-13-44"],  # cannot localize to the first run initially
    # ["boreas-2021-01-15-12-17"],  # still cannot
    # ["boreas-2021-03-02-13-38"],
]
PLAY_MODE = "result"

## OPTIONS
SEQUENCE = 0
START_FRAME = 0


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


class BoreasDatasetWrapper(BoreasDataset):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def get_lidar_iter(self, seq=0, time=False, pose=False, start=0, stop=None):
    for i, frame in enumerate(self.get_seq(seq).lidar_frames[start:stop], start):
      res = [i]
      if time:
        res = [*res, frame.timestamp]
      if pose:
        res = [*res, frame.pose]
      res = [*res, frame.load_data()]
      frame.unload_data()
      yield res


class BoreasPlayer(Node):

  def __init__(self) -> None:
    super().__init__("boreas_player")

    # Velodyne Lidar Alpha Prime specification
    channels = 128
    meas_range = 245
    fov_deg = np.array([-25., 15.])
    # when converting to polar coordinates, theta starts from z axis so this number should be array([2.00712864, 1.30899694])
    fov_rad = np.radians(fov_deg)  # array([-0.43633231,  0.26179939])

    avg_vert_ang_res = 0.005497100006281353  # vertical fov / 127 in radian
    # minimum reported in specification in radian
    # there are 128 channels, 127 gaps, this min is true for 103/127 gaps
    min_vert_ang_res = 0.0019198621771937625

    avg_hori_ang_res = 0.005  # heuristic value
    min_hori_ang_res = 0.001745329251994329  # minimum reported in specification in radian
    max_hori_ang_res = 0.006981317007977318  # maximum in radian

    dataset = BoreasDatasetWrapper(ROOT, SEQUENCES)

    self.lidar_iter = dataset.get_lidar_iter(SEQUENCE, True, True, START_FRAME)

    # Ground truth is provided w.r.t sensor, so we set sensor to vehicle
    # transform to identity
    yfwd2xfwd = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    self.T_robot_lidar = yfwd2xfwd @ dataset.sequences[SEQUENCE].calib.T_applanix_lidar
    # self.T_robot_lidar = dataset.sequences[SEQUENCE].calib.T_applanix_lidar
    self.T_lidar_robot = npla.inv(self.T_robot_lidar)

    # Set our world origin to be the first robot frame
    # T_world_robot[0] = T_world_enu @ T_enu_robot[0] = I so that
    # T_world_enu = inverse(T_enu_robot[0])
    T_enu_lidar_0 = dataset.sequences[SEQUENCE].lidar_frames[START_FRAME].pose
    self.T_world_enu = npla.inv(T_enu_lidar_0 @ self.T_lidar_robot)

    # Create a publisher that publishes sensor_msgs.PointCloud2
    self.clock_publisher = self.create_publisher(rosgraph_msgs.Clock, CLOCK_TOPIC, 1)
    # QoSPresetProfiles.get_from_short_key('SENSOR_DATA')
    self.pc_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/points', 10)
    self.gt_pc_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/ground_truth_points', 10)
    self.path_publisher = self.create_publisher(nav_msgs.Path, '/ground_truth_path', 10)
    self.tf_publisher = TransformBroadcaster(self)
    self.static_tf_publisher = StaticTransformBroadcaster(self)

    # Broadcast static T lidar robot transform
    tfs = pose2tfstamped(self.T_robot_lidar, Time(seconds=0).to_msg(), 'robot', 'velodyne')
    self.static_tf_publisher.sendTransform(tfs)

    # Publish the ground truth path
    ground_truth_poses = [
        dataset.sequences[SEQUENCE].lidar_frames[i].pose
        for i in range(START_FRAME, len(dataset.sequences[SEQUENCE].lidar_frames))
    ]
    path = self.poses2path(ground_truth_poses, Time(seconds=0).to_msg(), 'world')
    self.path_publisher.publish(path)

    input("Enter to start.")
    if PLAY_MODE == "timed":
      timer_period = 1 / 10
      self.timer = self.create_timer(timer_period, self.publish)
    elif PLAY_MODE == "result":
      self.result_sub = self.create_subscription(std_msgs.Bool, 'vtr/result', self.publish, 1)
      self.publish()
    else:
      while True:
        input("Enter to get next frame")
        self.publish()

  def publish(self, *args, **kwargs):

    frame, timestamp, T_enu_lidar, points = next(self.lidar_iter)

    T_world_robot = self.T_world_enu @ T_enu_lidar @ self.T_lidar_robot

    # Publish current time
    curr_time_secs = timestamp
    seconds = int(np.floor(curr_time_secs))
    nanoseconds = int((curr_time_secs - np.floor(curr_time_secs)) * 1e9)

    # print("T_world_robot:\n", T_world_robot)
    curr_time = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
    clock_msg = rosgraph_msgs.Clock()
    clock_msg.clock = curr_time
    self.clock_publisher.publish(clock_msg)

    # Supposed to be output from the algorithm
    # tfs = pose2tfstamped(T_world_robot, curr_time, 'world', 'robot')
    # self.tf_publisher.sendTransform(tfs)
    tfs = pose2tfstamped(T_world_robot, curr_time, 'world', 'robot_ground_truth')
    self.tf_publisher.sendTransform(tfs)

    # Publish points
    print("Publishing frame number: ", frame, ", time: ", curr_time)
    self.pc_publisher.publish(point_cloud(points[..., [0, 1, 2, 5]], 'velodyne', curr_time))
    # self.gt_pc_publisher.publish(point_cloud_rviz(points[..., :3], 'velodyne', curr_time))

  def poses2path(self, poses, stamp, to_frame):
    paths = nav_msgs.Path()
    paths.header.frame_id = to_frame
    paths.header.stamp = stamp
    for T_enu_lidar in poses:
      pose = self.T_world_enu @ T_enu_lidar @ self.T_lidar_robot

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
                     dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('t', np.float64)])
  data.x = points[:, 0]
  data.y = points[:, 1]
  data.z = points[:, 2]
  data.t = points[:, 3]
  data = data.tobytes()  # convert to bytes

  # The fields specify what the bytes represents.
  fields = [
      sensor_msgs.PointField(name=n, offset=i * points_itemsize, datatype=points_dtype, count=1)
      for i, n in enumerate('xyz')
  ]
  fields.append(sensor_msgs.PointField(name='t', offset=3 * points_itemsize, datatype=time_dtype, count=1))

  # The PointCloud2 message also has a header which specifies which coordinate frame it is represented in.
  header = std_msgs.Header(frame_id=parent_frame, stamp=stamp)

  return sensor_msgs.PointCloud2(header=header,
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

  # The fields specify what the bytes represents. The first 8 bytes represents the x-coordinate, the next 8 the y-coordinate, etc.
  fields = [
      sensor_msgs.PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')
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
  rclpy.init(args=args)
  boreas_player = BoreasPlayer()
  rclpy.spin(boreas_player)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
