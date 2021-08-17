#!/usr/bin/env python3

import sys
import os

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
import open3d as o3d


class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher_node')

        # This executable expectes the first argument to be the path to a
        # point cloud file. I.e. when you run it with ros:
        # ros2 run pcd_publisher pcd_publisher_node /path/to/ply
        assert len(sys.argv) > 1, "No ply file given."
        assert os.path.exists(sys.argv[1]), "File doesn't exist."
        pcd_path = sys.argv[1]

        # I use Open3D to read point clouds and meshes. It's a great library!
        pcd = o3d.io.read_point_cloud(pcd_path)
        # I then convert it into a numpy array.
        self.points = np.asarray(pcd.points)
        print(self.points.shape)

        # I create a publisher that publishes sensor_msgs.PointCloud2 to the
        # topic 'pcd'. The value '10' refers to the history_depth, which I
        # believe is related to the ROS1 concept of queue size.
        # Read more here:
        # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2,
                                                   'pcd', 10)
        timer_period = 1 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # This rotation matrix is used for visualization purposes. It rotates
        # the point cloud on each timer callback.
        self.R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi / 48])

    def timer_callback(self):
        # For visualization purposes, I rotate the point cloud with self.R
        # to make it spin.
        self.points = self.points @ self.R
        # Here I use the point_cloud() function to convert the numpy array
        # into a sensor_msgs.PointCloud2 object. The second argument is the
        # name of the frame the point cloud will be represented in. The default
        # (fixed) frame in RViz is called 'map'
        self.pcd = point_cloud(self.points, 'map')
        # Then I publish the PointCloud2 object
        self.pcd_publisher.publish(self.pcd)


def point_cloud(points, parent_frame):
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
    header = std_msgs.Header(frame_id=parent_frame)

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