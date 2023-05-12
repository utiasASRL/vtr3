#!/usr/bin/env python3

# Copyright 2021, Autonomous Space Robotics Lab (ASRL)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from enum import Enum

from vtr_navigation_msgs.srv import GraphState as GraphStateSrv
from vtr_navigation_msgs.srv import RobotState as RobotStateSrv
from vtr_navigation_msgs.srv import ServerState as ServerStateSrv
from vtr_navigation_msgs.srv import FollowingRoute as FollowingRouteSrv
from vtr_navigation_msgs.srv import TaskQueueState as TaskQueueStateSrv
from vtr_navigation_msgs.msg import GraphState, GraphUpdate, RobotState, GraphRoute
from vtr_navigation_msgs.msg import MoveGraph, AnnotateRoute
from vtr_navigation_msgs.msg import MissionCommand, ServerState
from vtr_navigation_msgs.msg import TaskQueueUpdate
from vtr_pose_graph_msgs.srv import MapInfo as MapInfoSrv
from vtr_tactic_msgs.msg import EnvInfo

from vtr_navigation.ros_manager import ROSManager

vtr_ui_logger = logging.getLogger('vtr_ui')
vtr_ui_logger.setLevel(logging.INFO)
hd = logging.StreamHandler()
fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
hd.setFormatter(fm)
vtr_ui_logger.addHandler(hd)


class VTRUI(ROSManager):
  """Client used to interface with the C++ MissionServer node."""

  class Notification(Enum):
    """Enumerates possible notifications that might come back from ROS; overloads parent definition"""
    Feedback = 0

  @ROSManager.on_ros
  def setup_ros(self, *args, **kwargs):
    """Sets up necessary ROS communications"""
    # graph state
    self._graph_state_cli = self.create_client(GraphStateSrv, "graph_state_srv")
    while not self._graph_state_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for graph_state_srv service...")
    self._graph_state_sub = self.create_subscription(GraphState, 'graph_state', self.graph_state_callback, 10)
    self._graph_update_sub = self.create_subscription(GraphUpdate, 'graph_update', self.graph_update_callback, 10)

    # robot state
    self._robot_state_cli = self.create_client(RobotStateSrv, "robot_state_srv")
    while not self._robot_state_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for robot_state_srv service...")
    self._robot_state_sub = self.create_subscription(RobotState, 'robot_state', self.robot_state_callback, 10)

    # route being followed
    self._following_route_cli = self.create_client(FollowingRouteSrv, "following_route_srv")
    while not self._following_route_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for following_route_srv service...")
    self._following_route_sub = self.create_subscription(GraphRoute, 'following_route', self.following_route_callback,
                                                         10)

    # graph manipulation
    self._move_graph_pub = self.create_publisher(MoveGraph, 'move_graph', 1)
    self._annotate_route_pub = self.create_publisher(AnnotateRoute, 'annotate_route', 1)

    # mission command
    self._mission_command_pub = self.create_publisher(MissionCommand, 'mission_command', 1)
    self._server_state_sub = self.create_subscription(ServerState, 'server_state', self.server_state_callback, 10)
    self._server_state_cli = self.create_client(ServerStateSrv, "server_state_srv")
    while not self._server_state_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for server_state_srv service...")

    # task queue
    self._task_queue_update_sub = self.create_subscription(TaskQueueUpdate, 'task_queue_update',
                                                           self.task_queue_update_callback, 10)
    self._task_queue_state_cli = self.create_client(TaskQueueStateSrv, "task_queue_state_srv")
    while not self._task_queue_state_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for task_queue_state_srv service...")

    # env info
    self._change_env_info_pub = self.create_publisher(EnvInfo, 'env_info', 1)

    # map center
    self._map_info_cli = self.create_client(MapInfoSrv, "map_info_srv")
    while not self._map_info_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for map_info_srv service...")

  @ROSManager.on_ros
  def get_graph_state(self):
    return self._graph_state_cli.call(GraphStateSrv.Request()).graph_state

  @ROSManager.on_ros
  def graph_state_callback(self, graph_state):
    self.notify("graph_state", graph_state=graph_state)

  @ROSManager.on_ros
  def graph_update_callback(self, graph_update):
    self.notify("graph_update", graph_update=graph_update)

  @ROSManager.on_ros
  def get_robot_state(self):
    return self._robot_state_cli.call(RobotStateSrv.Request()).robot_state

  @ROSManager.on_ros
  def robot_state_callback(self, robot_state):
    self.notify("robot_state", robot_state=robot_state)

  @ROSManager.on_ros
  def get_map_info(self):
    return self._map_info_cli.call(MapInfoSrv.Request()).map_info

  @ROSManager.on_ros
  def get_following_route(self):
    return self._following_route_cli.call(FollowingRouteSrv.Request()).following_route

  @ROSManager.on_ros
  def following_route_callback(self, following_route):
    self.notify("following_route", following_route=following_route)

  @ROSManager.on_ros
  def get_server_state(self):
    return self._server_state_cli.call(ServerStateSrv.Request()).server_state

  @ROSManager.on_ros
  def server_state_callback(self, server_state):
    self.notify("server_state", server_state=server_state)

  @ROSManager.on_ros
  def get_task_queue_state(self):
    return self._task_queue_state_cli.call(TaskQueueStateSrv.Request()).task_queue_state

  @ROSManager.on_ros
  def task_queue_update_callback(self, task_queue_update):
    self.notify("task_queue_update", task_queue_update=task_queue_update)

  @ROSManager.on_ros
  def set_pause(self, msg):
    self._mission_command_pub.publish(msg)

  @ROSManager.on_ros
  def add_goal(self, msg):
    self._mission_command_pub.publish(msg)

  @ROSManager.on_ros
  def cancel_goal(self, msg):
    self._mission_command_pub.publish(msg)

  @ROSManager.on_ros
  def move_robot(self, msg):
    self._mission_command_pub.publish(msg)

  @ROSManager.on_ros
  def merge(self, msg):
    self._mission_command_pub.publish(msg)

  @ROSManager.on_ros
  def confirm_merge(self, msg):
    self._mission_command_pub.publish(msg)

  @ROSManager.on_ros
  def continue_teach(self, msg):
    self._mission_command_pub.publish(msg)

  @ROSManager.on_ros
  def annotate_route(self, msg):
    self._annotate_route_pub.publish(msg)

  @ROSManager.on_ros
  def move_graph(self, msg):
    self._move_graph_pub.publish(msg)

  @ROSManager.on_ros
  def change_env_info(self, msg):
    self._change_env_info_pub.publish(msg)


if __name__ == "__main__":
  import time

  logger = logging.getLogger('default')
  logger.setLevel(logging.DEBUG)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("INITIALIZE ROS MANAGER")
  vtr_ui = VTRUI()  # ROS process start is non-blocking
  time.sleep(1)
  logger.info("INITIALIZE ROS MANAGER - DONE")

  time.sleep(2)

  logger.info("SHUTDOWN ROS MANAGER")
  vtr_ui.shutdown()
  logger.info("SHUTDOWN ROS MANAGER - DONE")
