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
from vtr_navigation_msgs.msg import MoveGraph, AnnotateRoute, UpdateWaypoint
from vtr_navigation_msgs.msg import MissionCommand, ServerState
from vtr_navigation_msgs.msg import TaskQueueUpdate
from vtr_pose_graph_msgs.srv import MapInfo as MapInfoSrv
from vtr_tactic_msgs.msg import EnvInfo
import rclpy

from vtr_navigation.ros_manager import ROSManager

vtr_ui_logger = logging.getLogger('multi_robot_vtr_ui')
vtr_ui_logger.setLevel(logging.INFO)
hd = logging.StreamHandler()
fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
hd.setFormatter(fm)
vtr_ui_logger.addHandler(hd)


class MultiRobotVTRUI(ROSManager):
  """Client used to interface with the C++ MissionServer node."""
  class Notification(Enum):
    """Enumerates possible notifications that might come back from ROS; overloads parent definition"""
    Feedback = 0

  @ROSManager.on_ros
  def setup_ros(self, *args, **kwargs):
    """Sets up necessary ROS communications"""
    self._robot_ids = args[0]
    self._robot_namespaces = {robot_id: "/" + robot_id + "/vtr/" for robot_id in self._robot_ids}
    # graph state
    self._graph_state_cli = {}
    self._graph_state_sub = {}
    self._graph_update_sub = {}
    # robot state
    self._robot_state_cli = {}
    self._robot_state_sub = {}
    # route being followed
    self._following_route_cli = {}
    self._following_route_sub = {}
    # server state
    self._server_state_cli = {}
    self._server_state_sub = {}
    self._task_queue_state_cli = {}
    self._task_queue_update_sub = {}
    # graph manipulation
    self._move_graph_pub = {}
    self._annotate_route_pub = {}
    self._update_waypoint_pub = {}
    # env info
    self._change_env_info_pub = {}
    # mission command
    self._mission_command_pub = {}
    # map info
    self._map_info_cli = {}

    for robot_id in self._robot_ids:
      print(f"Setting up ROS for robot {robot_id} with namespace {self._robot_namespaces[robot_id]}")
      namespace = self._robot_namespaces[robot_id]
      self._graph_state_cli[robot_id] = self.create_client(GraphStateSrv, namespace + "graph_state_srv")
      while not self._graph_state_cli[robot_id].wait_for_service(timeout_sec=1.0):
        vtr_ui_logger.info("Waiting for graph_state_srv service for robot " + robot_id)
      self._graph_state_sub[robot_id] = self.create_subscription(GraphState, namespace + 'graph_state', 
                                                                      (lambda rid: (lambda data: self.graph_state_callback(data, rid)))(robot_id), 10)
      self._graph_update_sub[robot_id] = self.create_subscription(GraphUpdate, namespace + 'graph_update', 
                                                                      (lambda rid: (lambda data: self.graph_update_callback(data, rid)))(robot_id), 10)
      
      self._robot_state_cli[robot_id] = self.create_client(RobotStateSrv, namespace + "robot_state_srv")
      while not self._robot_state_cli[robot_id].wait_for_service(timeout_sec=1.0):
        vtr_ui_logger.info("Waiting for robot_state_srv service for robot " + robot_id)
      self._robot_state_sub[robot_id] = self.create_subscription(RobotState, namespace + 'robot_state', 
                                                                      (lambda rid: (lambda data: self.robot_state_callback(data, rid)))(robot_id), 10)

      # route being followed
      self._following_route_cli[robot_id] = self.create_client(FollowingRouteSrv, namespace + "following_route_srv")
      while not self._following_route_cli[robot_id].wait_for_service(timeout_sec=1.0):
        vtr_ui_logger.info("Waiting for following_route_srv service for robot " + robot_id)
      self._following_route_sub[robot_id] = self.create_subscription(GraphRoute, namespace + 'following_route', 
                                                                      (lambda rid: (lambda data: self.following_route_callback(data, rid)))(robot_id), 10)
      # mission command
      self._mission_command_pub[robot_id] = self.create_publisher(MissionCommand, namespace + 'mission_command', 1)
      self._server_state_sub[robot_id] = self.create_subscription(ServerState, namespace + 'server_state', 
                                                                      (lambda rid: (lambda data: self.server_state_callback(data, rid)))(robot_id), 10)
      self._server_state_cli[robot_id] = self.create_client(ServerStateSrv, namespace + "server_state_srv")
      while not self._server_state_cli[robot_id].wait_for_service(timeout_sec=1.0):
        vtr_ui_logger.info("Waiting for server_state_srv service for robot " + robot_id)

      # task queue
      self._task_queue_update_sub[robot_id] = self.create_subscription(TaskQueueUpdate, namespace + 'task_queue_update',
                                                                      (lambda rid: (lambda data: self.task_queue_update_callback(data, rid)))(robot_id), 10)
      self._task_queue_state_cli[robot_id] = self.create_client(TaskQueueStateSrv, namespace + "task_queue_state_srv")
      while not self._task_queue_state_cli[robot_id].wait_for_service(timeout_sec=1.0):
        vtr_ui_logger.info("Waiting for task_queue_state_srv service for robot " + robot_id)

      # graph manipulation
      self._move_graph_pub[robot_id] = self.create_publisher(MoveGraph, namespace + 'move_graph', 1)
      self._annotate_route_pub[robot_id] = self.create_publisher(AnnotateRoute, namespace + 'annotate_route', 1)
      self._update_waypoint_pub[robot_id] = self.create_publisher(UpdateWaypoint, namespace + 'update_waypoint', 1)

      # env info
      self._change_env_info_pub[robot_id] = self.create_publisher(EnvInfo, namespace + 'env_info', 1)

    # map center
    self._map_info_cli = self.create_client(MapInfoSrv, namespace + "map_info_srv")
    while not self._map_info_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for map_info_srv service for robot " + robot_id)

  @ROSManager.on_ros
  def get_graph_state(self):
    print("[MultiRobotVTRUI] get_graph_state called")
    vtr_ui_logger.info("Base class: Getting graph state for robot " + next(iter(self._robot_namespaces.values())))
    robot = next(iter(self._robot_ids))
    # Get the graph state for a single robot. It should match between all
    while not self._graph_state_cli[robot].wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Base Class: Waiting for graph_state_srv service for robot " + robot)
    
    
    
    print("Calling graph state service for robot " + robot)
    return next(iter(self._graph_state_cli.values())).call(GraphStateSrv.Request()).graph_state

  @ROSManager.on_ros
  def graph_state_callback(self, graph_state, robot_id):
    vtr_ui_logger.info(f"Base Class: Graph state callback for robot {robot_id}")
    namespace = self._robot_namespaces[robot_id]
    self.notify("graph_state", graph_state=graph_state)
    self.notify(namespace + "graph_state", graph_state=graph_state)

  @ROSManager.on_ros
  def graph_update_callback(self, graph_update, robot_id):
    namespace = self._robot_namespaces[robot_id]
    self.notify("graph_update", graph_update=graph_update)
    self.notify(namespace + "graph_update", graph_update=graph_update)

  @ROSManager.on_ros
  def get_robot_state(self, robot_id):
    return self._robot_state_cli[robot_id].call(RobotStateSrv.Request()).robot_state

  @ROSManager.on_ros
  def robot_state_callback(self, robot_state, robot_id):
    print(f"[MultiRobotVTRUI] robot_state_callback called for {robot_id}")
    self.notify("robot_state", robot_state=robot_state, robot_id=robot_id)

  @ROSManager.on_ros
  def get_map_info(self):
    print("[MultiRobotVTRUI] get_map_info called")
    vtr_ui_logger.info("Base class: Getting map info for robot")
    #while not next(iter(self._map_info_cli.values())).wait_for_service(timeout_sec=1.0):
    #  vtr_ui_logger.info("Base Class: Waiting for map_info_srv service for robot " + next(iter(self._robot_ids)))
    service = self._map_info_cli#[self._robot_ids[0]]

    print(service)
    res = service.call(MapInfoSrv.Request()) 
    print(f"[MultiRobotVTRUI] Map info: {res}")
    return res.map_info 

  @ROSManager.on_ros
  def get_following_route(self, robot_id):
    # This will need to change for independent routes per robot
    return self._following_route_cli[robot_id].call(FollowingRouteSrv.Request()).following_route

  @ROSManager.on_ros
  def following_route_callback(self, following_route, robot_id):
    self.notify("following_route", following_route=following_route, robot_id=robot_id)

  @ROSManager.on_ros
  def get_server_state(self, robot_id):
    return self._server_state_cli[robot_id].call(ServerStateSrv.Request()).server_state

  @ROSManager.on_ros
  def server_state_callback(self, server_state, robot_id):
    self.notify("server_state", server_state=server_state, robot_id=robot_id)
    #self.notify(namespace + "server_state", server_state=server_state)

  @ROSManager.on_ros
  def get_task_queue_state(self):
    print("[MultiRobotVTRUI] get_task_queue_state called")
    vtr_ui_logger.info("Base class: Getting task queue state for robot " + next(iter(self._robot_namespaces.values())))
    return next(iter(self._task_queue_state_cli.values())).call(TaskQueueStateSrv.Request()).task_queue_state

  @ROSManager.on_ros
  def task_queue_update_callback(self, task_queue_update, robot_id):
    namespace = self._robot_namespaces[robot_id]
    vtr_ui_logger.info(f"Base Class: Task queue update callback for robot {robot_id}")
    self.notify("task_queue_update", task_queue_update=task_queue_update)
    #self.notify(namespace + "task_queue_update", task_queue_update=task_queue_update)

  @ROSManager.on_ros
  def set_pause(self, msg):
    # For now, we assume coordinated action across all robots
    for robot_id in self._robot_ids:
      self._mission_command_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def add_goal(self, msg):
    for robot_id in self._robot_ids:
      self._mission_command_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def cancel_goal(self, msg):
    for robot_id in self._robot_ids:
      self._mission_command_pub[robot_id].publish(msg)
  
  @ROSManager.on_ros
  def begin_goals(self, msg):
    for robot_id in self._robot_ids:
      self._mission_command_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def move_robot(self, msg, robot_id):
    self._mission_command_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def merge(self, msg):
    for robot_id in self._robot_ids:
      self._mission_command_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def confirm_merge(self, msg):
    for robot_id in self._robot_ids:
      self._mission_command_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def continue_teach(self, msg):
    for robot_id in self._robot_ids:
      self._mission_command_pub[robot_id].publish(msg)
  
  @ROSManager.on_ros
  def update_waypoint(self, msg):
    for robot_id in self._robot_ids:
      self._update_waypoint_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def annotate_route(self, msg):
    for robot_id in self._robot_ids:
      self._annotate_route_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def move_graph(self, msg):
    for robot_id in self._robot_ids:
      self._move_graph_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def change_env_info(self, msg):
    for robot_id in self._robot_ids:
      self._change_env_info_pub[robot_id].publish(msg)

  @ROSManager.on_ros
  def get_robot_ids(self):
    return self._robot_ids  

if __name__ == "__main__":
  import time

  logger = logging.getLogger('default')
  logger.setLevel(logging.DEBUG)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("INITIALIZE ROS MANAGER")
  vtr_ui = MultiRobotVTRUI()  # ROS process start is non-blocking
  time.sleep(1)
  logger.info("INITIALIZE ROS MANAGER - DONE")

  time.sleep(2)

  logger.info("SHUTDOWN ROS MANAGER")
  vtr_ui.shutdown()
  logger.info("SHUTDOWN ROS MANAGER - DONE")
