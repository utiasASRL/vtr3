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

import rclpy
from rclpy.node import Node
from vtr_navigation_msgs.msg import MissionCommand, ServerState, GraphState, RobotState, GoalHandle
from vtr_navigation_msgs.srv import GraphState as GraphStateSrv
from diagnostic_msgs.msg import DiagnosticStatus
import numpy as np
import time
from pyproj import CRS, Transformer

class ConvoyManager(Node):
  def __init__(self):
    super().__init__('convoy_manager')
    self.declare_parameter('robots', ["r1"])
    self.declare_parameter('path_planning.bicycle_mpc_follower.follow_distance', 2.0)
    self.declare_parameter('odometry.mapping.map_voxel_size', 0.3)
    self.declare_parameter('path_planning.bicycle_mpc_follower.distance_margin', 0.5)
    self.declare_parameter('path_planning.wheelbase', 0.5)
    self.declare_parameter('path_planning.max_ang_vel', 0.46)
    self._corridor_width = 0.2 # meters, appears to be decided by VTR, so this is set internally for now
    self._follow_distance = self.get_parameter('path_planning.bicycle_mpc_follower.follow_distance').value
    self._distance_margin = self.get_parameter('path_planning.bicycle_mpc_follower.distance_margin').value
    self._wheelbase = self.get_parameter('path_planning.wheelbase').value
    self._max_ang_vel = self.get_parameter('path_planning.max_ang_vel').value
    
    self._map_size= self.get_parameter('odometry.mapping.map_voxel_size').value

    self._robots = self.get_parameter('robots').value 
    self._robot_namespaces = {robot_id: "/" + robot_id + "/vtr/" for robot_id in self._robots}
    # Actually use map voxel size or mapping max_translation
    self._map_size= self.get_parameter('odometry.mapping.map_voxel_size').value

    self._server_state_subs = {}
    self._mission_command_pubs = {}
    self._robot_state_subs = {}
    self._robot_states = {robot_id: None for robot_id in self._robots}
    self._robot_vertex = {robot_id: None for robot_id in self._robots}
    self._localizing = False
    self._idle = True
    self._config_checked = False
    self._config_valid = False
    self._crs = None
    self._graph_state = None

    for robot_id in self._robots:
      namespace = self._robot_namespaces[robot_id]
      self._server_state_subs[robot_id] = self.create_subscription(ServerState, namespace + 'server_state', 
                                                                      (lambda rid: (lambda data: self.server_state_callback(data, rid)))(robot_id), 10)

      self._mission_command_pubs[robot_id] = self.create_publisher(MissionCommand, namespace + 'mission_command', 1)

      self._robot_state_subs[robot_id] = self.create_subscription(RobotState, namespace + 'robot_state',
                                                                      (lambda rid: (lambda data: self.robot_state_callback(data, rid)))(robot_id), 10)

    self._diagnostic_pub = self.create_publisher(DiagnosticStatus, '/vtr/mr_diagnostics', 10)
    self._diagnostic_timer = self.create_timer(0.2, self.publish_diagnostics)

    last_robot_namespace = self._robot_namespaces[self._robots[0]]
    self.get_logger().info(f"Last robot namespace: {last_robot_namespace}")
    self._graph_state_cli = self.create_client(GraphStateSrv, last_robot_namespace + 'graph_state_srv')

    while not self._graph_state_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info(f'service not available for {self._robots[0]}, waiting again...')


    self._server_states = {robot_id: None for robot_id in self._robots}

    self.get_logger().info(f"Convoy manager started for robots: {self._robots}")

    # Once VTR is running, localize the robots rather than using a set offset to assume they are in the right place
    self._localize_all()

  def LonLat_To_XY(self, lng, lat):
    if not self._crs:
      self._crs  = Transformer.from_crs("EPSG:4326", "EPSG:26917", always_xy=True)
    return self._crs.transform(lng, lat)

  def publish_diagnostics(self):
    msg = DiagnosticStatus()
    msg.name = "Convoy Manager"
    if self._config_checked:
      if self._config_valid:
        msg.level = DiagnosticStatus.OK
        msg.message = "Convoy configuration valid"
      else:
        msg.level = DiagnosticStatus.ERROR
        msg.message = "Convoy configuration invalid"
    else:
      msg.level = DiagnosticStatus.WARN
      msg.message = "Convoy configuration not yet checked"

    if self._localizing or self._idle:
      self._diagnostic_pub.publish(msg)
  
  def _localize_all(self,):
    self.get_logger().info("Setup complete notification received")
    # Appears we need to wait after services are set up for the GUI to setup. I didn't see an
    # obvious callback or signal so just wait a bit
    # TODO: Find a better way to do this, probably add an explicity 'READY' Signal from the GUI?
    time.sleep(5)
    # Iterate through robots and send a move robot command for each 
    for i in range(len(self._robots)):
      robot_id = self._robots[i]
      msg = MissionCommand()
      msg.type = MissionCommand.LOCALIZE
      # Map voxel size or mapping max_translation should be used here
      sid = (len(self._robots) - i - 1) * self._follow_distance // self._map_size
      msg.vertex = int(sid)
      self._mission_command_pubs[robot_id].publish(msg)
      self.get_logger().info(f"Published localize command to {robot_id} with sid {sid}")

  def robot_state_callback(self, msg, rid):
    # self.get_logger().info(f"Robot state callback for {rid}")
    if self._localizing:
      lon, lat, theta = msg.lng, msg.lat, msg.theta
      x, y = self.LonLat_To_XY(lon, lat)
      print(f"Robot {rid} localized to lon: {lon}, lat: {lat}, x: {x}, y: {y}, theta: {theta}")
      self._robot_states[rid] =  (x, y, theta)
      self._robot_vertex[rid] = msg.index
      self.get_logger().info(f"Robots localizing, checking for config if all robot states valid")
      if all(state is not None for state in self._robot_states.values()):
        self._config_checked = True
        self.get_logger().info(f"All robots localized, checking configuration validity")
        valid = self.check_valid_configuration()
        self._config_valid = valid
        if valid:
          self.get_logger().info(f"Configuration valid")
        else:
          self.get_logger().info(f"Configuration invalid, retrying localization")
      else:
        self.get_logger().info(f"Not all robots localized yet, waiting for more robot states")

  def check_valid_configuration(self):
    valid = True
    for i in range(1, len(self._robots)):
      valid &= self.check_follower_position(self._robot_states[self._robots[i-1]], self._robot_states[self._robots[i]])
    #for rid in self._robots:
      #valid &= self.check_robot_against_graph(self._robot_vertex[rid], self._robot_states[rid])
    return valid

  def check_follower_position(self, leader_pos, follower_pos):
    leader_x, leader_y, leader_theta = leader_pos
    follower_x, follower_y, follower_theta = follower_pos

    dx = follower_x - leader_x
    dy = follower_y - leader_y
    distance = np.sqrt(dx**2 + dy**2)

    expected_distance = self._follow_distance
    distance_margin = self._distance_margin

    if not (expected_distance - distance_margin <= distance <= expected_distance + distance_margin):
      self.get_logger().info(f"Follower at distance {distance} outside expected range [{expected_distance - distance_margin}, {expected_distance + distance_margin}]")
      return False

    angle_diff = (follower_theta - leader_theta + np.pi) % (2 * np.pi) - np.pi

    if abs(angle_diff) > np.pi / 4:  # 45 degrees tolerance
      self.get_logger().info(f"Follower angle difference {angle_diff} exceeds tolerance")
      return False

    return True

  def check_robot_against_graph(self, vertex_id, robot_pos):
    if self._graph_state is None:
      self._graph_state = self._graph_state_cli.call(GraphStateSrv.Request()).graph_state

    graph_vertex = self._graph_state.vertices[vertex_id]
    vertex_lon, vertex_lat = graph_vertex.lng, graph_vertex.lat
    vertex_x, vertex_y = self.LonLat_To_XY(vertex_lon, vertex_lat)

    dist_x, dist_y = robot_pos[0] - vertex_x, robot_pos[1] - vertex_y
    dist = np.sqrt(dist_x**2 + dist_y**2)
    if dist > self._corridor_width:
      self.get_logger().info(f"Robot at distance {dist} from graph vertex exceeds corridor width {self._corridor_width}")
      return False
    return True


  def server_state_callback(self, msg, rid):
    self.get_logger().info(f"Server state callback for {rid}: {msg.current_goal_state}")

    self._server_states[rid] = msg
    if msg.goals is not None and len(msg.goals) != 0 and msg.goals[0].type == GoalHandle.LOCALIZE and msg.current_goal_state == ServerState.RUNNING:
      self._localizing = True
      self._idle = False
    elif msg.goals is not None and len(msg.goals) != 0 and msg.goals[0].type == GoalHandle.IDLE and msg.current_goal_state == ServerState.RUNNING:
      self._idle = True
      self._localizing = False
    else:
      self._localizing = False

    # If any robot completes, send finish command to all robots
    if msg.current_goal_state == ServerState.FINISHING:
      mission_cmd = MissionCommand()
      for robot_id in self._robots:
        if robot_id != rid and (self._server_states[robot_id] is not None) and (self._server_states[robot_id].goals is not None) and len(self._server_states[robot_id].goals) > 0:
          active_goal = self._server_states[robot_id].goals[0]
          if active_goal.type == GoalHandle.REPEAT and msg.goals[0].type == GoalHandle.REPEAT and len(self._server_states[robot_id].goals) == len(msg.goals):
            mission_cmd.type = MissionCommand.CANCEL_GOAL
            mission_cmd.goal_handle = self._server_states[robot_id].goals[0]
            self.get_logger().info(f"Published cancel command to {robot_id}")
            self._mission_command_pubs[robot_id].publish(mission_cmd)
            self._server_states[robot_id] = None

def main():
  rclpy.init()
  convoy_manager = ConvoyManager()  # ROS process start is non-blocking
  rclpy.spin(convoy_manager)
  convoy_manager.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  import logging
  import time

  logger = logging.getLogger('default')
  logger.setLevel(logging.DEBUG)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("INITIALIZE CONVOY MANAGER")
  rclpy.init()
  convoy_manager = ConvoyManager()  # ROS process start is non-blocking
  
  time.sleep(1)
  logger.info("INITIALIZE CONVOY MANAGER - DONE")
  rclpy.spin(convoy_manager)

  time.sleep(2)

  logger.info("SHUTDOWN CONVOY MANAGER")
  convoy_manager.shutdown()
  logger.info("SHUTDOWN CONVOY MANAGER - DONE")

