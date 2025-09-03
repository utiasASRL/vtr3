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
from vtr_navigation_msgs.msg import MissionCommand, ServerState, GraphState
from vtr_navigation_msgs.srv import GraphState as GraphStateSrv
import time

## TODO: This probably belongs elsewhere, maybe we move it and future multirobot stuff to vtr_multirobot?

class ConvoyManager(Node):
  def __init__(self):
    super().__init__('convoy_manager')
    self.declare_parameter('robots', ["r1"])
    self.declare_parameter('path_planning.follow_distance', 2.0)
    self.declare_parameter('odometry.mapping.map_voxel_size', 0.3)
    self._follow_distance = self.get_parameter('path_planning.follow_distance').value
    self._map_size= self.get_parameter('odometry.mapping.map_voxel_size').value


    self._robots = self.get_parameter('robots').value 
    self._robot_namespaces = {robot_id: "/" + robot_id + "/vtr/" for robot_id in self._robots}
    # Actually use map voxel size or mapping max_translation
    self._map_size= self.get_parameter('odometry.mapping.map_voxel_size').value

    self._server_state_subs = {}
    self._mission_command_pubs = {}

    for robot_id in self._robots:
      namespace = self._robot_namespaces[robot_id]
      self._server_state_subs[robot_id] = self.create_subscription(ServerState, namespace + 'server_state', 
                                                                      (lambda rid: (lambda data: self.server_state_callback(data, rid)))(robot_id), 10)

      self._mission_command_pubs[robot_id] = self.create_publisher(MissionCommand, namespace + 'mission_command', 1)

    last_robot_namespace = self._robot_namespaces[self._robots[-1]]
    self.get_logger().info(f"Last robot namespace: {last_robot_namespace}")
    self._graph_state_cli = self.create_client(GraphStateSrv, namespace + 'graph_state_srv')

    while not self._graph_state_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info(f'service not available for {self._robots[-1]}, waiting again...')

    # Once VTR is running, localize the robots rather than using a set offset to assume they are in the right place
    self._localize_all()

  def _localize_all(self,):
    self.get_logger().info("Setup complete notification received")
    
    # Appears we need to wait after services are set up for the GUI to setup. I didn't see an
    # obvious callback or signal so just wait a bit
    time.sleep(1.0)
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
    

  def server_state_callback(self, msg, rid):
    # If any robot completes, send finish command to all robots
    if msg.current_goal_state == ServerState.FINISHING:
      mission_cmd = MissionCommand()
      mission_cmd.header.stamp = rospy.Time.now()
      mission_cmd.type = MissionCommand.FINISH
      for robot_id, pub in self._mission_command_pubs.items():
        self.get_logger().info(f"Published mission command to {robot_id}")
        pub.publish(mission_cmd)

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

