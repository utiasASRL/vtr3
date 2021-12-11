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

from vtr_navigation_msgs.srv import GraphState

from vtr_navigation_v2.ros_manager import ROSManager

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
    self._graph_state_cli = self.create_client(GraphState, "/vtr/graph_state_srv")  # TODO: put in namespace
    while not self._graph_state_cli.wait_for_service(timeout_sec=1.0):
      vtr_ui_logger.info("Waiting for graph_state_srv service...")

  @ROSManager.on_ros
  def get_graph_state(self):
    response = self._graph_state_cli.call(GraphState.Request())
    return response.graph_state


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
