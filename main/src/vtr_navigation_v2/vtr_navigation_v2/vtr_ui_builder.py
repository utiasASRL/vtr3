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

from multiprocessing.managers import BaseManager, BaseProxy

from vtr_navigation_v2.vtr_ui import VTRUI

ADDRESS = ''
PORT = 54687
AUTHKEY = b'vtr3-mission-client'


class VTRUIProxy(BaseProxy):
  """Multiprocessing.Manager proxy for a VTRUI object."""

  _exposed_ = ('get_graph_state', 'get_robot_state', 'get_server_state', 'set_pause', 'add_goal', 'cancel_goal',
               'annotate_route', 'move_graph')

  def get_graph_state(self):
    return self._callmethod('get_graph_state')

  def get_robot_state(self):
    return self._callmethod('get_robot_state')

  def get_server_state(self):
    return self._callmethod('get_server_state')

  def set_pause(self, pause):
    return self._callmethod('set_pause', args=(pause,))

  def add_goal(self, command):
    return self._callmethod('add_goal', args=(command,))

  def cancel_goal(self, command):
    return self._callmethod('cancel_goal', args=(command,))

  def annotate_route(self, annotation):
    return self._callmethod('annotate_route', args=(annotation,))

  def move_graph(self, move_graph_change):
    return self._callmethod('move_graph', args=(move_graph_change,))


def build_master(vtr_ui_class=VTRUI):
  """
  Builds a master mission client and publishes the connection parameters on rosparam
  Args:
    node_name:   ROS node name for the mission client
    server_path: ROS node name of the server to connect to
    address:     Address(es) on which to bind the multiprocessing manager (defaults to all)
    port:        Port on which to bind the multiprocessing manager
    authkey:     Authentication key required to connect to the multiprocessing manager
    cls:         Specific subclass of MissionClient to instantiate
  Return
    (MissionClient) MissionClient instance
  """

  class VTRUIManager(BaseManager):
    pass

  vtr_ui = vtr_ui_class()
  VTRUIManager.register('vtr_ui', callable=lambda: vtr_ui, proxytype=VTRUIProxy)
  mgr = VTRUIManager((ADDRESS, PORT), AUTHKEY)

  return vtr_ui, mgr


def build_remote():
  """Connects to a remote manager and returns a mission client proxy object"""

  class RemoteVTRUIManager(BaseManager):
    pass

  RemoteVTRUIManager.register('vtr_ui', proxytype=VTRUIProxy)

  rmgr = RemoteVTRUIManager((ADDRESS, PORT), AUTHKEY)
  rmgr.connect()

  return rmgr.vtr_ui()
