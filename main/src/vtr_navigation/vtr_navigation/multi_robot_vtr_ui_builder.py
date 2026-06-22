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

from vtr_navigation.multi_robot_vtr_ui import MultiRobotVTRUI

ADDRESS = ''
PORT = 54687
AUTHKEY = b'vtr3-mission-client'


class MultiRobotVTRUIProxy(BaseProxy):
  """Multiprocessing.Manager proxy for a VTRUI object."""

  _exposed_ = ('get_graph_state', 'get_robot_state', 'get_following_route', 'get_server_state', 'get_task_queue_state',
               'set_pause', 'add_goal', 'cancel_goal', 'cancel_all_goals', 'begin_goals', 'update_waypoint', 'annotate_route', 'move_graph', 'move_robot', 
               'merge', 'confirm_merge', 'continue_teach', 'change_env_info', 'get_map_info', 'get_robot_ids')

  def get_robot_ids(self):
    print("[Proxy] get_robot_ids: calling _callmethod")
    result = self._callmethod('get_robot_ids')
    print(f"[Proxy] get_robot_ids: result={result}")
    return result

  def get_graph_state(self):
    print("[Proxy] get_graph_state: calling _callmethod")
    result = self._callmethod('get_graph_state')
    print(f"[Proxy] get_graph_state: result={result}")
    return result

  def get_robot_state(self, robot_id):
    print(f"[Proxy] get_robot_state({robot_id}): calling _callmethod")
    result = self._callmethod('get_robot_state', args=(robot_id,))
    print(f"[Proxy] get_robot_state({robot_id}): result={result}")
    return result

  def get_following_route(self, robot_id):
    print(f"[Proxy] get_following_route({robot_id}): calling _callmethod")
    result = self._callmethod('get_following_route', args=(robot_id,))
    return result

  def get_server_state(self, robot_id):
    print(f"[Proxy] get_server_state({robot_id}): calling _callmethod")
    result = self._callmethod('get_server_state', args=(robot_id,))
    return result

  def get_task_queue_state(self):
    print("[Proxy] get_task_queue_state: calling _callmethod")
    result = self._callmethod('get_task_queue_state')
    print(f"[Proxy] get_task_queue_state: result={result}")
    return result

  def set_pause(self, pause):
    print(f"[Proxy] set_pause({pause}): calling _callmethod")
    result = self._callmethod('set_pause', args=(pause,))
    print(f"[Proxy] set_pause({pause}): result={result}")
    return result

  def add_goal(self, command):
    print(f"[Proxy] add_goal({command}): calling _callmethod")
    result = self._callmethod('add_goal', args=(command,))
    print(f"[Proxy] add_goal({command}): result={result}")
    return result

  def cancel_goal(self, command, robot_id):
    print(f"[Proxy] cancel_goal({command}): calling _callmethod")
    result = self._callmethod('cancel_goal', args=(command, robot_id))
    print(f"[Proxy] cancel_goal({command}): result={result}")
    return result

  def cancel_all_goals(self):
    print(f"[Proxy] cancel_all_goals(): calling _callmethod")
    result = self._callmethod('cancel_all_goals')
    print(f"[Proxy] cancel_all_goals(): result={result}")
    return result

  def begin_goals(self):
    print("[Proxy] begin_goals: calling _callmethod")
    result = self._callmethod('begin_goals')
    print(f"[Proxy] begin_goals: result={result}")
    return result

  def merge(self, command):
    print(f"[Proxy] merge({command}): calling _callmethod")
    result = self._callmethod('merge', args=(command,))
    print(f"[Proxy] merge({command}): result={result}")
    return result

  def confirm_merge(self, command):
    print(f"[Proxy] confirm_merge({command}): calling _callmethod")
    result = self._callmethod('confirm_merge', args=(command,))
    print(f"[Proxy] confirm_merge({command}): result={result}")
    return result

  def continue_teach(self, command):
    print(f"[Proxy] continue_teach({command}): calling _callmethod")
    result = self._callmethod('continue_teach', args=(command,))
    print(f"[Proxy] continue_teach({command}): result={result}")
    return result

  def move_robot(self, command, robot_id):
    print(f"[Proxy] move_robot({command}, {robot_id}): calling _callmethod")
    result = self._callmethod('move_robot', args=(command, robot_id))
    print(f"[Proxy] move_robot({command}, {robot_id}): result={result}")
    return result
  
  def update_waypoint(self, update):
    print(f"[Proxy] update_waypoint({update}): calling _callmethod")
    result = self._callmethod('update_waypoint', args=(update,))
    print(f"[Proxy] update_waypoint({update}): result={result}")
    return result

  def annotate_route(self, annotation):
    print(f"[Proxy] annotate_route({annotation}): calling _callmethod")
    result = self._callmethod('annotate_route', args=(annotation,))
    print(f"[Proxy] annotate_route({annotation}): result={result}")
    return result

  def move_graph(self, move_graph_change):
    print(f"[Proxy] move_graph({move_graph_change}): calling _callmethod")
    result = self._callmethod('move_graph', args=(move_graph_change,))
    print(f"[Proxy] move_graph({move_graph_change}): result={result}")
    return result

  def change_env_info(self, env_info):
    print(f"[Proxy] change_env_info({env_info}): calling _callmethod")
    result = self._callmethod('change_env_info', args=(env_info,))
    print(f"[Proxy] change_env_info({env_info}): result={result}")
    return result
  
  def get_map_info(self):
    print("[Proxy] get_map_info: calling _callmethod")
    result = self._callmethod('get_map_info')
    print(f"[Proxy] get_map_info: result={result}")
    return result


def build_master(vtr_ui_class=MultiRobotVTRUI):
  """
  Builds a master mission client and publishes the connection parameters on rosparam
  Args:
    node_name:   ROS node name for the mission client
    merver_path: ROS node name of the server to connect to
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
  VTRUIManager.register('vtr_ui', callable=lambda: vtr_ui, proxytype=MultiRobotVTRUIProxy)
  print(f"Starting VTR UI Manager at {ADDRESS}:{PORT} with authkey {AUTHKEY.decode()}")
  mgr = VTRUIManager((ADDRESS, PORT), AUTHKEY)
  print("VTR UI Manager started")

  return vtr_ui, mgr


def build_remote():
  """Connects to a remote manager and returns a mission client proxy object"""

  class RemoteVTRUIManager(BaseManager):
    pass

  RemoteVTRUIManager.register('vtr_ui', proxytype=MultiRobotVTRUIProxy)

  rmgr = RemoteVTRUIManager((ADDRESS, PORT), AUTHKEY)
  rmgr.connect()
  print(f"Connected to remote VTR UI Manager at {ADDRESS}:{PORT}")

  return rmgr.vtr_ui()
