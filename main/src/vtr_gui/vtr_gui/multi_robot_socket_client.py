
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
import time
import logging
import socketio

from vtr_navigation.multi_robot_vtr_ui import MultiRobotVTRUI
from vtr_navigation.multi_robot_vtr_ui_builder import build_master

'''
from vtr_gui.socket_client import goal_handle_from_ros, \
    graph_state_from_ros, graph_update_from_ros, \
    following_route_from_ros, map_info_from_ros, \
    task_queue_update_from_ros, task_queue_state_from_ros, \
    task_queue_task_from_ros, robot_state_from_ros, \
    server_state_from_ros, map_info_from_ros, \
    goal_handle_from_ros, map_info_from_ros

from vtr_gui.socket_client import SOCKET_ADDRESS, SOCKET_PORT, vtr_ui_logger
'''

from vtr_tactic_msgs.msg import EnvInfo
from vtr_navigation_msgs.msg import MoveGraph, AnnotateRoute, UpdateWaypoint
from vtr_navigation_msgs.msg import MissionCommand, ServerState, GoalHandle

# socket io server address and port
# NOTE this must match the ones specified in socket_server.py
SOCKET_ADDRESS = 'localhost'
SOCKET_PORT = 5201

vtr_ui_logger = logging.getLogger('multi_robot_vtr_ui')  # set up in vtr_ui.py
vtr_ui_logger.setLevel(logging.INFO)
hd = logging.StreamHandler()
fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
hd.setFormatter(fm)
vtr_ui_logger.addHandler(hd)

def graph_state_from_ros(ros_graph_state):
  return {
      'vertices': [{
          'id': v.id,
          'neighbors': [n for n in v.neighbors],
          'lng': v.lng,
          'lat': v.lat,
          'theta': v.theta,
          'type': v.type,
          'name': v.name
      } for v in ros_graph_state.vertices],
      'fixed_routes': [{
          'ids': [id for id in r.ids],
          'type': r.type
      } for r in ros_graph_state.fixed_routes],
      'active_routes': [{
          'ids': [id for id in r.ids],
          'type': r.type
      } for r in ros_graph_state.active_routes],
  }


def graph_update_from_ros(ros_graph_update):
  vf = ros_graph_update.vertex_from
  vt = ros_graph_update.vertex_to
  return {
      'vertex_from': {
          'id': vf.id,
          'neighbors': [n for n in vf.neighbors],
          'lng': vf.lng,
          'lat': vf.lat,
          'theta': vf.theta,
          'type': vf.type,
          'name': vf.name
      },
      'vertex_to': {
          'id': vt.id,
          'neighbors': [n for n in vt.neighbors],
          'lng': vt.lng,
          'lat': vt.lat,
          'theta': vt.theta,
          'type': vt.type,
          'name': vt.name
      },
  }


def robot_state_from_ros(ros_robot_state):
  return {
      'valid': ros_robot_state.valid,
      'lng': ros_robot_state.lng,
      'lat': ros_robot_state.lat,
      'theta': ros_robot_state.theta,
      'localized': ros_robot_state.localized,
      'target_valid': ros_robot_state.target_valid,
      'target_lng': ros_robot_state.target_lng,
      'target_lat': ros_robot_state.target_lat,
      'target_theta': ros_robot_state.target_theta,
      'target_localized': ros_robot_state.target_localized,
  }

def map_info_from_ros(ros_map_info):
  return {
      'lat': ros_map_info.lat,
      'lng': ros_map_info.lng,
      'theta': ros_map_info.theta,
      'scale': ros_map_info.scale,
  }

def following_route_from_ros(ros_following_route):
  return {'ids': [id for id in ros_following_route.ids]}


def goal_handle_from_ros(ros_goal_handle):
  goal_handle = dict()
  # goal id
  goal_handle["id"] = [int(x) for x in ros_goal_handle.id]
  # goal type
  if ros_goal_handle.type == GoalHandle.IDLE:
    goal_handle["type"] = "idle"
  elif ros_goal_handle.type == GoalHandle.TEACH:
    goal_handle["type"] = "teach"
  elif ros_goal_handle.type == GoalHandle.REPEAT:
    goal_handle["type"] = "repeat"
  elif ros_goal_handle.type == GoalHandle.LOCALIZE:
    goal_handle["type"] = "localize"
  elif ros_goal_handle.type == GoalHandle.PAUSE:
    goal_handle["type"] = "pause"
  elif ros_goal_handle.type == GoalHandle.SELECT_CONTROLLER:
    goal_handle["type"] = "select"
  else:
    goal_handle["type"] = "unknown"
  # pause before
  goal_handle["pause_before"] = ros_goal_handle.pause_before / 1000.0
  # pause after
  goal_handle["pause_after"] = ros_goal_handle.pause_after / 1000.0
  # waypointsk
  goal_handle["waypoints"] = [x for x in ros_goal_handle.waypoints]

  return goal_handle


def server_state_from_ros(ros_server_state):
  server_state = dict()
  # server state
  if ros_server_state.server_state == ServerState.EMPTY:
    server_state["server_state"] = "EMPTY"
  elif ros_server_state.server_state == ServerState.PROCESSING:
    server_state["server_state"] = "PROCESSING"
  elif ros_server_state.server_state == ServerState.PENDING_PAUSE:
    server_state["server_state"] = "PENDING_PAUSE"
  elif ros_server_state.server_state == ServerState.PAUSED:
    server_state["server_state"] = "PAUSED"
  else:
    server_state["server_state"] = "UNKNOWN"
  # current goal id
  server_state["current_goal_id"] = [int(x) for x in ros_server_state.current_goal_id]
  # current goal state
  if ros_server_state.current_goal_state == ServerState.EMPTY:
    server_state["current_goal_state"] = "EMPTY"
  elif ros_server_state.current_goal_state == ServerState.STARTING:
    server_state["current_goal_state"] = "STARTING"
  elif ros_server_state.current_goal_state == ServerState.RUNNING:
    server_state["current_goal_state"] = "RUNNING"
  elif ros_server_state.current_goal_state == ServerState.FINISHING:
    server_state["current_goal_state"] = "FINISHING"
  else:
    server_state["current_goal_state"] = "UNKNOWN"
  # current goals
  server_state["goals"] = [goal_handle_from_ros(gh) for gh in ros_server_state.goals]
  return server_state

def diagnostics_info_from_ros(ros_diagnostics):
  return {
          'level': str(ros_diagnostics.level),
          'name': ros_diagnostics.name,
          'message': ros_diagnostics.message,
  }

def task_queue_task_from_ros(ros_task_queue_task):
  return {
      'id': ros_task_queue_task.id,
      'name': ros_task_queue_task.name,
      'vid': ros_task_queue_task.vid,
  }


def task_queue_update_from_ros(ros_task_queue_update):
  return {
      'type': ros_task_queue_update.type,
      'task': task_queue_task_from_ros(ros_task_queue_update.task),
  }


def task_queue_state_from_ros(ros_task_queue_state):
  return {
      'tasks': [task_queue_task_from_ros(t) for t in ros_task_queue_state.tasks],
  }

class MultiRobotSocketVTRUI(MultiRobotVTRUI):
  """Subclass of a normal mission client that caches robot/path data and pushes
  notifications out over Socket.io
  """

  def __init__(self):
    super().__init__()

    self._socketio = socketio.Client()
    while True:
      try:
        self._socketio.connect('http://' + SOCKET_ADDRESS + ':' + str(SOCKET_PORT))
        break
      except socketio.exceptions.ConnectionError:
        vtr_ui_logger.info("Waiting for socket io server...")
        time.sleep(1)
    self._send = lambda name, msg: self._socketio.emit("notification/" + name, msg)

  def get_graph_state(self):
    ros_graph_state = super().get_graph_state()
    return graph_state_from_ros(ros_graph_state)

  def get_robot_state(self, idx):
    ros_robot_state = super().get_robot_state(idx)
    return robot_state_from_ros(ros_robot_state)
  
  def get_map_info(self):
    ros_map_info = super().get_map_info()
    return map_info_from_ros(ros_map_info)

  def get_server_state(self, robot_id):
    ros_server_state = super().get_server_state(robot_id)
    return server_state_from_ros(ros_server_state)

  def get_task_queue_state(self):
    ros_task_queue_state = super().get_task_queue_state()
    return task_queue_state_from_ros(ros_task_queue_state)

  def get_following_route(self, robot_id):
    ros_following_route = super().get_following_route(robot_id)
    return following_route_from_ros(ros_following_route)

  def set_pause(self, data):
    ros_command = MissionCommand()
    ros_command.type = MissionCommand.PAUSE
    ros_command.pause = bool(data['pause'])
    return super().set_pause(ros_command)

  def add_goal(self, data):
    ros_command = MissionCommand()
    ros_command.type = MissionCommand.ADD_GOAL
    if (data['type'] == 'repeat'):
      ros_command.goal_handle.type = GoalHandle.REPEAT
    elif (data['type'] == 'teach'):
      ros_command.goal_handle.type = GoalHandle.TEACH
    elif (data['type'] == 'localize'):
      ros_command.goal_handle.type = GoalHandle.LOCALIZE
    else:
      ros_command.goal_handle.type = GoalHandle.IDLE
    ros_command.goal_handle.pause_before = int(data['pause_before'] * 1000)
    ros_command.goal_handle.pause_after = int(data['pause_after'] * 1000)
    ros_command.goal_handle.waypoints = [int(id) for id in data['waypoints']]
    return super().add_goal(ros_command)

  def cancel_goal(self, data, robot_id):
    ros_command = MissionCommand()
    ros_command.type = MissionCommand.CANCEL_GOAL
    ros_command.goal_handle.id = [int(id) for id in data['id']]
    return super().cancel_goal(ros_command, robot_id)
  
  def cancel_all_goals(self):
    ros_command = MissionCommand()
    ros_command.type = MissionCommand.CANCEL_GOAL
    ros_command.goal_handle.id = [int(0) for i in range(16)]
    return super().cancel_goal(ros_command, robot_id=None)

  def begin_goals(self):
    ros_command = MissionCommand()
    ros_command.type = MissionCommand.BEGIN_GOALS
    return super().begin_goals(ros_command)

  def move_robot(self, data, robot_id):
    ros_command = MissionCommand()
    ros_command.type = MissionCommand.LOCALIZE
    ros_command.vertex = int(data['vertex'])
    return super().move_robot(ros_command, robot_id)
  
  def update_waypoint(self, data):
    ros_waypoint_update = UpdateWaypoint()
    ros_waypoint_update.vertex_id = int(data['vertex_id'])
    ros_waypoint_update.type = int(data['type'])
    if int(data['type']) == UpdateWaypoint.ADD:
      ros_waypoint_update.name = data['name']
    else:
      ros_waypoint_update.name = ""
    return super().update_waypoint(ros_waypoint_update)

  def annotate_route(self, data):
    ros_annotate_route = AnnotateRoute()
    ros_annotate_route.type = int(data['type'])
    ros_annotate_route.ids = [int(id) for id in data['ids']]
    return super().annotate_route(ros_annotate_route)

  def move_graph(self, data):
    ros_move_graph = MoveGraph()
    ros_move_graph.lng = float(data['lng'])
    ros_move_graph.lat = float(data['lat'])
    ros_move_graph.theta = float(data['theta'])
    ros_move_graph.scale = float(data['scale'])
    return super().move_graph(ros_move_graph)

  def change_env_info(self, data):
    ros_env_info = EnvInfo()
    ros_env_info.terrain_type = int(data['terrain_type'])
    return super().change_env_info(ros_env_info)

  def _notify_hook(self, name, *args, **kwargs):
    if name == 'diagnostics':
      self._send(name, {'diagnostics': diagnostics_info_from_ros(kwargs["diagnostics"])})
    if name == 'graph_state':
      self._send(name, {'graph_state': graph_state_from_ros(kwargs["graph_state"])})
    if name == 'graph_update':
      self._send(name, {'graph_update': graph_update_from_ros(kwargs["graph_update"])})
    if name == 'robot_state':
      robot_state = robot_state_from_ros(kwargs["robot_state"])
      robot_id = kwargs.get("robot_id", "default")
      self._send(name, {'robot_state': robot_state, 'robot_id': robot_id})
    if name == 'server_state':
      server_state = server_state_from_ros(kwargs["server_state"])
      robot_id = kwargs.get("robot_id", "default")
      self._send(name, {'server_state': server_state, 'robot_id': robot_id})
    if name == 'following_route':
      robot_id = kwargs.get("robot_id", "default")
      following_route = following_route_from_ros(kwargs["following_route"])
      self._send(name, {'following_route': following_route, 'robot_id': robot_id})
    if name == 'task_queue_update':
      self._send(name, {'task_queue_update': task_queue_update_from_ros(kwargs["task_queue_update"])})
    

def main():
  vtr_gui, mgr = build_master(MultiRobotSocketVTRUI)
  mgr.get_server().serve_forever()

  vtr_gui.shutdown()

if __name__ == '__main__':
  main()