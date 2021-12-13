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

import socketio

from vtr_navigation_v2.vtr_ui import VTRUI
from vtr_navigation_v2.vtr_ui_builder import build_master

from vtr_navigation_msgs.msg import MoveGraph, AnnotateRoute

# socket io server address and port
# NOTE this must match the ones specified in socket_server.py
SOCKET_ADDRESS = 'localhost'
SOCKET_PORT = 5201


def graph_state_from_ros(ros_graph_state):
  return {
      'vertices': [{
          'id': v.id,
          'neighbors': [n for n in v.neighbors],
          'lng': v.lng,
          'lat': v.lat,
          'theta': v.theta,
          'type': v.type,
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
      },
      'vertex_to': {
          'id': vt.id,
          'neighbors': [n for n in vt.neighbors],
          'lng': vt.lng,
          'lat': vt.lat,
          'theta': vt.theta,
          'type': vt.type,
      },
  }


class SocketVTRUI(VTRUI):
  """Subclass of a normal mission client that caches robot/path data and pushes
  notifications out over Socket.io
  """

  def __init__(self):
    super().__init__()

    self._socketio = socketio.Client()
    self._socketio.connect('http://' + SOCKET_ADDRESS + ':' + str(SOCKET_PORT))
    self._send = lambda name, msg: self._socketio.emit("notification/" + name, msg)

  def get_graph_state(self):
    ros_graph_state = super().get_graph_state()
    return graph_state_from_ros(ros_graph_state)

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

  def _notify_hook(self, name, *args, **kwargs):
    if name == 'graph_state':
      self._send(name, {'graph_state': graph_state_from_ros(kwargs["graph_state"])})
    if name == 'graph_update':
      self._send(name, {'graph_update': graph_update_from_ros(kwargs["graph_update"])})


def main():
  vtr_gui, mgr = build_master(SocketVTRUI)
  mgr.get_server().serve_forever()

  vtr_gui.shutdown()


if __name__ == '__main__':
  main()
