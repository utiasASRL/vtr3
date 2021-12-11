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

# socket io server address and port
# NOTE this must match the ones specified in socket_server.py
SOCKET_ADDRESS = 'localhost'
SOCKET_PORT = 5201


class SocketVTRUI(VTRUI):
  """Subclass of a normal mission client that caches robot/path data and pushes
  notifications out over Socket.io
  """

  def __init__(self):
    super().__init__()

    self._socketio = socketio.Client()
    self._socketio.connect('http://' + SOCKET_ADDRESS + ':' + str(SOCKET_PORT))
    self._send = lambda name, msg: self._socketio.emit("notification/" + name, msg)

  def get_graph_state2(self):
    ros_graph_state = super().get_graph_state()
    # to json serializable format
    graph_state = {
        'vertices': [{
            'id': v.id,
            'lng': v.lng,
            'lat': v.lat,
            'theta': v.theta,
            'neigobors': [n for n in v.neighbors],
        } for v in ros_graph_state.vertices],
        'fixed_routes': [{
            'ids': [id for id in r.ids],
            'type': r.type
        } for r in ros_graph_state.fixed_routes],
        'active_routes': [{
            'ids': [id for id in r.ids],
            'type': r.type
        } for r in ros_graph_state.active_routes],
        'current_route': {
            'ids': [id for id in ros_graph_state.current_route.ids],
            'type': ros_graph_state.current_route.type,
        },
    }
    return graph_state

  def _after_listen_hook(self, name, args, kwargs):
    self._send(name, {'args': args, 'kwargs': kwargs})


def main():
  vtr_gui, mgr = build_master(SocketVTRUI)
  mgr.get_server().serve_forever()

  vtr_gui.shutdown()


if __name__ == '__main__':
  main()
