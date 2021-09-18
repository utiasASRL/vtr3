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

from vtr_mission_planning.mission_client import MissionClient

# socket io server address and port
# NOTE this must match the ones specified in socket_server.py
SOCKET_ADDRESS = 'localhost'
SOCKET_PORT = 5201


class SocketMissionClient(MissionClient):
  """Subclass of a normal mission client that caches robot/path data and pushes
  notifications out over Socket.io
  """

  def __init__(self):
    super().__init__()

    self._socketio = socketio.Client()
    self._socketio.connect('http://' + SOCKET_ADDRESS + ':' + str(SOCKET_PORT))
    self._send = lambda msg: self._socketio.emit('message', msg)

  def _after_listen_hook(self, notification, args, kwargs):
    self._send({'type': notification.name, 'args': args, 'kwargs': kwargs})
