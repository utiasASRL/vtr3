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
import socketio

from vtr_mission_planning.mission_client import MissionClient
from vtr_mission_planning.ros_manager import RosManager
from vtr_messages.msg import RobotStatus, GraphUpdate, GraphPath

# socket io server address and port
# NOTE this must match the ones specified in socket_server.py
SOCKET_ADDRESS = 'localhost'
SOCKET_PORT = 5201

log = logging.getLogger('SocketClient')
log.setLevel(logging.INFO)


def vertex_to_json(v):
  """Converts a ROS vertex message into a JSON-serializable dictionary

  :param v: GlobalVertex message
  """
  p = v.t_vertex_world.position
  o = v.t_vertex_world.orientation
  return {
      'id': v.id,
      'T_vertex_world': {
          'position': [p.x, p.y, p.z],
          'orientation': [o.x, o.y, o.z, o.w]
      },
      'T_projected': [v.t_projected.x, v.t_projected.y, v.t_projected.theta],
      'neighbours': list(v.neighbours)
  }


class SocketMissionClient(MissionClient):
  """Subclass of a normal mission client that caches robot/path data and pushes
  notifications out over Socket.io
  """

  def __init__(self, group=None, name=None, args=(), kwargs={}):
    # super().__init__(group=group, name=name, args=args, kwargs=kwargs)
    super().__init__()

    self._socketio = None
    self._send = lambda x: log.info(
        "Dropping message because socket client isn't ready: %s", str(x))

  @RosManager.on_ros
  def setup_ros(self, *args, **kwargs):

    # Robot status
    self._path_seq = 0
    self._path = []
    self._trunk_vertex = None
    self._trunk_lng_lat_theta = [0, 0, 0]
    self._t_leaf_trunk = [0, 0, 0]
    self._cov_leaf_trunk = []
    self._target_vertex = None
    self._target_lng_lat_theta = [0, 0, 0]
    self._t_leaf_target = [0, 0, 0]
    self._cov_leaf_target = []

    self._status_sub = self.create_subscription(RobotStatus, 'robot',
                                                self.robot_callback, 1)
    self._graph_sub = self.create_subscription(GraphUpdate, 'graph_updates',
                                               self.graph_callback, 50)
    self._path_sub = self.create_subscription(GraphPath, 'out/following_path',
                                              self.path_callback, 1)

  def kill_server(self):
    """Kill the socket server because it doesn't die automatically"""
    log.info('Killing the SocketIO server.')
    self._socketio.emit('kill')

  def _after_start_hook(self):
    """Launch the socket client post-startup"""
    self._socketio = socketio.Client()
    self._socketio.connect('http://' + SOCKET_ADDRESS + ':' + str(SOCKET_PORT))
    self._send = lambda msg: self._socketio.emit('message', msg)

  def _after_listen_hook(self, func, args, kwargs):
    self._send({'type': func.name, 'args': args, 'kwargs': kwargs})

  @RosManager.on_ros
  def robot_callback(self, msg):
    """Callback when a new robot position is received"""
    self._path_seq = msg.path_seq

    self._trunk_vertex = msg.trunk_vertex
    self._trunk_lng_lat_theta = list(msg.lng_lat_theta)
    self._t_leaf_trunk = [
        msg.t_leaf_trunk.x,
        msg.t_leaf_trunk.y,
        msg.t_leaf_trunk.theta,
    ]
    self._cov_leaf_trunk = list(msg.cov_leaf_trunk)

    self._target_vertex = msg.target_vertex
    self._target_lng_lat_theta = list(msg.target_lng_lat_theta)
    self._t_leaf_target = [
        msg.t_leaf_target.x,
        msg.t_leaf_target.y,
        msg.t_leaf_target.theta,
    ]
    self._cov_leaf_target = list(msg.cov_leaf_target)

    self.notify(
        self.Notification.RobotChange,
        self._path_seq,
        self._trunk_vertex,
        self._trunk_lng_lat_theta,
        self._t_leaf_trunk,
        self._cov_leaf_trunk,
        self._target_vertex,
        self._target_lng_lat_theta,
        self._t_leaf_target,
        self._cov_leaf_target,
    )

  @RosManager.on_ros
  def graph_callback(self, msg):
    """Callback for incremental graph updates"""
    vals = {
        'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        'seq': msg.seq,
        'invalidate': msg.invalidate,
        'vertices': [vertex_to_json(v) for v in msg.vertices]
    }
    self.notify(self.Notification.GraphChange, vals)

  @RosManager.on_ros
  def path_callback(self, msg):
    self._path = list(msg.vertex_id_list)
    self.notify(self.Notification.PathChange, list(msg.vertex_id_list))

  @property
  @RosManager.on_ros
  def path(self):
    return self._path

  @property
  @RosManager.on_ros
  def path_seq(self):
    return self._path_seq

  @property
  @RosManager.on_ros
  def trunk_vertex(self):
    return self._trunk_vertex

  @property
  @RosManager.on_ros
  def trunk_lng_lat_theta(self):
    return self._trunk_lng_lat_theta

  @property
  @RosManager.on_ros
  def t_leaf_trunk(self):
    return self._t_leaf_trunk

  @property
  @RosManager.on_ros
  def cov_leaf_trunk(self):
    return self._cov_leaf_trunk

  @property
  @RosManager.on_ros
  def target_vertex(self):
    return self._target_vertex

  @property
  @RosManager.on_ros
  def target_lng_lat_theta(self):
    return self._target_lng_lat_theta

  @property
  @RosManager.on_ros
  def t_leaf_target(self):
    return self._t_leaf_target

  @property
  @RosManager.on_ros
  def cov_leaf_target(self):
    return self._cov_leaf_target
