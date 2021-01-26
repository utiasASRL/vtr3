#!/usr/bin/env python

from socketIO_client import SocketIO

from vtr_interface import SOCKET_ADDRESS, SOCKET_PORT
from vtr_mission_planning.mission_client import MissionClient
from vtr_mission_planning.ros_manager import RosManager
from vtr_messages.msg import RobotStatus, GraphUpdate

import logging
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
    self._trunk_vertex = None
    self._path_seq = 0
    self._path = []
    self._t_leaf_trunk = [0, 0, 0]
    self._t_leaf_target = [0, 0, 0]
    self._cov_leaf_trunk = []
    self._cov_leaf_target = []
    self._target_vertex = None

    self._status_sub = self.create_subscription(RobotStatus, 'robot',
                                                self.robot_callback, 1)
    self._graph_sub = self.create_subscription(GraphUpdate, 'graph_updates',
                                               self.graph_callback, 50)

  def kill_server(self):
    """Kill the socket server because it doesn't die automatically"""
    log.info('Killing the SocketIO server.')
    self._socketio.emit('kill')

  def _after_start_hook(self):
    """Launch the socket client post-startup"""
    self._socketio = SocketIO(SOCKET_ADDRESS,
                              SOCKET_PORT,
                              wait_for_connection=True)
    self._send = self._socketio.send

  def _after_listen_hook(self, func, args, kwargs):
    self._send({'type': func.name, 'args': args, 'kwargs': kwargs})

  @RosManager.on_ros
  def robot_callback(self, msg):
    """Callback when a new robot position is received"""
    self._trunk_vertex = msg.trunk_vertex
    self._target_vertex = msg.target_vertex
    self._path_seq = msg.path_seq
    self._t_leaf_trunk = [
        msg.t_leaf_trunk.x, msg.t_leaf_trunk.y, msg.t_leaf_trunk.theta
    ]
    self._t_leaf_target = [
        msg.t_leaf_target.x, msg.t_leaf_target.y, msg.t_leaf_target.theta
    ]
    self._cov_leaf_trunk = list(msg.cov_leaf_trunk)
    self._cov_leaf_target = list(msg.cov_leaf_target)

    self.notify(self.Notification.RobotChange, msg.trunk_vertex, msg.path_seq,
                self._t_leaf_trunk, self._cov_leaf_trunk, msg.target_vertex,
                self._t_leaf_target, self._cov_leaf_target)

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

  @property
  @RosManager.on_ros
  def trunk_vertex(self):
    return self._trunk_vertex

  @property
  @RosManager.on_ros
  def path_seq(self):
    return self._path_seq

  @property
  @RosManager.on_ros
  def t_leaf_trunk(self):
    return self._t_leaf_trunk

  @property
  @RosManager.on_ros
  def t_leaf_target(self):
    return self._t_leaf_target

  @property
  @RosManager.on_ros
  def cov_leaf_trunk(self):
    return self._cov_leaf_trunk

  @property
  @RosManager.on_ros
  def cov_leaf_target(self):
    return self._cov_leaf_target

  @property
  @RosManager.on_ros
  def path(self):
    return self._path