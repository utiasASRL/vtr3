#!/usr/bin/env python

from enum import Enum

import rospy
import actionlib
from socketIO_client import SocketIO

# from std_msgs.msg import Empty

from vtr_interface import SOCKET_ADDRESS, SOCKET_PORT
from vtr_planning import BaseMissionClient, ros_locked, build_master

# from asrl__messages.msg import RobotStatus, Path
# from asrl__interface.msg import UserPromptAction, UserPromptResult, UserPromptFeedback
# from asrl__safety_monitor.msg import MonitorDebug
# from asrl__pose_graph.msg import GraphUpdate

import logging
log = logging.getLogger('SocketClient')
log.setLevel(logging.INFO)


class SocketMissionClient(BaseMissionClient):
  """Subclass of a normal mission client that caches robot/path data and pushes 
  notifications out over Socket.io
  """

  # # TODO enum must be defined outside the class. No longer true in python 3.
  # Prompt = Prompt

  def __init__(self, group=None, name=None, args=(), kwargs={}):
    super().__init__(group=group, name=name, args=args, kwargs=kwargs)

    self.socketio = None
    self._send = lambda x: log.info(
        "Dropping message because socket client isn't ready: %s", str(x))

  def kill_server(self):
    """Kill the socket server because it doesn't die automatically"""
    log.info('Killing the SocketIO server.')
    self.socketio.emit('kill')

  def _post_start(self):
    """Launch the socket client post-startup"""
    self.socketio = SocketIO(SOCKET_ADDRESS,
                             SOCKET_PORT,
                             wait_for_connection=True)
    self._send = self.socketio.send

  def _notify_loop(self):
    """Listens to the notification queue and invokes callbacks in the main
    process
    """
    while True:
      kind, args, kwargs = self.notify.get()

      # Use a lock here so that we can't modify the callbacks while we are calling them
      with self.lock:
        _ = [f(*args, **kwargs) for f in self._callbacks.get(kind, {}).values()]

      self._send({'type': kind.name, 'args': args, 'kwargs': kwargs})

  def _ros_setup(self, *args, **kwargs):
    super()._ros_setup(*args, **kwargs)


# Do not remove this if statement; it will cause fork bombs
if __name__ == '__main__':
  client, mgr = build_master(server_path=rospy.remap_name('/Navigation'),
                             cls=SocketMissionClient)

  # This has to happen in a specific order:

  # 1) Start the mission client, spawning a separate process with ROS in it
  client.start()

  # 2) Start a python Multiprocessing.Manager that contains the client.  This blocks the main process.
  mgr.get_server().serve_forever()

  # 3) The server is interrupted (Ctrl-C), which shuts down the Multiprocessing.Manager
  # 4) Shut down the client, stopping ROS and joining the process that was spawned in (1)
  client.shutdown()

  # 5) Tell the web server to exit using a SocketIO request, as it cannot be killed cleanly from the terminal
  client.kill_server()
