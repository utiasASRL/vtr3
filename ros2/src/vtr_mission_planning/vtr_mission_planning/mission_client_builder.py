#!/usr/bin/env python

from functools import partial
from multiprocessing.managers import BaseManager, BaseProxy
from multiprocessing import current_process
from base64 import b64encode, b64decode

from .mission_client import MissionClient

ADDRESS = ''
PORT = 54687
AUTHKEY = b'vtr3-mission-client'


class MissionClientProxy(BaseProxy):
  """Multiprocessing.Manager proxy for a MissionClient"""

  # _exposed_ = ('add_goal', 'cancel_goal', 'cancel_all', 'move_goal',
  #              'set_pause', 'wait_for_status', 'respond_prompt', 'close_loop',
  #              '__getattr__')
  _exposed_ = ('set_pause', 'add_goal', 'cancel_goal', 'cancel_all',
               '__getattribute__')

  def set_pause(self, pause=True):
    """Sets the pause state of the mission server

    :param paused: whether or not to pause the server
    """
    return self._callmethod('set_pause', args=(pause,))

  def add_goal(self,
               goal_type,
               path=None,
               pause_before=0,
               pause_after=0,
               vertex=2**64 - 1):
    """Adds a new goal to the mission server, at the end of the queue

    :param goal_type: enum representing the type of goal to add
    :param path: list of vertices to visit
    :param pause_before: duration in seconds to pause before execution
    :param pause_after: duration in seconds to pause after execution
    :param vertex: target center vertex for relocalization/merge goals
    """
    return self._callmethod('add_goal',
                            args=(goal_type,),
                            kwds={
                                'path': path,
                                'vertex': vertex,
                                'pause_before': pause_before,
                                'pause_after': pause_after
                            })

  def cancel_goal(self, goal_id):
    """Cancels a currently tracked goal in the mission server

    :param goal_id: goal id to be cancelled
    """
    return self._callmethod('cancel_goal', args=(goal_id,))

  def cancel_all(self):
    """Cancel all goals currently being tracked by the mission server"""
    return self._callmethod('cancel_all')

  # def move_goal(self, goal_id, idx=-1, before=None):
  #   """Moves a currently tracked goal to a new position in the queue

  #       :param goal_id: id of the goal to move
  #       :param idx: index in the queue to move it to
  #       :param before: id of the goal that should come immediately after this goal
  #       """
  #   return self._callmethod('move_goal',
  #                           args=(goal_id,),
  #                           kwds={
  #                               'idx': idx,
  #                               'before': before
  #                           })

  # def wait_for_status(self, status):
  #   """Blocks until the MissionServer to enters a specific state

  #       :param status: status enum to wait for
  #       """
  #   return self._callmethod('wait_for_status', args=(status,))

  # def respond_prompt(self, pid, value, status=None):
  #   """Return a user response to a prompt

  #       :param pid: unique id of the prompt responded to
  #       :param value: the option selected by the user
  #       :param status: optional status string for timeouts/etc
  #       """
  #   return self._callmethod('respond_prompt', args=(pid, value, status))

  ### Old code for reference
  # def __getattr__(self, name):
  #   """Proxies direct class member access of proxied variables"""
  #   if name in [
  #       'goals', 'feedback', 'status', 'trunk_vertex', 'path_seq',
  #       'T_leaf_trunk', 'T_leaf_target', 'path', 'cov_leaf_target',
  #       'cov_leaf_trunk'
  #   ]:
  #     return self._callmethod('__getattr__', args=(name,))
  def __getattr__(self, name):
    """Proxies direct class member access of proxied variables"""
    if name in [
        'status', 'goals', 'trunk_vertex', 'path_seq', 't_leaf_trunk',
        't_leaf_target', 'path', 'cov_leaf_target', 'cov_leaf_trunk'
    ]:
      return self._callmethod('__getattribute__', args=(name,))


def build_master_client(client_cls=MissionClient, args=[], kwargs={}):
  """
  Builds a master mission client and publishes the connection parameters on rosparam
    :param node_name:   ROS node name for the mission client
    :param server_path: ROS node name of the server to connect to
    :param address:     Address(es) on which to bind the multiprocessing manager (defaults to all)
    :param port:        Port on which to bind the multiprocessing manager
    :param authkey:     Authentication key required to connect to the multiprocessing manager
    :param cls:         Specific subclass of MissionClient to instantiate

    :returns:   MissionClient, ClientManager
    :rtype:     cls, BaseManager
  """

  # Publish these so other clients can find the manager
  # rospy.set_param(node_name + '/address', address)
  # rospy.set_param(node_name + '/port',    port)
  # rospy.set_param(node_name + '/authkey', b64encode(authkey))
  # node_name = 'MissionClient'
  # server_path = '/MissionServer'
  # address = ''
  # port = 54687
  # authkey = current_process().authkey

  # a = tuple([node_name, server_path] + args)
  # k = kwargs
  # k['log_level'] = rospy.INFO
  # client = cls(args=a, kwargs=k)
  client = client_cls()

  # Local printout of events
  # client.register_callback(Notification.Cancel,   partial(print_event, Notification.Cancel))
  # client.register_callback(Notification.Complete, partial(print_event, Notification.Complete))
  # client.register_callback(Notification.Error,    partial(print_event, Notification.Error))
  # client.register_callback(Notification.Feedback, partial(print_event, Notification.Feedback))
  # client.register_callback(Notification.Started,  partial(print_event, Notification.Started))
  # client.register_callback(Notification.NewGoal,  partial(print_event, Notification.NewGoal))
  # client.register_callback(Notification.StatusChange, partial(print_event, Notification.StatusChange))

  class ClientManager(BaseManager):
    pass

  ClientManager.register('client',
                         callable=lambda: client,
                         proxytype=MissionClientProxy)
  mgr = ClientManager((ADDRESS, PORT), AUTHKEY)

  return client, mgr


def build_remote_client():
  """Connects to a remote manager and returns a mission client proxy object

  :param node_name: Namespace of the mission client node, for acquiring connection info
  """

  # node_name = 'MissionClient'
  # server_path = '/MissionServer'
  # address = ''
  # port = 54687
  # authkey = current_process().authkey

  # if not rospy.has_param(node_name + '/address') or not rospy.has_param(node_name + '/port') \
  #         or not rospy.has_param(node_name + '/authkey'):
  #   raise RuntimeError(
  #       "Client connection parameters have not been set.  Is the master client running?"
  #   )

  # # Get or set defaults from/in rosparam for the manager connection
  # address = rospy.get_param(node_name + '/address')
  # port = rospy.get_param(node_name + '/port')
  # authkey = rospy.get_param(
  #     node_name + '/authkey').data  # vtr3 TODO: confirm this is correct
  # authkey = b64decode(authkey)

  class RemoteClientManager(BaseManager):
    pass

  RemoteClientManager.register('client', proxytype=MissionClientProxy)

  rmgr = RemoteClientManager((ADDRESS, PORT), AUTHKEY)
  rmgr.connect()

  return rmgr.client()
