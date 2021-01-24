#!/usr/bin/env python
"""Defines a class that isolates ROS in a separate process.
This file contains a lot of really finicky multiprocessing stuff, and the
order in which things are spawned is very important.
Don't touch this unless you're prepared to deal with concurrency issues.
"""

import abc
import functools
import logging
import uuid

from enum import Enum
from multiprocessing import Process, Queue
from threading import Thread, RLock, Event

import rclpy
from rclpy.node import Node


class RosWorker(Node):

  def __init__(self, notify, notification, call_queue, return_queue,
               setup_calls, cleanup_calls, methods):
    rclpy.init()
    super().__init__('mission_client')

    self.Notification = notification
    self._notify = notify
    self._call_queue = call_queue
    self._return_queue = return_queue

    self._lock = RLock()
    for k, v in methods.items():

      def call(*args, _v=v, **kwargs):
        with self._lock:
          return _v(self, *args, **kwargs)

      setattr(self, k, call)

    # setup and cleanup calls are treated differently to support class inheritance
    self.setup_calls = setup_calls
    self.cleanup_calls = cleanup_calls

    self.setup_ros()

    self._listener = Thread(target=self._listen)
    self._listener.daemon = True
    self._listener.start()

    rclpy.spin(self)

  def _listen(self):
    """Listens for incoming ROS commands from the main process"""
    while True:
      func, args, kwargs = self._call_queue.get()
      print("Ros process is calling", func)
      self._return_queue.put(getattr(self, func)(*args, **kwargs))

  def setup_ros(self, *args, **kwargs):
    """Sets up necessary ROS communications"""
    for c in self.setup_calls:
      c(self, *args, **kwargs)

  def cleanup_ros(self, *args, **kwargs):
    """Destropy ROS communications"""
    for c in reversed(self.cleanup_calls):
      c(self, *args, **kwargs)

  def notify(self, name, *args, **kwargs):
    print("Ros process is notifying", name)
    self._notify.put((name, args, kwargs))

  def shutdown(self):
    self.cleanup_ros()
    self.destroy_node()
    rclpy.shutdown()


class RosManager():
  """Manages ROS to non-ROS communications.
  This class spawns it's own ROS node in a separate process, and proxies
  data/commands to and from it to provide some level of isolation. When start()
  is called, two copies of the class will exist: one in your process and one in
  a new process that becomes a ROS node.
  In general, things that begin with an _ are not meant to be called directly;
  their behaviour is undefined if called directly on an instance of the class.
  """

  class Notification(Enum):
    """Enumerates possible notifications that might come back from ROS;
    overloads parent definition
    """

  __proxy_methods__ = dict()
  __setup_ros_calls__ = []
  __cleanup_ros_calls__ = []

  @classmethod
  def on_ros(cls, func):
    """Function decorator that registers a local function such that Object.$name(args) in the main process calls the
    decorated function inside the ROS process

    :param name: name of the function that must be called in the main process
    """

    if func.__name__ == "setup_ros":
      cls.__setup_ros_calls__.append(func)
    elif func.__name__ == "cleanup_ros":
      cls.__cleanup_ros_calls__.append(func)
    else:
      cls.__proxy_methods__[func.__name__] = func

    def decorated_func(self, *args, **kwargs):
      print("Main process is calling", func.__name__)
      self._ros_worker_call.put((func.__name__, args, kwargs))
      return self._ros_worker_return.get()

    return decorated_func

  def __init__(self):
    # Inter-process communication happens through these queues
    self._notify = Queue()
    self._ros_worker_call = Queue()
    self._ros_worker_return = Queue()
    self._process = Process(target=lambda: RosWorker(
        self._notify, self.Notification, self._ros_worker_call, self.
        _ros_worker_return, self.__setup_ros_calls__, self.
        __cleanup_ros_calls__, self.__proxy_methods__))

    # Thread to read the notification queue in a loop
    self._lock = RLock()
    self._callbacks = {k: dict() for k in self.Notification}
    self._listener = Thread(target=self._listen)
    self._listener.daemon = True

  def start(self):
    self._process.start()
    self._listener.start()
    self._after_start_hook()

  def shutdown(self):
    self._ros_worker_call.put(("shutdown", (), {}))
    self._process.join()
    self._process.terminate()

  def _after_start_hook(self):
    pass

  def _listen(self):
    """Listens for incoming ROS commands from the main process"""
    while True:
      func, args, kwargs = self._notify.get()
      with self._lock:
        print("Main process is notifying", func)
        [f(*args, **kwargs) for f in self._callbacks.get(func, {}).values()]
      self._after_listen_hook(func, args, kwargs)

  def _after_listen_hook(self, func, args, kwargs):
    pass
