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
"""Defines a class that isolates ROS in a separate process."""

import logging
from enum import Enum
from multiprocessing import Process, Queue, Condition
from threading import Thread

import rclpy
from rclpy.node import Node

logger = logging.getLogger('MissionClient')


class RosWorker(Node):

  def __init__(self, call_queue, return_queue, notify_queue, notification,
               setup_calls, methods):
    logger.debug("Contructing ROS worker.")

    super().__init__('mission_client')

    self._call_queue = call_queue
    self._return_queue = return_queue
    self._notify_queue = notify_queue
    self.Notification = notification

    for k, v in methods.items():

      def call(*args, _v=v, **kwargs):
        return _v(self, *args, **kwargs)

      setattr(self, k, call)

    for setup_call in setup_calls:
      setup_call(self)

    self._listener = Thread(target=self._listen)
    self._listener.daemon = True
    self._listener.start()

    logger.debug("Contructing ROS worker - done.")

  def _listen(self):
    """Listens for incoming ROS commands from the main process"""
    logger.debug("ROS process listener starts listening.")
    while True:
      func, args, kwargs = self._call_queue.get()
      logger.debug("ROS process is calling: %s", func)
      ret = getattr(self, func)(*args, **kwargs)
      self._return_queue.put(ret)

  def notify(self, name, *args, **kwargs):
    logger.debug("ROS process is notifying: %s", name)
    self._notify_queue.put((name, args, kwargs))


class ROSManager():
  """Manages ROS to non-ROS communications.
  This class spawns it's own ROS node in a separate process, and proxies
  data/commands to and from it to provide some level of isolation. When start()
  is called, two copies of the class will exist: one in your process and one in
  a new process that becomes a ROS node.
  """

  class Notification(Enum):
    """Enumerates possible notifications that might come back from ROS;
    overloads parent definition
    """

  __setup_ros_calls__ = []
  __proxy_methods__ = dict()

  @classmethod
  def on_ros(cls, func):
    """Function decorator that registers a local function such that
    Object.$name(args) in the main process calls the decorated function inside
    the ROS process.
    Args:
      func (func): function that must be called in the ROS process
    """

    if func.__name__ == "setup_ros":
      cls.__setup_ros_calls__.append(func)
    else:
      cls.__proxy_methods__[func.__name__] = func

    def decorated_func(self, *args, **kwargs):
      logger.debug("Main process is calling %s", func.__name__)
      self._ros_worker_call.put((func.__name__, args, kwargs))
      return self._ros_worker_return.get()

    return decorated_func

  def __init__(self):
    logger.debug("Contructing ROS manager.")

    # Inter-process communication happens through these queues
    self._ros_shutdown_cv = Condition()
    self._ros_worker_call = Queue()
    self._ros_worker_return = Queue()
    self._ros_worker_notify = Queue()
    self._process = Process(target=self._ros_process_work)

    # Thread to read the notification queue in a loop
    self._listener = Thread(target=self._listen)
    self._listener.daemon = True
    self._listener.start()

    logger.debug("Contructing ROS manager - done.")

  def start(self):
    self._process.start()

  def shutdown(self):
    with self._ros_shutdown_cv:
      self._ros_shutdown_cv.notify()
    self._process.join()

  def _ros_process_work(self):
    rclpy.init()
    node = RosWorker(
        self._ros_worker_call,
        self._ros_worker_return,
        self._ros_worker_notify,
        self.Notification,
        self.__setup_ros_calls__,
        self.__proxy_methods__,
    )

    spin_thread = Thread(target=lambda: rclpy.spin(node))
    spin_thread.start()
    with self._ros_shutdown_cv:
      self._ros_shutdown_cv.wait()
    rclpy.shutdown()
    spin_thread.join()

  def _listen(self):
    """Listens for incoming ROS commands from the main process"""
    logger.debug("Main process listener starts listening.")
    while True:
      notification, args, kwargs = self._ros_worker_notify.get()
      logger.debug("Main process is notifying %s", notification)
      self._after_listen_hook(notification, args, kwargs)

  def _after_listen_hook(self, notification, args, kwargs):
    pass


if __name__ == "__main__":

  import time

  logger.setLevel(logging.DEBUG)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("INITIALIZE ROS MANAGER")
  ros_manager = ROSManager()
  logger.info("INITIALIZE ROS MANAGER - DONE")

  time.sleep(2)

  logger.info("START ROS MANAGER")
  ros_manager.start()  # this is a non-blocking call
  logger.info("START ROS MANAGER - DONE")

  time.sleep(2)

  logger.info("SHUTDOWN ROS MANAGER")
  ros_manager.shutdown()
  logger.info("SHUTDOWN ROS MANAGER - DONE")