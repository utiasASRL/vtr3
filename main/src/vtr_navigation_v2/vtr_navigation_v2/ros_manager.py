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
from multiprocessing import Process, Queue, Event
from threading import Thread

import rclpy
from rclpy.node import Node

hd = logging.StreamHandler()
fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
hd.setFormatter(fm)
worker_logger = logging.getLogger('ros_worker')
worker_logger.setLevel(logging.DEBUG)
worker_logger.addHandler(hd)
manager_logger = logging.getLogger('ros_manager')
manager_logger.setLevel(logging.DEBUG)
manager_logger.addHandler(hd)


class ROSWorker(Node):

  def __init__(self, call_queue, return_queue, notify_queue, setup_calls, methods):
    super().__init__('ros_manager_worker')

    self._call_queue = call_queue
    self._return_queue = return_queue
    self._notify_queue = notify_queue

    for k, v in methods.items():

      def call(*args, _v=v, **kwargs):
        return _v(self, *args, **kwargs)

      setattr(self, k, call)

    for setup_call in setup_calls:
      setup_call(self)

    self._listener = Thread(target=self._listen)
    self._listener.start()

  def stop(self):
    worker_logger.debug("Stopping ROS worker:listener thread.")
    self._call_queue.put(("stop", (), {}))
    self._listener.join()
    worker_logger.debug("Stopping ROS worker:listener thread - done.")

  def _listen(self):
    """Listens for incoming ROS commands from the main process"""
    worker_logger.debug("ROS process listener starts listening.")
    while True:
      func, args, kwargs = self._call_queue.get()
      worker_logger.debug(f"ROS process is calling: {func}")
      if func == "stop":  # special signal for joining the thread
        break
      ret = getattr(self, func)(*args, **kwargs)
      self._return_queue.put(ret)
    worker_logger.debug("ROS process listener stops listening.")

  def notify(self, name, *args, **kwargs):
    """Transmits messages to the main process"""
    worker_logger.debug(f"ROS process is notifying: {name}")
    self._notify_queue.put((name, args, kwargs))


class ROSManager():
  """Manages ROS to non-ROS communications.
  This class spawns it's own ROS node in a separate process, and proxies
  data/commands to and from it to provide some level of isolation. When start()
  is called, two copies of the class will exist: one in your process and one in
  a new process that becomes a ROS node.
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
      manager_logger.debug("Main process is calling %s", func.__name__)
      self._ros_worker_call.put((func.__name__, args, kwargs))
      return self._ros_worker_return.get()

    return decorated_func

  def __init__(self):
    manager_logger.debug("Contructing ROS manager.")

    # Thread to read the notifications in a loop
    self._listener = Thread(target=self._listen)

    # Inter-process communication happens through these queues
    self._ros_shutdown_event = Event()
    self._ros_worker_call = Queue()
    self._ros_worker_return = Queue()
    self._ros_worker_notify = Queue()
    self._ros_process = Process(target=self._ros_process_work)

    # start the listener thread first
    self._listener.start()
    #
    self._ros_process.start()

    manager_logger.debug(
        "Contructing ROS manager. - done")  # NOTE this might be printed before ROS process has fully started

  def shutdown(self):
    manager_logger.debug("Shutting down ROS manager.")

    # shutdown ros process first
    self._ros_shutdown_event.set()
    self._ros_process.join()

    # joins the listener thread
    self._ros_worker_notify.put(('stop', (), {}))
    self._listener.join()

    manager_logger.debug("Shutting down ROS manager. - done")

  def _ros_process_work(self):
    manager_logger.debug("Contructing the ROS process.")
    rclpy.init()
    node = ROSWorker(
        self._ros_worker_call,
        self._ros_worker_return,
        self._ros_worker_notify,
        self.__setup_ros_calls__,
        self.__proxy_methods__,
    )

    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    manager_logger.debug("Contructing the ROS process. - done")

    self._ros_shutdown_event.wait()

    manager_logger.debug("Stopping the ROS process.")
    rclpy.shutdown()  # stop the spin thread
    spin_thread.join()
    node.stop()  # stop ROS worker internal threads
    manager_logger.debug("Stopping the ROS process. - done")

  def _listen(self):
    """Listens for messages from the ROS process"""
    manager_logger.debug("Main process listener starts listening.")
    while True:
      type, args, kwargs = self._ros_worker_notify.get()
      manager_logger.debug("Main process is notifying %s", type)
      if type == "stop":
        break
      self._after_listen_hook(type, args, kwargs)
    manager_logger.debug("Main process listener stops listening.")

  def _after_listen_hook(self, type, args, kwargs):
    pass


if __name__ == "__main__":

  import time

  logger = logging.getLogger('default')
  logger.setLevel(logging.DEBUG)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("INITIALIZE ROS MANAGER")
  ros_manager = ROSManager()  # ROS process start is non-blocking
  time.sleep(1)
  logger.info("INITIALIZE ROS MANAGER - DONE")

  time.sleep(2)

  logger.info("SHUTDOWN ROS MANAGER")
  ros_manager.shutdown()
  logger.info("SHUTDOWN ROS MANAGER - DONE")