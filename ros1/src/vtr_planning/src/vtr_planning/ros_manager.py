"""Defines a class that isolates ROS in a separate process.
This file contains a lot of really finicky multiprocessing stuff, and the
order in which things are spawned is very important.
Don't touch this unless you're prepared to deal with concurrency issues.
"""

import logging
from multiprocessing import Process, Queue
from threading import Thread, RLock, Event

import rospy


class IdGen(object):
  """Generates ids that are guaranteed to be sequential and unique"""
  _lock = RLock()
  _seq = -1

  @classmethod
  def get(cls):
    """Get the next ID"""
    with cls._lock:
      cls._seq += 1
      return cls._seq


def ros_locked(func):
  """Decorator to acquire a lock in the ROS process before executing a class
  function

  :param func: the decorated function
  """

  def locked_func(*args, **kwargs):
    with args[0]._lock:
      return func(*args, **kwargs)

  return locked_func


def local_locked(func):
  """Decorator to acquire a lock in the main process before executing a class
  function

  :param func: the decorated function
  """

  def locked_func(*args, **kwargs):
    with args[0].lock:
      return func(*args, **kwargs)

  return locked_func


class BaseRosManager(Process):
  """Manages ROS to non-ROS communications.
  This class spawns it's own ROS node in a separate process, and proxies
  data/commands to and from it to provide some level of isolation. When start()
  is called, two copies of the class will exist: one in your process and one in
  a new process that becomes a ROS node.
  In general, things that begin with an _ are not meant to be called directly;
  their behaviour is undefined if called directly on an instance of the class.
  """

  Notification = None

  def __init__(self, group=None, name=None, args=(), kwargs={}):
    Process.__init__(self,
                     group=group,
                     target=self._ros_loop,
                     name=name,
                     args=args,
                     kwargs=kwargs)

    # Initialize callbacks to do nothing
    self._callbacks = {k: dict() for k in self.Notification}
    self._proxies = dict()

    # Inter-process communication happens through these queues
    self.to_ros = Queue()
    self.from_ros = Queue()
    self.notify = Queue()

    # Kill all children, just in case (Y)
    self.daemon = True

    # NOTE: these are THREADING rlocks: they are not synced across processes.
    # We could use only one, but relying on the same member variable name
    # referring to different things in different processes is bad.
    self.lock = RLock()  # lock for the main process
    self._lock = RLock()  # lock for the ros process

    # Thread to read the notification queue in a loop
    self._notify_thread = Thread(target=self._notify_loop)
    self._notify_thread.daemon = True

  def start(self):
    """Overridden start method to ensure that the notification listener thread 
    is started *after* the ROS process. This is necessary, as you cannot call
    os.fork on a threaded process.
    """
    Process.start(self)
    self._notify_thread.start()
    self._post_start()

  def _post_start(self):
    """Method to override in derived classes for post-startup hooks."""
    pass

  def register_callback(self, event, fcn):
    """Register a callback to be executed locally on a notification

    :param event: notification enum representing the type of event the callback will fire on
    :param fcn: the callback function (signature is dependent on the event type)
    :returns: a unique, integer ID representing this callback
    """
    with self.lock:
      if event not in self.Notification:
        raise RuntimeError("Value " + str(event) +
                           " is not raised by this class")

      idx = IdGen.get()
      self._callbacks[event][idx] = fcn
      return idx

  def remove_callback(self, event, idx):
    """Remove a previously registered callback

    :param event: notification enum representing the type of event to remove a callback from
    :param idx: the unique ID representing the callback (generated at registration)
    """
    with self.lock:
      if event not in self.Notification:
        raise RuntimeError("Value " + str(event) +
                           " is not raised by this class")

      if idx not in self._callbacks[event].keys():
        raise RuntimeError("No callback with id %d for event of type " % idx +
                           str(event))

      del self._callbacks[event][idx]

  def wait_for_condition(self,
                         event,
                         condition=lambda *a, **k: True,
                         precondition=lambda s: False):
    """Exits immediately if $precondition(self) is met, otherwise blocks until a 
    notification satisfying $condition(*args, **kwargs) is raised.

    :param event: notification enum representing the type of event to check the 
                  condition on.
    :param condition: a function that accepts the appropriate arguments for the 
                      event type, and returns True when the condition being 
                      checked for is met.
    :param precondition: a function that accepts a BaseRosManager instance and 
                         returns True if the manager is already in the desired 
                         state.
    """
    with self.lock:
      # Return immediately if we already have this status
      if precondition(self):
        return

      idx, e = self._build_event(event, condition)

    rospy.logdebug("Waiting for condition...")
    self._wait_for(event, idx, e)

  def _build_event(self, event, cond=lambda *a, **k: True):
    """Builds a threading Event that is set when a notification of type $event
    satisfies $cond.

    :param event: notification enum representing the type of event to check the condition on
    :param cond:  a function that accepts the appropriate arguments for the event type, and returns True when
                  the condition being checked for is met
    """
    with self.lock:
      e = Event()

      def f(name, *args, **kwargs):
        if cond(*args, **kwargs):
          e.set()

      idx = self.register_callback(event, f)

    return idx, e

  def _wait_for(self, event, idx, e):
    """Waits for a previously built event, then removes the associated callback

    :param event: notification enum representing the type of event being waited on
    :param idx: the unique ID of the callback that will set the event
    :param e: a threading.Event object to wait for
    """
    e.wait()
    self.remove_callback(event, idx)

  def _ros_setup(self, *args, **kwargs):
    """Override this in a subclass to set up the necessary ROS
    services/subscribers.
    """
    pass

  def _ros_loop(self, ns, *args, **kwargs):
    """Main execution entry point for the new ROS process

    :param ns: name of the node to spawn
    """
    rospy.init_node(ns, disable_signals=True, **kwargs)
    self._ros_setup(*args, **kwargs)

    listener = Thread(target=self._ros_proxy_listener)
    listener.daemon = True
    listener.start()

    rospy.spin()

  def _shutdown(self):
    """Shuts down ROS, causing the main process and all children to terminate"""
    rospy.loginfo("Shutting down master client ROS process...")
    rospy.signal_shutdown("Shutdown command received")

  def shutdown(self):
    """Terminates the ROS process"""
    # Lock to ensure that we don't add more commands while we are shutting down
    with self.lock:
      self.to_ros.put(('shutdown', (), {}))

      try:
        self.join(1)
      except Exception:
        # IDK... sometimes ROS just doesn't stop properly
        logging.warning("ROS process is hanging; escalating to SIGTERM...")
        self.terminate()

  # Dictionaries that map main-process function names to backend cass funtions
  __proxy_methods = {'shutdown': _shutdown}
  __proxy_vars = {}

  def _ros_proxy_listener(self):
    """Listens for incoming ROS commands from the main process"""
    while True:
      cmd, args, kwargs = self.to_ros.get()
      with self._lock:
        if cmd in self.__proxy_methods.keys():
          self.from_ros.put(self.__proxy_methods[cmd](self, *args, **kwargs))
        elif cmd in self.__proxy_vars.keys():
          self.from_ros.put(self.__proxy_vars[cmd](self, *args, **kwargs))

  def _callmethod(self, name, *args, **kwargs):
    """Call a proxied method directly.  This is a separate method because lambda functions cannot be sent across
        process boundaries, and we need to return results to remote slave clients.

        :param name: name of the method being called
        """
    if name in self.__proxy_methods.keys():
      # Lock here to make sure that commands/variables come back in order
      with self.lock:
        rospy.logdebug("Proxying %s: %s, %s", name, str(args), str(kwargs))
        self.to_ros.put((name, args, kwargs))
        return self.from_ros.get()

  @classmethod
  def proxy_func(cls, name):
    """Function decorator that registers a local function such that Object.$name(args) in the main process calls the
        decorated function inside the ROS process

        :param name: name of the function that must be called in the main process
        """

    def pfunc(self, *args, **kwargs):
      return self._callmethod(name, *args, **kwargs)

    setattr(cls, name, pfunc)

    def proxy_decorator(func):
      cls.__proxy_methods[name] = func
      return func

    return proxy_decorator

  @classmethod
  def proxy_var(cls, name):
    """Function decorator that registers a local attribute such that the value 
    of Object.$name in the main process is the result of the decorated function, 
    called in the ROS process.

    :param name: name of the member variable in the main process
    """

    def proxy_decorator(func):
      cls.__proxy_vars[name] = func
      return func

    return proxy_decorator

  def __getattr__(self, name):
    """Python black magic to automatically convert member lookup into 
    inter-process communication.

    :param name: name of the member being accessed
    """
    if name in self.__proxy_vars.keys():
      # Lock here to make sure that commands/variables come back in order
      with self.lock:
        self.to_ros.put((name, (), {}))
        return self.from_ros.get()
    else:
      raise AttributeError("No such member of class BaseMissionClient: %s" %
                           name)

  def _notify_loop(self):
    """Listens to the notification queue and invokes callbacks in the main 
    process.
    """
    while True:
      kind, args, kwargs = self.notify.get()

      # Use a lock here so that we can't modify the callbacks while we are calling them
      with self.lock:
        _ = [f(*args, **kwargs) for f in self._callbacks.get(kind, {}).values()]

  def _notify(self, name, *args, **kwargs):
    """Proxy notifications to the main process using a queue.

    :param name: name of the notification event
    """
    # Don't need to lock here since all the notifications are asynchronous anyways

    assert kwargs or args
    self.notify.put((name, args, kwargs))
