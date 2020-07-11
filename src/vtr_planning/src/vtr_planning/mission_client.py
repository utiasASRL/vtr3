# Defines BaseMissionClient and Notification.  These are used to interface with the C++ MissionServer node.

import rospy
from actionlib.action_client import ActionClient, CommState, GoalStatus
from vtr_planning.msg import MissionAction, MissionGoal as Goal, MissionStatus, MissionFeedback
from vtr_planning.srv import GoalReorder, GoalReorderResponse, MissionPause, MissionPauseResponse

from .ros_manager import BaseRosManager, ros_locked, local_locked
from enum import Enum


def handle_id(goal_handle):
  """Returns the ID associated with a goal handle
    :param goal_handle: Goal handle to extract the ID from
    :rtype: str
    """
  return goal_handle.comm_state_machine.action_goal.goal_id.id


def handle_goal(goal_handle):
  """Returns the goal object associated with a goal handle
    :param goal_handle: Goal handle to extract the goal object from
    :rtype: Goal
    """
  return goal_handle.comm_state_machine.action_goal.goal


def handle_dict(goal_handle):
  """Converts a goal handle into a picklable dictionary
    :param goal_handle: Goal handle to convert
    :rtype: dict
    """
  goal = handle_goal(goal_handle)

  tmp = {'id': handle_id(goal_handle)}

  if goal.target == Goal.IDLE:
    tmp['target'] = 'Idle'
  elif goal.target == Goal.TEACH:
    tmp['target'] = 'Teach'
  elif goal.target == Goal.REPEAT:
    tmp['target'] = 'Repeat'
  elif goal.target == Goal.MERGE:
    tmp['target'] = 'Merge'
  elif goal.target == Goal.LOCALIZE:
    tmp['target'] = 'Localize'
  else:
    raise RuntimeError("Got a malformed goal type of %d" % (goal.target,))

  tmp['path'] = goal.path
  tmp['vertex'] = goal.vertex
  tmp['pauseBefore'] = goal.pauseBefore.to_sec()
  tmp['pauseAfter'] = goal.pauseAfter.to_sec()

  tmp['inProgress'] = goal_handle.get_comm_state() == CommState.ACTIVE

  return tmp


def msg_dict(msg):
  """Convert a generic ROS message into a dictionary
    :param msg: ROS message to convert
    :rtype: dict
    """
  return {k: getattr(msg, k) for k in msg.__slots__}


class Notification(Enum):
  """Enumerates possible notifications that might come back from ROS; overloads parent definition"""
  Feedback = 0
  Complete = 1
  Cancel = 2
  Error = 3
  Started = 4
  NewGoal = 5
  StatusChange = 6
  RobotChange = 7
  PathChange = 8
  GraphChange = 9
  SafetyStatus = 10
  OverlayRefresh = 11


class BaseMissionClient(BaseRosManager):
  """Runs the ROS communications of a MissionClient in another process and proxies commands/results.  This class
    represents a persistent client, and must be shut down manually"""

  # For some reason, defining the Enum in here doesn't work...
  Notification = Notification

  def __init__(self, group=None, name=None, args=(), kwargs={}):
    BaseRosManager.__init__(self,
                            group=group,
                            name=name,
                            args=args,
                            kwargs=kwargs)

    self._action = None
    self._reorder = None
    self._status_sub = None
    self._pause = None

    self._queue = []
    self._goals = {}
    self._feedback = {}
    self._status = MissionStatus.PAUSED

  def wait_for_status(self, status):
    """Waits for the MissionServer to enter a specific state
        :param status: status enum to wait for
        """
    rospy.loginfo("Waiting for status %s", str(status))
    self.wait_for_condition(self.Notification.StatusChange,
                            lambda *a, **k: a[0] == status,
                            lambda s: s.status == status)

  def _ros_setup(self, *args, **kwargs):
    """Sets up necessary ROS communications"""
    self._action = ActionClient(args[0] + "/manager", MissionAction)
    self._reorder = rospy.ServiceProxy(args[0] + '/reorder', GoalReorder)
    self._pause = rospy.ServiceProxy(args[0] + '/pause', MissionPause)
    self._status_sub = rospy.Subscriber(args[0] + '/status',
                                        MissionStatus,
                                        self._status_cb,
                                        queue_size=1)

    rospy.loginfo("Connecting to server...")
    result = self._action.wait_for_action_server_to_start(rospy.Duration(30))
    if not result:
      rospy.logfatal("Could not connect to server!")
    else:
      rospy.loginfo("Connected")

  @BaseRosManager.proxy_func('add_goal')
  def _add_goal(self,
                goal_type,
                path=None,
                pause_before=0,
                pause_after=0,
                vertex=2**64 - 1):
    """Adds a new goal inside the ROS process
        :param goal_type: enum representing the type of goal to add
        :param path: list of vertices to visit
        :param pause_before: duration in seconds to pause before execution
        :param pause_after: duration in seconds to pause after execution:
        """

    rospy.logdebug("Got new goal: " + str(goal_type))

    if goal_type not in [
        Goal.IDLE, Goal.TEACH, Goal.REPEAT, Goal.MERGE, Goal.LOCALIZE
    ]:
      rospy.logfatal("Goal of type %d not in range [0,4]" % (goal_type,))
      raise RuntimeError("Goal of type %d not in range [0,4]" % (goal_type,))

    g = Goal(target=goal_type,
             path=path,
             vertex=vertex,
             pauseBefore=rospy.Duration(pause_before),
             pauseAfter=rospy.Duration(pause_after))

    gh = self._action.send_goal(g, self._transition_cb, self._feedback_cb)

    self._goals[handle_id(gh)] = gh
    self._queue.append(handle_id(gh))

    self._notify(Notification.NewGoal, handle_dict(gh))
    return handle_id(gh)

  @BaseRosManager.proxy_func('cancel_goal')
  def _cancel_goal(self, goal_id):
    """Cancels a goal inside the ROS process
        :param goal_id: goal id to be cancelled
        """
    # Check to make sure we are still tracking this goal
    if goal_id not in self._goals.keys():
      return False

    rospy.loginfo("Cancelling goal %s" % goal_id)

    self._goals[goal_id].cancel()
    return True

  @BaseRosManager.proxy_func('cancel_all')
  def _cancel_all(self):
    """Cancels all goals inside the ROS process"""

    # This is safe, because the callbacks that modify _queue block until this function returns and _lock is released
    for x in reversed(self._queue):
      self._goals[x].cancel()

    return True

  @BaseRosManager.proxy_func('move_goal')
  def _move_goal(self, goal_id, idx=-1, before=""):
    """Moves a goal inside the ROS process
        :param goal_id: id of the goal to move
        :param idx: index in the queue to move it to
        :param before: id of the goal that should come immediately after this goal
        """
    # Check to make sure we are still tracking this goal
    if goal_id not in self._goals.keys():
      return False

    if before != "" and before not in self._goals.keys():
      return False

    try:
      res = self._reorder(goalId=goal_id, toIdx=idx, beforeId=before)
    except rospy.ServiceException as exc:
      rospy.logerr("Could not reorder goals: %s", str(exc))
      return False

    if res.returnCode != GoalReorderResponse.SUCCESS:
      return False

    return True

  @BaseRosManager.proxy_func('set_pause')
  def _set_pause(self, paused=True):
    """Sets the pause state of the mission server
        :param paused: whether or not to pause the server
        """
    res = self._pause(paused)
    rospy.loginfo("Pause result: %s", str(res.responseCode))
    return res.responseCode

  @BaseRosManager.proxy_var('goals')
  def _get_goals(self, item=None):
    """Gets all goals in order
        :param item: get
        """
    tmp = [handle_dict(self._goals[x]) for x in self._queue]

    if item is None:
      return tmp
    else:
      return tmp[item]

  @BaseRosManager.proxy_var('feedback')
  def _get_feedback(self, item=None):
    """Gets all goals in order"""
    tmp = [msg_dict(self._feedback[x]) for x in self._queue]

    if item is None:
      return tmp
    else:
      return tmp[item]

  @BaseRosManager.proxy_var('status')
  def _get_status(self):
    """Get the current mission server status"""
    return self._status

  @ros_locked
  def _status_cb(self, msg):
    """Updates local state and proxies status messages back to the main process
        :param msg: mission server status message
        """
    if self._queue != msg.missionQueue or self._status != msg.status:
      self._queue = msg.missionQueue
      self._status = msg.status
      self._notify(self.Notification.StatusChange, self._status, self._queue)

  @ros_locked
  def _feedback_cb(self, goal_handle, feedback):
    """Callback on goal feedback; updates feedback cache and proxies notifications to the main process
        :param goal_handle: handle to the goal receiving feedback
        :param feedback: feedback message
        """
    # Check to make sure we are still tracking this goal
    if handle_id(goal_handle) not in self._goals.keys():
      return

    if handle_id(goal_handle) not in self._feedback.keys():
      self._feedback[handle_id(goal_handle)] = feedback
      self._notify(self.Notification.Feedback, handle_id(goal_handle),
                   msg_dict(feedback))
      rospy.logdebug("First feedback for goal %s ", handle_id(goal_handle))
      rospy.logdebug(feedback)
    else:
      old = self._feedback[handle_id(goal_handle)]
      self._feedback[handle_id(goal_handle)] = feedback
      if old.percentComplete != feedback.percentComplete or old.waiting != feedback.waiting:
        self._notify(self.Notification.Feedback, handle_id(goal_handle),
                     msg_dict(feedback))
        rospy.logdebug("Updated feedback for goal %s ", handle_id(goal_handle))
      rospy.logdebug("Updated feedback for goal2 %s ", handle_id(goal_handle))
      rospy.logdebug(feedback)

  @ros_locked
  def _transition_cb(self, goal_handle):
    """Callback on goal transition; updates local goal state and proxies notifications to the main process
        :param goal_handle: handle to the goal that was updated
        """
    # Check to make sure we are still tracking this goal
    if handle_id(goal_handle) not in self._goals.keys():
      return

    # Goal has completed successfully
    if goal_handle.get_goal_status() == GoalStatus.SUCCEEDED:
      rospy.logdebug("Goal %s succeeded.", handle_id(goal_handle))
      self._notify(self.Notification.Complete, handle_id(goal_handle))
    # Goal has failed for an internal reason
    elif goal_handle.get_goal_status() in [
        GoalStatus.ABORTED, GoalStatus.LOST, GoalStatus.REJECTED
    ]:
      rospy.logdebug("Goal %s aborted.", handle_id(goal_handle))
      self._notify(self.Notification.Error, handle_id(goal_handle),
                   goal_handle.get_goal_status())
    # Goal was cancelled by the user
    elif goal_handle.get_goal_status() in [
        GoalStatus.RECALLED, GoalStatus.PREEMPTED
    ]:
      rospy.logdebug("Goal %s cancelled.", handle_id(goal_handle))
      self._notify(self.Notification.Cancel, handle_id(goal_handle))
    # Goal has been started
    elif goal_handle.get_goal_status() == GoalStatus.ACTIVE:
      rospy.logdebug("Goal %s became active.", handle_id(goal_handle))
      self._notify(self.Notification.Started, handle_id(goal_handle))

    # If the goal is in a terminal state, remove it from being tracked
    if goal_handle.get_goal_status() in [
        GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.LOST,
        GoalStatus.RECALLED, GoalStatus.PREEMPTED, GoalStatus.REJECTED
    ]:
      rospy.logdebug("Goal %s deleted from queue.", handle_id(goal_handle))
      del self._goals[handle_id(goal_handle)]
      del self._feedback[handle_id(goal_handle)]
