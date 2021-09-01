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

import uuid
import time
import math
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from collections import OrderedDict

from vtr_mission_planning.ros_manager import RosManager
from builtin_interfaces.msg import Duration
from unique_identifier_msgs.msg import UUID
from vtr_messages.action import Mission
from vtr_messages.srv import MissionPause
from vtr_messages.msg import MissionStatus


def msg2dict(msg):
  """Convert a generic ROS message into a dictionary (un-nested)
  :param msg: ROS message to convert
  :rtype: dict
  """
  return {k: getattr(msg, k) for k in msg.get_fields_and_field_types().keys()}


def goalinfo2dict(goalinfo):
  """Converts a goal into a picklable dictionary
  :param goal_handle: Goal handle to convert
  :rtype: dict
  """
  goal = goalinfo['info']
  tmp = dict()
  if goal.target == Mission.Goal.IDLE:
    tmp['target'] = 'Idle'
  elif goal.target == Mission.Goal.TEACH:
    tmp['target'] = 'Teach'
  elif goal.target == Mission.Goal.REPEAT:
    tmp['target'] = 'Repeat'
  elif goal.target == Mission.Goal.MERGE:
    tmp['target'] = 'Merge'
  elif goal.target == Mission.Goal.LOCALIZE:
    tmp['target'] = 'Localize'
  else:
    raise RuntimeError("Got a malformed goal type of %d" % (goal.target,))
  tmp['path'] = list(goal.path)
  tmp['vertex'] = goal.vertex
  tmp['pauseBefore'] = goal.pause_before.sec + goal.pause_before.nanosec / 1e9
  tmp['pauseAfter'] = goal.pause_after.sec + goal.pause_after.nanosec / 1e9

  tmp['id'] = goalinfo['id']
  tmp['inProgress'] = goalinfo['in_progress']
  tmp['waiting'] = goalinfo['waiting']
  tmp['percentComplete'] = goalinfo['percent_complete']

  return tmp


class MissionClient(RosManager):
  """Client used to interface with the C++ MissionServer node.
  """

  class Notification(Enum):
    """Enumerates possible notifications that might come back from ROS;
    overloads parent definition
    """
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

  def __init__(self):
    super().__init__()

  @RosManager.on_ros
  def setup_ros(self, *args, **kwargs):
    """Sets up necessary ROS communications"""
    # Mission action server
    self._action = ActionClient(self, Mission, "manager")
    self._action.wait_for_server()
    self._goals = OrderedDict()
    self._goalinfos = OrderedDict()  # contains goal infos (sync with above)
    self._feedback = {}
    #
    self._pause = self.create_client(MissionPause, 'pause')
    self._pause.wait_for_service()
    #
    self._status_sub = self.create_subscription(MissionStatus, 'status',
                                                self.status_callback, 1)
    self._queue = []
    self._status = MissionStatus.PAUSED

  @RosManager.on_ros
  def cleanup_ros(self, *args, **kwargs):
    """Sets up necessary ROS communications"""
    self._action.destroy()
    self._pause.destroy()

  @RosManager.on_ros
  def status_callback(self, msg):
    if self._queue != msg.mission_queue or self._status != msg.status:
      self._queue = msg.mission_queue
      self._status = msg.status

      status = msg.status
      mission_queue = [
          str(uuid.UUID(bytes=m.uuid.tobytes())) for m in msg.mission_queue
      ]
      self.notify(self.Notification.StatusChange, status, mission_queue)

  @RosManager.on_ros
  def set_pause(self, pause=True):
    """Sets the pause state of the mission server

    :param pause: whether or not to pause the server
    """
    req = MissionPause.Request()
    req.pause = pause
    # This call has to be async because otherwise the process can hang due to
    # other callback in ros internal queue and cannot be rearranged, in which
    # case or thread lock in RosWorker causes the hang.
    res_future = self._pause.call_async(req)
    res_future.add_done_callback(self.set_pause_callback)

  @RosManager.on_ros
  def set_pause_callback(self, future):
    # print("Pause result:", str(future.result().response_code))
    pass

  @RosManager.on_ros
  def add_goal(self,
               goal_type=None,
               path=(0, 0),
               pause_before=0,
               pause_after=0,
               vertex=2**64 - 1):
    """Adds a new goal inside the ROS process

    :param goal_type enum representing the type of goal to add
    :param path list of vertices to visit
    :param pause_before duration in seconds to pause before execution
    :param pause_after duration in seconds to pause after execution:
    """

    if goal_type not in [
        Mission.Goal.IDLE,
        Mission.Goal.TEACH,
        Mission.Goal.REPEAT,
        Mission.Goal.MERGE,
        Mission.Goal.LOCALIZE,
        Mission.Goal.OTHER,
    ]:
      raise RuntimeError("Goal of type %d not in range [0,5]" % (goal_type,))

    goal = Mission.Goal()
    goal.target = int(goal_type)
    goal.path = path
    goal.vertex = vertex
    goal.pause_before = Duration()
    goal.pause_before.sec = math.floor(pause_before)
    goal.pause_before.nanosec = math.floor(
        (pause_before - math.floor(pause_before)) * 1e9)
    goal.pause_after = Duration()
    goal.pause_after.sec = math.floor(pause_after)
    goal.pause_after.nanosec = math.floor(
        (pause_after - math.floor(pause_after)) * 1e9)

    goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
    goal_uuid_str = str(uuid.UUID(bytes=goal_uuid.uuid.tobytes()))
    #
    self._goalinfos[goal_uuid_str] = {
        'id': goal_uuid_str,
        'in_progress': False,
        'waiting': False,
        'percent_complete': 0,
        'info': goal,
    }
    self.get_logger().info(
        "Add a goal with id <{}> of type {}, path {}, pause before {}, pause after {}, vertex {}."
        .format(goal_uuid_str, goal_type, path, pause_before, pause_after,
                vertex))
    # send the goal (async)
    send_goal_future = self._action.send_goal_async(
        goal, feedback_callback=self.feedback_callback, goal_uuid=goal_uuid)
    send_goal_future.add_done_callback(self.response_callback)

    return goal_uuid_str

  @RosManager.on_ros
  def cancel_all(self):
    """Cancels a goal inside the ROS process

    :param goal_id: goal id to be cancelled
    """
    # This is safe, because the callbacks that modify _queue block until this function returns and _lock is released
    for k, v in reversed(self._goals.items()):
      print(k, v)
      self.cancel_goal(k)
    return True

  @RosManager.on_ros
  def cancel_goal(self, goal_uuid):
    """Cancels a goal inside the ROS process

    :param goal_id: goal id to be cancelled
    """
    # Check to make sure we are still tracking this goal
    if goal_uuid not in self._goals.keys():
      self.get_logger().info(
          "No tracking goal with id {}, so not canceled.".format(goal_uuid))
      return False

    # Cancel the goal
    self.get_logger().info("Cancel goal with id <{}>".format(goal_uuid))
    self._goals[goal_uuid].cancel_goal_async()
    return True

  @RosManager.on_ros
  def feedback_callback(self, feedback_handle):
    feedback = feedback_handle.feedback
    goal_uuid = str(uuid.UUID(bytes=feedback_handle.goal_id.uuid.tobytes()))

    if not goal_uuid in self._goals.keys():
      return

    old = (self._feedback[goal_uuid]
           if goal_uuid in self._feedback.keys() else None)
    # update feedback and sync with goalinfos
    self._feedback[goal_uuid] = feedback
    self._goalinfos[goal_uuid].update(msg2dict(feedback))
    if not old:
      if feedback.in_progress == True:  # a goal may start immediately on first feedback
        self.notify(self.Notification.Started, goal_uuid)
      self.notify(self.Notification.Feedback, goal_uuid, msg2dict(feedback))
      self.get_logger().info(
          "Goal with id <{}> gets first feedback: in_progress {}, waiting {}, and percent complete {}"
          .format(goal_uuid, feedback.in_progress, feedback.waiting,
                  feedback.percent_complete))
    else:
      if old.in_progress != feedback.in_progress:  # delayed start of a goal
        self.notify(self.Notification.Started, goal_uuid)

      if (old.percent_complete != feedback.percent_complete or
          old.waiting != feedback.waiting):
        self.notify(self.Notification.Feedback, goal_uuid, msg2dict(feedback))

      self.get_logger().info(
          "Goal with id <{}> gets updated feedback: in_progress {}, waiting {}, and percent complete {}"
          .format(goal_uuid, feedback.in_progress, feedback.waiting,
                  feedback.percent_complete))

  @RosManager.on_ros
  def response_callback(self, future):
    goal_handle = future.result()
    goal_uuid = str(uuid.UUID(bytes=goal_handle.goal_id.uuid.tobytes()))
    if goal_handle.accepted:
      self.get_logger().info(
          'Goal with id <{}> has been accepted.'.format(goal_uuid))
      self._goals[goal_uuid] = goal_handle
      get_result_future = goal_handle.get_result_async()
      get_result_future.add_done_callback(
          lambda future, uuid=goal_uuid: self.get_result_callback(uuid, future))
      # TODO yuchen is it OK to notify here?
      self.notify(self.Notification.NewGoal,
                  goalinfo2dict(self._goalinfos[goal_uuid]))
    else:
      self.get_logger().info(
          'Goal with id <{}> has been rejected.'.format(goal_uuid))
      del self._goalinfos[goal_uuid]

  @RosManager.on_ros
  def get_result_callback(self, uuid, future):
    result = future.result().result
    status = future.result().status

    if not uuid in self._goals.keys():
      return

    # Goal has completed successfully
    if status == GoalStatus.STATUS_SUCCEEDED:
      self.get_logger().info(
          'Goal with id <{}> succeeded with result: {}'.format(uuid, result))
      self.notify(self.Notification.Complete, uuid)
    # Goal has failed for an internal reason
    elif status == GoalStatus.STATUS_ABORTED:
      self.get_logger().info("Goal with id <{}> aborted.".format(uuid))
      self.notify(self.Notification.Error, uuid, status)
    # Goal was cancelled by the user
    elif status == GoalStatus.STATUS_CANCELED:
      self.get_logger().info("Goal with id <{}> cancelled.".format(uuid))
      self.notify(self.Notification.Cancel, uuid)
    # Goal has been started
    else:
      # GoalStatus.STATUS_EXECUTING -> will we ever enter this state?
      self.get_logger().info("Goal with id <{}> in unknown state.".format(uuid))
      self.notify(self.Notification.Started, uuid)

    # If the goal is in a terminal state, remove it from being tracked
    if status in [
        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_CANCELED
    ]:
      self.get_logger().info(
          "Goal with id <{}> deleted from queue.".format(uuid))
      del self._goals[uuid]
      del self._goalinfos[uuid]
      if uuid in self._feedback.keys():
        del self._feedback[uuid]

  @property
  @RosManager.on_ros
  def status(self):
    """Get the current mission server status"""
    return {
        MissionStatus.PROCESSING: "PROCESSING",
        MissionStatus.PAUSED: "PAUSED",
        MissionStatus.PENDING_PAUSE: "PENDING_PAUSE",
        MissionStatus.EMPTY: "EMPTY"
    }.get(self._status, "")

  @property
  @RosManager.on_ros
  def goals(self):
    """Get the current mission server status"""
    goals = []
    for uuid in self._goals.keys():
      goals.append(goalinfo2dict(self._goalinfos[uuid]))
    return goals


if __name__ == "__main__":
  mc = MissionClient()
  mc.start()

  print("\nStart")
  mc.set_pause(False)

  print("\nAdd an Idle goal, cancel after succeeded")
  uuid = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(2)
  time.sleep(1)
  mc.cancel_goal(uuid)  # no goal to cancel as it has succeeded
  print("\nAdd an Idle goal, cancel before it is accepted")
  uuid = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  mc.cancel_goal(uuid)  # no goal to cancel as it has not been accepted
  time.sleep(2)
  time.sleep(1)
  print("\nAdd an Idle goal, cancel before it is executed")
  uuid = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(0.5)
  mc.cancel_goal(uuid)  # no goal to cancel as it has not been accepted
  time.sleep(1.5)
  time.sleep(1)
  print("\nAdd an Idle goal, cancel after it is executed but before finished")
  uuid = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(1.5)
  mc.cancel_goal(uuid)  # no goal to cancel as it has not been accepted
  time.sleep(0.5)
  time.sleep(1)

  print("\nAdd two Idle goal, cancel after succeeded")
  uuid0 = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(0.5)
  uuid1 = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(4)
  time.sleep(1)
  mc.cancel_all()  # no goal to cancel as both have succeeded
  print("\nAdd two Idle goal cancel the first before it is finished.")
  uuid0 = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(0.5)
  uuid1 = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(0.5)
  mc.cancel_goal(uuid0)  # first goal canceled
  time.sleep(3.5)
  time.sleep(1)
  print("\nAdd two Idle goal cancel the first after it is finished")
  uuid0 = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(0.5)
  uuid1 = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(1.5)
  mc.cancel_goal(uuid0)  # no goal to cancel as it is finished
  time.sleep(1.5)
  time.sleep(1)

  print("\nStop")
  mc.set_pause(True)
  mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(1)
  mc.cancel_all()

  # Shut down
  time.sleep(1)
  mc.shutdown()
