#!/usr/bin/env python

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
      self.notify(self.Notification.StatusChange, self._status, self._queue)

  @RosManager.on_ros
  def set_pause(self, pause=True):
    """Sets the pause state of the mission server

    :param paused: whether or not to pause the server
    """
    req = MissionPause.Request()
    req.pause = pause
    # This call has to be async because otherwise the process can hang due to
    # other callback in ros internal queue and cannot be rearanged, in which
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
               path=(),
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
    goal_uuid_str = goal_uuid.uuid.tostring()

    self.get_logger().info(
        "Add a goal with id <{}> of type {}, path {}, pause before {}, pause after {}, vertex {}."
        .format(goal_uuid_str, goal_type, path, pause_before, pause_after,
                vertex))

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

  @RosManager.on_ros
  def cancel_goal(self, uuid):
    """Cancels a goal inside the ROS process

    :param goal_id: goal id to be cancelled
    """
    # Check to make sure we are still tracking this goal
    if uuid not in self._goals.keys():
      self.get_logger().info(
          "No tracking goal with id {}, so not canceled.".format(uuid))
      return False

    # Cancel the goal
    self.get_logger().info("Cancel goal with id <{}>".format(uuid))
    self._goals[uuid].cancel_goal_async()
    return True

  @RosManager.on_ros
  def feedback_callback(self, feedback_handle):
    feedback = feedback_handle.feedback
    uuid = feedback_handle.goal_id.uuid.tostring()

    if not uuid in self._goals.keys():
      return

    if not uuid in self._feedback.keys():
      self._feedback[uuid] = feedback
      self.notify(self.Notification.Feedback, uuid, feedback)
      self.get_logger().info(
          "Goal with id <{}> gets first feedback saying percent complete {} and waiting {}"
          .format(uuid, feedback.percent_complete, feedback.waiting))
    else:
      old = self._feedback[uuid]
      self._feedback[uuid] = feedback

      if old.percent_complete != feedback.percent_complete or old.waiting != feedback.waiting:
        self.notify(self.Notification.Feedback, uuid, feedback)
      self.get_logger().info(
          "Goal with id <{}> gets updated feedback saying percent complete {} and waiting {}"
          .format(uuid, feedback.percent_complete, feedback.waiting))

  @RosManager.on_ros
  def response_callback(self, future):
    goal_handle = future.result()
    uuid = goal_handle.goal_id.uuid.tostring()
    if not goal_handle.accepted:
      self.get_logger().info(
          'Goal with id <{}> has been rejected.'.format(uuid))
      return
    self.get_logger().info('Goal with id <{}> has been accepted.'.format(uuid))
    self._goals[uuid] = goal_handle
    get_result_future = goal_handle.get_result_async()
    get_result_future.add_done_callback(
        lambda future, uuid=uuid: self.get_result_callback(uuid, future))

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
      if uuid in self._feedback.keys():
        del self._feedback[uuid]


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
