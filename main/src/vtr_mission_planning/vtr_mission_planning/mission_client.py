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

import logging
import math
import uuid
from enum import Enum
from collections import OrderedDict

from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from unique_identifier_msgs.msg import UUID
from action_msgs.msg import GoalStatus

from vtr_messages.action import Mission
from vtr_messages.srv import MissionPause
from vtr_messages.msg import MissionStatus, RobotStatus, GraphUpdate, GraphPath

from .ros_manager import ROSManager

logger = logging.getLogger('MissionClient')


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


class MissionClient(ROSManager):
  """Client used to interface with the C++ MissionServer node."""

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

  @ROSManager.on_ros
  def setup_ros(self, *args, **kwargs):
    """Sets up necessary ROS communications"""
    # yapf: disable

    # start/pause client for starting and pausing the navigation system
    self._pause = self.create_client(MissionPause, 'pause')
    self._pause.wait_for_service()

    # mission client for teach/repeat goal addition and deletion
    self._goals = OrderedDict()
    self._goalinfos = OrderedDict()  # contains goal infos (sync with above)
    self._feedback = {}
    self._action = ActionClient(self, Mission, "manager")
    self._action.wait_for_server()

    # mission status subscription and caches
    self._queue = []
    self._status = MissionStatus.PAUSED
    self._status_sub = self.create_subscription(MissionStatus, 'status', self.status_callback, 10)

    # robot status subscription and caches
    self._path_seq = 0
    self._trunk_vertex = None
    self._trunk_lng_lat_theta = [0, 0, 0]
    self._t_leaf_trunk = [0, 0, 0]
    self._cov_leaf_trunk = []
    self._target_vertex = None
    self._target_lng_lat_theta = [0, 0, 0]
    self._t_leaf_target = [0, 0, 0]
    self._cov_leaf_target = []
    self._status_sub = self.create_subscription(RobotStatus, 'robot', self.robot_callback, 10)

    # graph updates subscription (no cache needed)
    self._graph_sub = self.create_subscription(GraphUpdate, 'graph_updates', self.graph_callback, 10)

    # following path subscription and cache
    self._path = []
    self._path_sub = self.create_subscription(GraphPath, 'out/following_path', self.path_callback, 10)

    # yapf: enable

  @ROSManager.on_ros
  def status_callback(self, msg):
    if self._queue != msg.mission_queue or self._status != msg.status:

      logger.info(
          "Received status change - status {}, mission queue {}.".format(
              msg.status, msg.mission_queue))

      self._queue = msg.mission_queue
      self._status = msg.status

      status = msg.status
      mission_queue = [
          str(uuid.UUID(bytes=m.uuid.tobytes())) for m in msg.mission_queue
      ]
      self.notify(self.Notification.StatusChange, status, mission_queue)

  @ROSManager.on_ros
  def set_pause(self, pause=True):
    """Sets the pause state of the mission server
    Args:
      pause: whether or not to pause the server
    """
    logger.info("Setting system pause to %s.", pause)

    req = MissionPause.Request()
    req.pause = pause
    # This call has to be async because otherwise the process can hang due to
    # other callback in ros internal queue and cannot be rearranged, in which
    # case or thread lock in RosWorker causes the hang.
    res_future = self._pause.call_async(req)
    res_future.add_done_callback(self.set_pause_callback)

  @ROSManager.on_ros
  def set_pause_callback(self, future):
    rc = future.result().response_code
    logger.info("Setting system pause done - %s.", rc)

  @ROSManager.on_ros
  def add_goal(self,
               goal_type=None,
               path=(0, 0),
               pause_before=0,
               pause_after=0,
               vertex=2**64 - 1):
    """Adds a new goal inside the ROS process
    Args:
      goal_type: enum representing the type of goal to add
      path: list of vertices to visit
      pause_before: duration in seconds to pause before execution
      pause_after: duration in seconds to pause after execution:
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
    logger.info(
        "Add a goal <{}> of type {}, path {}, pause before {}, pause after {}, vertex {}."
        .format(goal_uuid_str, goal_type, path, pause_before, pause_after,
                vertex))
    # send the goal (async)
    send_goal_future = self._action.send_goal_async(
        goal, feedback_callback=self.feedback_callback, goal_uuid=goal_uuid)
    send_goal_future.add_done_callback(self.response_callback)

    return goal_uuid_str

  @ROSManager.on_ros
  def cancel_all(self):
    """Cancels a goal inside the ROS process
    Args:
      goal_id (str): goal id to be cancelled
    """
    # This is safe, because the callbacks that modify _queue block until this function returns and _lock is released
    for k, v in reversed(self._goals.items()):
      self.cancel_goal(k)
    return True

  @ROSManager.on_ros
  def cancel_goal(self, goal_uuid_str):
    """Cancels a goal inside the ROS process
    Args:
      goal_id_str (str): goal id to be cancelled
    """
    # Check to make sure we are still tracking this goal
    if goal_uuid_str not in self._goals.keys():
      logger.info("Not tracking goal <{}>, not cancel.".format(goal_uuid_str))
      return False

    # Cancel the goal
    logger.info("Cancel goal <{}>".format(goal_uuid_str))
    self._goals[goal_uuid_str].cancel_goal_async()
    return True

  @ROSManager.on_ros
  def feedback_callback(self, feedback_handle):
    feedback = feedback_handle.feedback
    goal_uuid_str = str(uuid.UUID(bytes=feedback_handle.goal_id.uuid.tobytes()))

    if not goal_uuid_str in self._goals.keys():
      return

    old = (self._feedback[goal_uuid_str]
           if goal_uuid_str in self._feedback.keys() else None)
    # update feedback and sync with goalinfos
    self._feedback[goal_uuid_str] = feedback
    self._goalinfos[goal_uuid_str].update(msg2dict(feedback))
    if not old:
      if feedback.in_progress == True:  # a goal may start immediately on first feedback
        self.notify(self.Notification.Started, goal_uuid_str)
      self.notify(self.Notification.Feedback, goal_uuid_str, msg2dict(feedback))

      logger.info(
          "Goal <{}> gets first feedback: in_progress {}, waiting {}, and percent complete {}"
          .format(goal_uuid_str, feedback.in_progress, feedback.waiting,
                  feedback.percent_complete))
    else:
      if old.in_progress != feedback.in_progress:  # delayed start of a goal
        self.notify(self.Notification.Started, goal_uuid_str)

      if (old.percent_complete != feedback.percent_complete or
          old.waiting != feedback.waiting):
        self.notify(self.Notification.Feedback, goal_uuid_str,
                    msg2dict(feedback))

      logger.info(
          "Goal <{}> gets updated feedback: in_progress {}, waiting {}, and percent complete {}"
          .format(goal_uuid_str, feedback.in_progress, feedback.waiting,
                  feedback.percent_complete))

  @ROSManager.on_ros
  def response_callback(self, future):
    """Callback when the new goal has been processed by the mission server,
    accepted or rejected.
    """
    goal_handle = future.result()
    goal_uuid_str = str(uuid.UUID(bytes=goal_handle.goal_id.uuid.tobytes()))
    if goal_handle.accepted:
      logger.info('Goal <{}> has been accepted.'.format(goal_uuid_str))
      self._goals[goal_uuid_str] = goal_handle
      get_result_future = goal_handle.get_result_async()
      get_result_future.add_done_callback(
          lambda future, uuid=goal_uuid_str: self.get_result_callback(
              uuid, future))
      # TODO yuchen is it OK to notify here?
      self.notify(self.Notification.NewGoal,
                  goalinfo2dict(self._goalinfos[goal_uuid_str]))
    else:
      logger.info('Goal <{}> has been rejected.'.format(goal_uuid_str))
      del self._goalinfos[goal_uuid_str]

  @ROSManager.on_ros
  def get_result_callback(self, uuid, future):
    result = future.result().result
    status = future.result().status

    if not uuid in self._goals.keys():
      return

    # Goal has completed successfully
    if status == GoalStatus.STATUS_SUCCEEDED:
      logger.info('Goal <{}> succeeded: {}'.format(uuid, result))
      self.notify(self.Notification.Complete, uuid)
    # Goal was cancelled by the user
    elif status == GoalStatus.STATUS_CANCELED:
      logger.info("Goal <{}> cancelled.".format(uuid))
      self.notify(self.Notification.Cancel, uuid)
    # Goal has failed for an internal reason
    elif status == GoalStatus.STATUS_ABORTED:
      logger.info("Goal <{}> aborted.".format(uuid))
      self.notify(self.Notification.Error, uuid, status)
    else:
      logger.error("Goal <{}> in invalid state.".format(uuid))
      raise RuntimeError("Goal <{}> in invalid state.".format(uuid))

    # If the goal is in a terminal state, remove it from being tracked
    if status in [
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    ]:
      logger.info("Goal <{}> deleted from queue.".format(uuid))
      del self._goals[uuid]
      del self._goalinfos[uuid]
      if uuid in self._feedback.keys():
        del self._feedback[uuid]

  @ROSManager.on_ros
  def robot_callback(self, msg):
    """Callback when a new robot position is received"""

    logger.info("Received robot update.")

    self._path_seq = msg.path_seq

    self._trunk_vertex = msg.trunk_vertex
    self._trunk_lng_lat_theta = list(msg.lng_lat_theta)
    self._t_leaf_trunk = [
        msg.t_leaf_trunk.x,
        msg.t_leaf_trunk.y,
        msg.t_leaf_trunk.theta,
    ]
    self._cov_leaf_trunk = list(msg.cov_leaf_trunk)

    self._target_vertex = msg.target_vertex
    self._target_lng_lat_theta = list(msg.target_lng_lat_theta)
    self._t_leaf_target = [
        msg.t_leaf_target.x,
        msg.t_leaf_target.y,
        msg.t_leaf_target.theta,
    ]
    self._cov_leaf_target = list(msg.cov_leaf_target)

    self.notify(
        self.Notification.RobotChange,
        self._path_seq,
        self._trunk_vertex,
        self._trunk_lng_lat_theta,
        self._t_leaf_trunk,
        self._cov_leaf_trunk,
        self._target_vertex,
        self._target_lng_lat_theta,
        self._t_leaf_target,
        self._cov_leaf_target,
    )

  @ROSManager.on_ros
  def graph_callback(self, msg):
    """Callback for incremental graph updates"""

    logger.info("Received graph update.")

    vals = {
        'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        'seq': msg.seq,
        'invalidate': msg.invalidate,
        'vertices': [vertex_to_json(v) for v in msg.vertices]
    }

    self.notify(self.Notification.GraphChange, vals)

  @ROSManager.on_ros
  def path_callback(self, msg):

    logger.info("Received repeat path update.")

    self._path = list(msg.vertex_id_list)

    self.notify(self.Notification.PathChange, self._path)

  @property
  @ROSManager.on_ros
  def status(self):
    """Get the current mission server status"""
    return {
        MissionStatus.PROCESSING: "PROCESSING",
        MissionStatus.PAUSED: "PAUSED",
        MissionStatus.PENDING_PAUSE: "PENDING_PAUSE",
        MissionStatus.EMPTY: "EMPTY"
    }.get(self._status, "")

  @property
  @ROSManager.on_ros
  def goals(self):
    """Get the current mission server status"""
    goals = []
    for uuid in self._goals.keys():
      goals.append(goalinfo2dict(self._goalinfos[uuid]))
    return goals

  @property
  @ROSManager.on_ros
  def path(self):
    return self._path

  @property
  @ROSManager.on_ros
  def path_seq(self):
    return self._path_seq

  @property
  @ROSManager.on_ros
  def trunk_vertex(self):
    return self._trunk_vertex

  @property
  @ROSManager.on_ros
  def trunk_lng_lat_theta(self):
    return self._trunk_lng_lat_theta

  @property
  @ROSManager.on_ros
  def t_leaf_trunk(self):
    return self._t_leaf_trunk

  @property
  @ROSManager.on_ros
  def cov_leaf_trunk(self):
    return self._cov_leaf_trunk

  @property
  @ROSManager.on_ros
  def target_vertex(self):
    return self._target_vertex

  @property
  @ROSManager.on_ros
  def target_lng_lat_theta(self):
    return self._target_lng_lat_theta

  @property
  @ROSManager.on_ros
  def t_leaf_target(self):
    return self._t_leaf_target

  @property
  @ROSManager.on_ros
  def cov_leaf_target(self):
    return self._cov_leaf_target


if __name__ == "__main__":

  import time

  logger.setLevel(logging.INFO)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("INITIALIZE AND START MISSION CLIENT")
  mc = MissionClient()
  mc.start()
  time.sleep(1)
  logger.info("INITIALIZE AND START MISSION CLIENT - DONE")

  logger.info("UNPAUSING THE SYSTEM")
  mc.set_pause(False)
  time.sleep(1)
  logger.info("UNPAUSING THE SYSTEM - DONE")

  logger.info("ADD AN IDLE GOAL, CANCEL AFTER SUCCEEDED")
  uuid_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(5)
  time.sleep(1)
  mc.cancel_goal(uuid_str)  # no goal to cancel as it has succeeded
  time.sleep(1)
  logger.info("ADD AN IDLE GOAL, CANCEL AFTER SUCCEEDED - DONE")

  logger.info("Add AN IDLE GOAL, CANCEL BEFORE IT IS ACCEPTED")
  uuid_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  mc.cancel_goal(uuid_str)  # no goal to cancel as it has not been accepted
  time.sleep(5)  # wait until the goal has been finished
  time.sleep(1)
  logger.info("Add AN IDLE GOAL, CANCEL BEFORE IT IS ACCEPTED - DONE")

  logger.info("ADD AN IDLE GOAL, CANCEL BEFORE IT IS EXECUTED")
  uuid_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(1.5)
  mc.cancel_goal(uuid_str)
  time.sleep(1)
  logger.info("ADD AN IDLE GOAL, CANCEL BEFORE IT IS EXECUTED - DONE")

  logger.info("ADD AN IDLE GOAL, CANCEL AFTER EXECUTED BEFORE FINISHED")
  uuid_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(2.5)
  mc.cancel_goal(uuid_str)
  time.sleep(1)
  logger.info("ADD AN IDLE GOAL, CANCEL AFTER EXECUTED BEFORE FINISHED - DONE")

  logger.info("ADD TWO IDLE GOALS, CANCEL AFTER SUCCEEDED")
  uuid0_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  uuid1_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(10)
  time.sleep(1)
  mc.cancel_goal(uuid0_str)  # no goal to cancel as both have succeeded
  mc.cancel_goal(uuid1_str)
  logger.info("ADD TWO IDLE GOALS, CANCEL AFTER SUCCEEDED - DONE")

  logger.info("ADD TWO IDLE GOALS, CANCEL FIRST BEFORE FINISHED")
  uuid0_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  uuid1_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(2.5)
  mc.cancel_goal(uuid0_str)  # first goal canceled
  time.sleep(7.5)
  time.sleep(1)
  logger.info("ADD TWO IDLE GOALS, CANCEL FIRST BEFORE FINISHED - DONE")

  logger.info("ADD TWO IDLE GOALS, CANCEL SECOND BEFORE STARTS")
  uuid0_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  uuid1_str = mc.add_goal(Mission.Goal.IDLE, (), 1, 1)
  time.sleep(3)
  mc.cancel_goal(uuid1_str)  # no goal to cancel as it is finished
  time.sleep(7)
  time.sleep(1)
  logger.info("ADD TWO IDLE GOALS, CANCEL SECOND BEFORE STARTS - DONE")

  # Shut down
  mc.shutdown()
