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
"""Defines a test mission server for testing the mission client."""

import collections
import logging
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vtr_messages.action import Mission
from vtr_messages.srv import MissionPause
from vtr_messages.msg import MissionStatus, RobotStatus, GraphUpdate, GraphPath

logger = logging.getLogger('MissionServer')


class TestMissionServer(Node):
  """Minimal action server that processes one goal at a time."""

  def __init__(self):
    super().__init__('test_mission_server')
    # yapf: disable

    # start/pause server for starting and pausing the navigation system
    self._pause_service = self.create_service(MissionPause, 'pause', self.pause_callback)

    self._current_goal = None
    self._goal_queue_lock = threading.Lock()
    self._goal_queue = collections.deque()
    self._action_server = ActionServer(
        self, Mission, 'manager',
        goal_callback=self.goal_callback,
        handle_accepted_callback=self.handle_accepted_callback,
        execute_callback=self.execute_callback,
        cancel_callback=self.cancel_callback,
        callback_group=ReentrantCallbackGroup(),
    )

    # yapf: enable

  def pause_callback(self, request, response):
    # print("check this {}".format(request.pause))
    logger.info("Incoming pause request: pause=%s", request.pause)
    response.response_code = response.SUCCESS
    return response

  def goal_callback(self, goal_request):
    """Accept or reject a client request to begin an action."""
    logger.info('Received goal request - accept it')
    time.sleep(1)  # simulate a delay before actually accepting the goal
    return GoalResponse.ACCEPT

  def handle_accepted_callback(self, goal_handle):
    """Start or defer execution of an already accepted goal."""
    with self._goal_queue_lock:
      if self._current_goal is not None:
        # Put incoming goal in the queue
        self._goal_queue.append(goal_handle)
        logger.info('Goal put in the queue')
      else:
        # Start goal execution right away
        logger.info('Goal executed right away')
        self._current_goal = goal_handle
        self._current_goal.execute()

  def cancel_callback(self, goal):
    """Accept or reject a client request to cancel an action."""
    logger.info('Received cancel request')
    return CancelResponse.ACCEPT

  def execute_callback(self, goal_handle):
    """Execute the goal."""
    time.sleep(1)
    try:
      logger.info('Executing goal...')

      # Append the seeds for the Mission sequence
      feedback = Mission.Feedback()

      # Start executing the action
      total_time = 3
      for i in range(total_time):
        if goal_handle.is_cancel_requested:
          goal_handle.canceled()
          logger.info('Goal canceled')
          return Mission.Result()

        # Update feedback message
        feedback.in_progress = True
        feedback.waiting = False
        feedback.percent_complete = i / total_time

        # Publish the feedback
        logger.info('Publishing feedback: {}'.format(feedback))
        goal_handle.publish_feedback(feedback)

        # Sleep for demonstration purposes
        time.sleep(1)

      goal_handle.succeed()  # populate status
      result = Mission.Result()  # populate result message
      logger.info('Returning result: {}'.format(result))

      return result
    finally:
      with self._goal_queue_lock:
        try:
          # Start execution of the next goal in the queue.
          self._current_goal = self._goal_queue.popleft()
          logger.info('Next goal pulled from the queue')
          self._current_goal.execute()
        except IndexError:
          # No goal in the queue.
          self._current_goal = None


if __name__ == '__main__':

  import time

  logger.setLevel(logging.DEBUG)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  rclpy.init()
  action_server = TestMissionServer()
  # use a two-thread executor so that a goal can be cancelled while being
  # executed.
  rclpy.spin(action_server, executor=MultiThreadedExecutor(2))
  rclpy.shutdown()
