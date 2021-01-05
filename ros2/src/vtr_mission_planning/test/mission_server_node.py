#!/usr/bin/env python

import collections
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vtr_messages.action import Mission
from vtr_messages.srv import MissionPause


class TestMissionServer(Node):
  """Minimal action server that processes one goal at a time."""

  def __init__(self):
    super().__init__('test_mission_server')
    self._goal_queue = collections.deque()
    self._goal_queue_lock = threading.Lock()
    self._current_goal = None

    self._pause_service = self.create_service(MissionPause, 'pause',
                                              self.pause_callback)

    self._action_server = ActionServer(
        self,
        Mission,
        'manager',
        goal_callback=self.goal_callback,
        handle_accepted_callback=self.handle_accepted_callback,
        execute_callback=self.execute_callback,
        cancel_callback=self.cancel_callback,
        callback_group=ReentrantCallbackGroup())

  def pause_callback(self, request, response):
    self.get_logger().info('Incoming request: ' + str(request.pause))
    return response

  def goal_callback(self, goal_request):
    """Accept or reject a client request to begin an action."""
    self.get_logger().info('Received goal request')
    return GoalResponse.ACCEPT

  def handle_accepted_callback(self, goal_handle):
    """Start or defer execution of an already accepted goal."""
    with self._goal_queue_lock:
      if self._current_goal is not None:
        # Put incoming goal in the queue
        self._goal_queue.append(goal_handle)
        self.get_logger().info('Goal put in the queue')
      else:
        # Start goal execution right away
        self._current_goal = goal_handle
        self._current_goal.execute()

  def cancel_callback(self, goal):
    """Accept or reject a client request to cancel an action."""
    self.get_logger().info('Received cancel request')
    return CancelResponse.ACCEPT

  def execute_callback(self, goal_handle):
    """Execute the goal."""
    try:
      self.get_logger().info('Executing goal...')

      # Append the seeds for the Mission sequence
      feedback = Mission.Feedback()
      # feedback

      # Start executing the action
      for _ in range(2):
        if goal_handle.is_cancel_requested:
          goal_handle.canceled()
          self.get_logger().info('Goal canceled')
          return Mission.Result()

        # Update feedback message
        # feedback.sequence.append(feedback.sequence[i] +
        #                              feedback.sequence[i - 1])

        # Publish the feedback
        self.get_logger().info('Publishing feedback: {0}'.format(feedback))
        goal_handle.publish_feedback(feedback)

        # Sleep for demonstration purposes
        time.sleep(1)

      goal_handle.succeed()

      # Populate result message
      result = Mission.Result()
      # result.sequence = feedback.sequence

      self.get_logger().info('Returning result: {0}'.format(result))

      return result
    finally:
      with self._goal_queue_lock:
        try:
          # Start execution of the next goal in the queue.
          self._current_goal = self._goal_queue.popleft()
          self.get_logger().info('Next goal pulled from the queue')
          self._current_goal.execute()
        except IndexError:
          # No goal in the queue.
          self._current_goal = None

  def destroy(self):
    self._action_server.destroy()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)

  action_server = TestMissionServer()

  # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
  executor = MultiThreadedExecutor()
  rclpy.spin(action_server, executor=executor)

  action_server.destroy()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
