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
from vtr_mission_planning.mission_client import MissionClient
from builtin_interfaces.msg import Duration
from unique_identifier_msgs.msg import UUID
from vtr_messages.action import Mission
from vtr_messages.srv import MissionPause
from vtr_messages.msg import MissionStatus

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
