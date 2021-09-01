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
"""
Basic command line tool to send command to the mission server. Need master
client running.
"""

import time
import sys

from vtr_mission_planning.ros_manager import RosManager
from vtr_mission_planning.mission_client import MissionClient
from vtr_mission_planning.mission_client_builder import build_remote_client
from vtr_mission_planning.cmd_util import ArgParser
from vtr_messages.action import Mission
from vtr_messages.srv import MissionPause
from vtr_messages.msg import MissionStatus


def main(target=None, path=(), before=0, after=0, start_vertex=0, **_):
  mc = build_remote_client()
  if target == "start":
    mc.set_pause(False)
  elif target == "pause":
    mc.set_pause(True)
  elif target == "cancel":
    mc.cancel_all()
  elif target == "idle":
    uuid = mc.add_goal(Mission.Goal.IDLE, path, before, after)
  elif target == "teach":
    uuid = mc.add_goal(Mission.Goal.TEACH, path, before, after)
  elif target == "repeat":
    uuid = mc.add_goal(Mission.Goal.REPEAT, path, before, after)
  else:
    print("Unknown command. Doing nothing.")


if __name__ == "__main__":

  exp_parser = ArgParser(allow_unknown_args=False)
  exp_parser.parser.add_argument(
      "--target",
      help=
      "choose between: [start (i.e. pause=False), pause, cancel, idle, teach, repeat]",
      type=str,
      default=None,
  )
  exp_parser.parser.add_argument(
      "--path",
      help="vertex IDs",
      type=int,
      nargs='+',
      default=[0, 0],
  )
  exp_parser.parser.add_argument(
      "--before",
      help="wait before (s)",
      type=int,
      default=0,
  )
  exp_parser.parser.add_argument(
      "--after",
      help="wait after (s)",
      type=int,
      default=0,
  )
  exp_parser.parse(sys.argv)
  main(**exp_parser.get_dict())
