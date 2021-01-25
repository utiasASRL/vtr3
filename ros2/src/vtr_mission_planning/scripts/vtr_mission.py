#!/usr/bin/env python
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
