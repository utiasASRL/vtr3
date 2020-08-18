#!/usr/bin/env python

import logging
import argparse

from vtr_planning.msg import MissionGoal as Goal

import vtr_planning.client_proxy as client_proxy
remote_client = client_proxy.remote_client

logging.basicConfig(level=logging.DEBUG)

parser = argparse.ArgumentParser(description='Add a new goal')
parser.add_argument('action',
                    type=str,
                    help='One of {Teach, Repeat, Idle, Merge, Localize}')
parser.add_argument('path',
                    type=int,
                    nargs='*',
                    metavar='vertex',
                    help='A list of vertices to visit, in order')
parser.add_argument('-v',
                    dest='vertex',
                    type=int,
                    nargs='?',
                    metavar='vertex',
                    help='Target vertex',
                    default=2**64 - 1)
parser.add_argument('-b',
                    dest='before',
                    type=float,
                    nargs='?',
                    metavar='before',
                    help='Duration to pause before execution [s]',
                    default=0.0)
parser.add_argument('-a',
                    dest='after',
                    type=float,
                    nargs='?',
                    metavar='after',
                    help='Duration to pause after execution [s]',
                    default=0.0)

# Do not remove this line.  Just don't.  It will cause fork bombs.
if __name__ == "__main__":
  args = parser.parse_args()

  if args.action.lower() == "idle":
    gid = remote_client().add_goal(Goal.IDLE,
                                   pause_before=args.before,
                                   pause_after=args.after)
  elif args.action.lower() == "teach":
    gid = remote_client().add_goal(Goal.TEACH,
                                   pause_before=args.before,
                                   pause_after=args.after)
  elif args.action.lower() == "repeat":
    if len(args.path) == 0:
      raise RuntimeError("Cannot issue a repeat command with an empty path")
    gid = remote_client().add_goal(Goal.REPEAT,
                                   path=args.path,
                                   pause_before=args.before,
                                   pause_after=args.after)
  elif args.action.lower() == "merge":
    gid = remote_client().add_goal(Goal.MERGE,
                                   path=args.path,
                                   vertex=args.vertex,
                                   pause_before=args.before,
                                   pause_after=args.after)
  elif args.action.lower() == "localize":
    gid = remote_client().add_goal(Goal.LOCALIZE,
                                   path=args.path,
                                   vertex=args.vertex,
                                   pause_before=args.before,
                                   pause_after=args.after)
  else:
    raise RuntimeError("Unrecognized goal type: %s", args.action)

  if gid is None or gid == "":
    logging.error("Server returned an empty goal id!")
  else:
    logging.info("Successfully added goal %s", gid)
