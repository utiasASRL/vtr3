#!/usr/bin/env python

import logging
import argparse

import vtr_planning.client_proxy as client_proxy
remote_client = client_proxy.remote_client

logging.basicConfig(level=logging.DEBUG)

parser = argparse.ArgumentParser(description='Cancel a goal')
parser.add_argument('index',
                    type=int,
                    nargs='?',
                    help='Target goal index',
                    default=None)
parser.add_argument('-i',
                    type=str,
                    nargs='?',
                    metavar='id',
                    dest='id',
                    help='Target goal id',
                    default=None)

# Do not remove this line.  Just don't.  It will cause fork bombs.
if __name__ == "__main__":
  args = parser.parse_args()

  client = remote_client()

  if args.index is None and args.id is None:
    res = client.cancel_all()
  elif args.id is not None:
    res = client.cancel_goal(args.id)
  else:
    goals = client.goals
    if args.index < 0 or args.index >= len(goals):
      raise RuntimeError("Goal index is out of range!")

    res = client.cancel_goal(goals[args.index]['id'])

  if res:
    logging.info("Goal cancelled.")
  else:
    logging.error("Goal could not be cancelled!")
